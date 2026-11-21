//! Simulated calibration harness.
//!
//! Generates RSSI and compass observations for a floor plan with known transmitter positions,
//! runs the ranging, bearing and fusion chain, and scores the estimates. The noise and
//! path-loss model are the same ones the algorithms assume, so the numbers describe the model
//! only. They are not real-world accuracy.
//!
//! Scoring: distance error is averaged over every post-warmup sample. Bearing and position
//! error are scored on the estimate after a full turn, because partial turns are biased.
//! Each trial is one full turn with its own seed, so the statistics cover many turns.

use crate::algorithm::Algorithm;
use crate::bearing::BearingEstimator;
use crate::fusion::Fusion;
use crate::pipes::Pipeline;
use crate::ranging::RssiRanging;
use serde::Deserialize;
use spi::{RadioSample, Transport};
use std::f64::consts::PI;
use std::fmt::Write as _;
use spi::ManualFacing;
use std::sync::Arc;
use std::time::Duration;

const MIN_ANCHOR_DISTANCE_M: f64 = 0.5;
const HEADING_WINDOW: usize = 64;
const SMOOTHING: f64 = 0.3;

#[derive(Debug, Clone, Deserialize)]
pub struct FloorPlan {
    pub room: Room,
    pub sensor: Sensor,
    pub path_loss: PathLoss,
    pub simulation: Simulation,
    pub anchors: Vec<Anchor>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Room {
    pub width_m: f64,
    pub depth_m: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Sensor {
    pub position_m: [f64; 2],
    pub heading_noise_deg: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct PathLoss {
    pub tx_power_dbm: f64,
    pub exponent: f64,
    pub rssi_noise_db: f64,
    pub directivity_db: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Simulation {
    pub seed: u64,
    pub sweep_steps: usize,
    pub warmup_steps: usize,
    pub trials: usize,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Anchor {
    pub id: String,
    pub position_m: [f64; 2],
}

impl FloorPlan {
    pub fn from_toml(text: &str) -> Result<Self, String> {
        let plan: FloorPlan = toml::from_str(text).map_err(|e| e.to_string())?;
        plan.validate()?;
        Ok(plan)
    }

    fn validate(&self) -> Result<(), String> {
        if self.anchors.is_empty() {
            return Err("at least one anchor is required".into());
        }
        if self.path_loss.exponent <= 0.0 {
            return Err("path_loss.exponent must be positive".into());
        }
        if self.simulation.sweep_steps == 0 || self.simulation.warmup_steps >= self.simulation.sweep_steps {
            return Err("simulation.warmup_steps must be less than sweep_steps".into());
        }
        if self.simulation.trials == 0 {
            return Err("simulation.trials must be at least 1".into());
        }
        let inside_room = |[x, y]: [f64; 2]| {
            (0.0..=self.room.width_m).contains(&x) && (0.0..=self.room.depth_m).contains(&y)
        };
        if !inside_room(self.sensor.position_m) {
            return Err("sensor.position_m must be inside the room".into());
        }
        for anchor in &self.anchors {
            if !inside_room(anchor.position_m) {
                return Err(format!("anchor {} is outside the room", anchor.id));
            }
            if distance(anchor.position_m, self.sensor.position_m) < MIN_ANCHOR_DISTANCE_M {
                return Err(format!("anchor {} is too close to the sensor", anchor.id));
            }
        }
        Ok(())
    }
}

/// Error statistics for one transmitter.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceReport {
    pub id: String,
    pub distance_mae_m: f64,
    pub bearing_mae_deg: f64,
    pub position_mean_m: f64,
    pub position_p90_m: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Report {
    pub devices: Vec<DeviceReport>,
    pub position_mean_m: f64,
    pub position_p90_m: f64,
}

/// Runs every trial with its own seed. The same plan always gives the same report.
pub fn run(plan: &FloorPlan) -> Report {
    let truths: Vec<Truth> = plan
        .anchors
        .iter()
        .map(|anchor| Truth::new(anchor, plan.sensor.position_m))
        .collect();
    let mut errors: Vec<Errors> = vec![Errors::default(); truths.len()];

    for trial in 0..plan.simulation.trials {
        let seed = plan.simulation.seed.wrapping_add(trial as u64);
        simulate_turn(plan, &truths, seed, &mut errors);
    }

    let devices: Vec<DeviceReport> = truths
        .iter()
        .zip(errors.iter())
        .map(|(truth, errs)| DeviceReport {
            id: truth.id.clone(),
            distance_mae_m: mean(&errs.distance),
            bearing_mae_deg: mean(&errs.bearing),
            position_mean_m: mean(&errs.position),
            position_p90_m: percentile(&errs.position, 0.9),
        })
        .collect();
    let all_positions: Vec<f64> = errors.iter().flat_map(|e| e.position.iter().copied()).collect();

    Report {
        devices,
        position_mean_m: mean(&all_positions),
        position_p90_m: percentile(&all_positions, 0.9),
    }
}

/// One full turn of the sensor. Appends the errors of this turn to `errors`.
fn simulate_turn(plan: &FloorPlan, truths: &[Truth], seed: u64, errors: &mut [Errors]) {
    let mut rng = SplitMix64::new(seed);
    let facing = Arc::new(ManualFacing::new());
    let algorithms: Vec<Box<dyn Algorithm>> = vec![
        Box::new(RssiRanging::new(
            plan.path_loss.tx_power_dbm,
            plan.path_loss.exponent,
            SMOOTHING,
        )),
        Box::new(BearingEstimator::new(facing.clone(), HEADING_WINDOW)),
        Box::new(Fusion::default()),
    ];
    let mut pipeline = Pipeline::new(algorithms);
    let steps = plan.simulation.sweep_steps;
    let mut latest = None;

    for step in 0..steps {
        let true_heading = step as f64 * 360.0 / steps as f64;
        let observed_heading = true_heading + rng.gaussian() * plan.sensor.heading_noise_deg;
        facing.set(Some(observed_heading.rem_euclid(360.0)));

        for (truth, errs) in truths.iter().zip(errors.iter_mut()) {
            let facing_gain = plan.path_loss.directivity_db
                * (true_heading - truth.bearing_deg).to_radians().cos();
            let mean_rssi = plan.path_loss.tx_power_dbm
                - 10.0 * plan.path_loss.exponent * truth.distance_m.log10()
                + facing_gain;
            let noisy = mean_rssi + rng.gaussian() * plan.path_loss.rssi_noise_db;
            let sample = RadioSample {
                transport: Transport::BluetoothLe,
                device_id: truth.id.clone(),
                rssi_dbm: noisy.round().clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16,
                observed_at: Duration::from_millis(step as u64 * 100),
            };

            let frame = pipeline.process(&sample);
            if step >= plan.simulation.warmup_steps {
                if let Some(d) = frame.device(&truth.id).and_then(|d| d.distance_m) {
                    errs.distance.push((d - truth.distance_m).abs());
                }
            }
            latest = Some(frame);
        }
    }

    // The last frame holds every device's estimate after the full turn.
    let Some(final_frame) = latest else {
        return;
    };
    for (truth, errs) in truths.iter().zip(errors.iter_mut()) {
        let Some(device) = final_frame.device(&truth.id) else {
            continue;
        };
        if let Some(b) = device.bearing_deg {
            errs.bearing.push(angle_gap(b, truth.bearing_deg));
        }
        if let Some((east, north)) = device.position_m {
            errs.position.push(distance_pair((east, north), truth.relative_m));
        }
    }
}

/// Renders a report as a text table, labelled as simulated output.
pub fn format_report(report: &Report) -> String {
    let mut out = String::from("SIMULATED results: model-based, not real-world accuracy\n\n");
    out.push_str("Device                        Dist MAE (m)  Bearing MAE (deg)  Pos mean (m)  Pos p90 (m)\n");
    for d in &report.devices {
        let _ = writeln!(
            out,
            "{:<29} {:>12.2} {:>18.1} {:>13.2} {:>12.2}",
            d.id, d.distance_mae_m, d.bearing_mae_deg, d.position_mean_m, d.position_p90_m
        );
    }
    let _ = writeln!(
        out,
        "\nAll devices: position mean {:.2} m, p90 {:.2} m",
        report.position_mean_m, report.position_p90_m
    );
    out
}

#[derive(Clone, Default)]
struct Errors {
    distance: Vec<f64>,
    bearing: Vec<f64>,
    position: Vec<f64>,
}

struct Truth {
    id: String,
    distance_m: f64,
    bearing_deg: f64,
    relative_m: (f64, f64),
}

impl Truth {
    fn new(anchor: &Anchor, sensor: [f64; 2]) -> Self {
        let relative = (anchor.position_m[0] - sensor[0], anchor.position_m[1] - sensor[1]);
        Self {
            id: anchor.id.clone(),
            distance_m: distance(anchor.position_m, sensor),
            bearing_deg: relative.0.atan2(relative.1).to_degrees().rem_euclid(360.0),
            relative_m: relative,
        }
    }
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    distance_pair((a[0], a[1]), (b[0], b[1]))
}

fn distance_pair(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

fn angle_gap(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(360.0);
    d.min(360.0 - d)
}

fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        return f64::NAN;
    }
    values.iter().sum::<f64>() / values.len() as f64
}

fn percentile(values: &[f64], p: f64) -> f64 {
    if values.is_empty() {
        return f64::NAN;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.total_cmp(b));
    let rank = (p * sorted.len() as f64).ceil() as usize;
    sorted[rank.clamp(1, sorted.len()) - 1]
}

/// Small deterministic generator, so results do not depend on an external RNG crate.
struct SplitMix64(u64);

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Standard normal sample (Box-Muller).
    fn gaussian(&mut self) -> f64 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXAMPLE: &str = include_str!("../../calibration/floorplan.example.toml");

    #[test]
    fn example_plan_runs_with_finite_fusion_error() {
        let plan = FloorPlan::from_toml(EXAMPLE).unwrap();
        let report = run(&plan);
        assert_eq!(report.devices.len(), plan.anchors.len());
        assert!(report.position_mean_m.is_finite());
        assert!(report.position_p90_m.is_finite());
        assert!(report.devices.iter().all(|d| d.position_p90_m.is_finite()));
    }

    #[test]
    fn fixed_seed_is_reproducible() {
        let plan = FloorPlan::from_toml(EXAMPLE).unwrap();
        assert_eq!(run(&plan), run(&plan));
    }
}
