//! Survey: positions from distances measured at spots the user stands on, in turn.
//!
//! The user marks a spot, stays there while samples arrive, then moves and marks the next spot.
//! With three or more spots that are not on one line, each device's position is solved by
//! weighted least squares over the per-spot mean distances. The first spot is the origin.

use crate::algorithm::Algorithm;
use crate::model::{PlacementMethod, ScreenFrame};
use spi::RadioSample;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};

/// Fewest spots that can fix a position in the plane.
pub const MIN_SPOTS: usize = 3;
/// Distances are clamped to this before weighting, so a near-zero estimate cannot dominate.
const MIN_RANGE_M: f64 = 0.5;
/// Normalised spread of the spot directions below this means the spots are on one line.
const COLLINEAR_EPS: f64 = 1e-3;
const MAX_ITERATIONS: usize = 20;

/// One marked spot and the distance estimates recorded while standing on it.
#[derive(Debug, Clone, Default)]
pub struct SurveySpot {
    /// Location in the survey's frame (metres east, north), as clicked on the map.
    pub position_m: (f64, f64),
    /// Per device: (sum of distance estimates, count).
    distances: HashMap<String, (f64, u32)>,
}

impl SurveySpot {
    /// Mean distance estimate for a device at this spot, if any was recorded.
    pub fn mean_distance(&self, device_id: &str) -> Option<f64> {
        let (sum, count) = self.distances.get(device_id)?;
        (*count > 0).then(|| sum / f64::from(*count))
    }
}

/// Spots the user has marked, in order. Lives outside the pipeline so it survives pause and resume.
#[derive(Debug, Default)]
pub struct Survey {
    spots: Vec<SurveySpot>,
}

impl Survey {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn spots(&self) -> &[SurveySpot] {
        &self.spots
    }

    /// Starts a new spot. Samples recorded from now on belong to it.
    pub fn mark(&mut self, position_m: (f64, f64)) {
        self.spots.push(SurveySpot {
            position_m,
            ..Default::default()
        });
    }

    pub fn clear(&mut self) {
        self.spots.clear();
    }

    /// The first spot, which is the origin of every survey position.
    pub fn origin(&self) -> Option<(f64, f64)> {
        self.spots.first().map(|spot| spot.position_m)
    }

    fn record(&mut self, device_id: &str, distance_m: f64) {
        if let Some(spot) = self.spots.last_mut() {
            let entry = spot.distances.entry(device_id.to_string()).or_insert((0.0, 0));
            entry.0 += distance_m;
            entry.1 += 1;
        }
    }
}

/// A solved position relative to the survey origin, with its RMS range residual.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fix {
    pub position_m: (f64, f64),
    /// RMS difference between measured and fitted distances, in metres. Used as the uncertainty.
    pub residual_m: f64,
}

/// Weighted least-squares position from `(spot position, measured distance)` pairs.
///
/// Weights are 1/d², since RSSI distance error grows with distance. A linear solve gives a start,
/// and Gauss-Newton refines it. Returns `None` for fewer than three spots, coincident spots, or
/// spots on one line (their geometry cannot resolve the mirror image of a fix).
pub fn solve_position(ranges: &[((f64, f64), f64)]) -> Option<Fix> {
    if ranges.len() < MIN_SPOTS {
        return None;
    }
    let weights: Vec<f64> = ranges
        .iter()
        .map(|(_, d)| 1.0 / d.max(MIN_RANGE_M).powi(2))
        .collect();

    let ((x0, y0), d0) = ranges[0];
    // Subtracting the first range equation from the others leaves linear equations in (x, y):
    // 2(p_i - p_0) . x = |p_i|^2 - |p_0|^2 - d_i^2 + d_0^2.
    let mut normal = [[0.0_f64; 2]; 2];
    let mut rhs = [0.0_f64; 2];
    let mut directions = [[0.0_f64; 2]; 2];
    for (i, ((xi, yi), di)) in ranges.iter().enumerate().skip(1) {
        let (ax, ay) = (2.0 * (xi - x0), 2.0 * (yi - y0));
        let length = (ax * ax + ay * ay).sqrt();
        if length < 1e-9 {
            return None;
        }
        let unit = [ax / length, ay / length];
        for r in 0..2 {
            for c in 0..2 {
                directions[r][c] += unit[r] * unit[c];
            }
        }
        let b = xi * xi + yi * yi - x0 * x0 - y0 * y0 - di * di + d0 * d0;
        let w = weights[i];
        for r in 0..2 {
            for c in 0..2 {
                normal[r][c] += w * [ax, ay][r] * [ax, ay][c];
            }
            rhs[r] += w * [ax, ay][r] * b;
        }
    }
    let spread = directions[0][0] * directions[1][1] - directions[0][1] * directions[1][0];
    if spread < COLLINEAR_EPS {
        return None;
    }
    let det = normal[0][0] * normal[1][1] - normal[0][1] * normal[1][0];
    if det.abs() < 1e-12 {
        return None;
    }
    let mut x = (
        (normal[1][1] * rhs[0] - normal[0][1] * rhs[1]) / det,
        (normal[0][0] * rhs[1] - normal[1][0] * rhs[0]) / det,
    );

    for _ in 0..MAX_ITERATIONS {
        // Jacobian of range_i(x) = |x - p_i| is the unit vector from p_i to x.
        let (mut j11, mut j12, mut j22, mut g1, mut g2) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for (((px, py), d), w) in ranges.iter().zip(&weights) {
            let (dx, dy) = (x.0 - px, x.1 - py);
            let r = (dx * dx + dy * dy).sqrt().max(1e-9);
            let (jx, jy) = (dx / r, dy / r);
            let residual = r - d;
            j11 += w * jx * jx;
            j12 += w * jx * jy;
            j22 += w * jy * jy;
            g1 += w * jx * residual;
            g2 += w * jy * residual;
        }
        let det = j11 * j22 - j12 * j12;
        if det.abs() < 1e-12 {
            break;
        }
        let step = (-(j22 * g1 - j12 * g2) / det, -(j11 * g2 - j12 * g1) / det);
        x = (x.0 + step.0, x.1 + step.1);
        if step.0.hypot(step.1) < 1e-6 {
            break;
        }
    }

    if !x.0.is_finite() || !x.1.is_finite() {
        return None;
    }
    let sum_sq: f64 = ranges
        .iter()
        .map(|((px, py), d)| {
            let range = (x.0 - px).hypot(x.1 - py);
            (range - d).powi(2)
        })
        .sum();
    Some(Fix {
        position_m: x,
        residual_m: (sum_sq / ranges.len() as f64).sqrt(),
    })
}

/// Solves each device's position from its spots and publishes the same frame as the other chains.
pub struct SurveyLocator {
    survey: Arc<Mutex<Survey>>,
}

impl SurveyLocator {
    pub fn new(survey: Arc<Mutex<Survey>>) -> Self {
        Self { survey }
    }
}

impl Algorithm for SurveyLocator {
    fn name(&self) -> &'static str {
        "survey"
    }

    fn apply(&mut self, sample: &RadioSample, frame: &mut ScreenFrame) {
        let mut survey = lock(&self.survey);
        let Some(device) = frame
            .devices
            .iter_mut()
            .find(|d| d.device_id == sample.device_id)
        else {
            return;
        };
        let id = device.device_id.clone();
        if let Some(distance) = device.distance_m {
            survey.record(&id, distance);
        }

        let Some(origin) = survey.origin() else {
            return;
        };
        let ranges: Vec<((f64, f64), f64)> = survey
            .spots()
            .iter()
            .filter_map(|spot| {
                let mean = spot.mean_distance(&id)?;
                let (x, y) = spot.position_m;
                Some(((x - origin.0, y - origin.1), mean))
            })
            .collect();
        let Some(fix) = solve_position(&ranges) else {
            return;
        };

        let current = survey.spots().last().map_or(origin, |spot| spot.position_m);
        let (dx, dy) = (fix.position_m.0 - (current.0 - origin.0), fix.position_m.1 - (current.1 - origin.1));
        device.position_m = Some(fix.position_m);
        device.position_uncertainty_m = Some(fix.residual_m);
        device.bearing_deg = Some(dx.atan2(dy).to_degrees().rem_euclid(360.0));
        device.bearing_spread_deg = None;
        device.distance_m = Some(dx.hypot(dy));
        device.placement = PlacementMethod::Survey;
    }
}

fn lock(survey: &Mutex<Survey>) -> MutexGuard<'_, Survey> {
    survey.lock().expect("survey lock poisoned")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::DeviceState;
    use spi::Transport;
    use std::time::Duration;

    fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
        (a.0 - b.0).hypot(a.1 - b.1)
    }

    /// Ranges from `truth` to each spot, with fixed offsets standing in for measurement noise.
    fn noisy_ranges(truth: (f64, f64), spots: &[(f64, f64)], noise: &[f64]) -> Vec<((f64, f64), f64)> {
        spots
            .iter()
            .zip(noise.iter().cycle())
            .map(|(spot, e)| (*spot, dist(*spot, truth) + e))
            .collect()
    }

    #[test]
    fn recovers_position_from_three_noisy_spots() {
        let spots = [(0.0, 0.0), (6.0, 0.0), (0.0, 6.0)];
        let truth = (3.0, 2.0);
        let fix = solve_position(&noisy_ranges(truth, &spots, &[0.3, -0.2, 0.1])).unwrap();
        assert!(dist(fix.position_m, truth) < 0.5, "fix {:?}", fix.position_m);
        assert!(fix.residual_m > 0.0 && fix.residual_m < 0.5);
    }

    #[test]
    fn recovers_position_from_four_noisy_spots() {
        let spots = [(0.0, 0.0), (6.0, 0.0), (0.0, 6.0), (6.0, 6.0)];
        let truth = (2.0, 4.0);
        let fix = solve_position(&noisy_ranges(truth, &spots, &[0.25, -0.3, 0.15, -0.1])).unwrap();
        assert!(dist(fix.position_m, truth) < 0.5, "fix {:?}", fix.position_m);
    }

    #[test]
    fn exact_ranges_recover_exact_position() {
        let spots = [(0.0, 0.0), (6.0, 1.0), (1.0, 6.0)];
        let truth = (4.0, 3.0);
        let fix = solve_position(&noisy_ranges(truth, &spots, &[0.0])).unwrap();
        assert!(dist(fix.position_m, truth) < 1e-3);
        assert!(fix.residual_m < 1e-3);
    }

    #[test]
    fn fewer_than_three_spots_give_no_position() {
        let spots = [(0.0, 0.0), (6.0, 0.0)];
        assert_eq!(solve_position(&noisy_ranges((3.0, 2.0), &spots, &[0.0])), None);
    }

    #[test]
    fn collinear_spots_give_no_position() {
        let spots = [(0.0, 0.0), (3.0, 0.0), (6.0, 0.0), (9.0, 0.0)];
        assert_eq!(solve_position(&noisy_ranges((3.0, 2.0), &spots, &[0.0])), None);
    }

    #[test]
    fn coincident_spots_give_no_position() {
        let spots = [(0.0, 0.0), (0.0, 0.0), (6.0, 0.0)];
        assert_eq!(solve_position(&noisy_ranges((3.0, 2.0), &spots, &[0.0])), None);
    }

    fn device_frame(id: &str, distance_m: f64) -> ScreenFrame {
        let sample = RadioSample {
            transport: Transport::BluetoothLe,
            device_id: id.into(),
            rssi_dbm: -60,
            observed_at: Duration::from_secs(1),
            details: Default::default(),
        };
        let mut state = DeviceState::from_sample(&sample);
        state.distance_m = Some(distance_m);
        ScreenFrame {
            at: sample.observed_at,
            devices: vec![state],
        }
    }

    #[test]
    fn locator_places_device_from_marked_spots_and_publishes_bearing() {
        let survey = Arc::new(Mutex::new(Survey::new()));
        let spots = [(0.0, 0.0), (6.0, 0.0), (0.0, 6.0)];
        let truth = (3.0, 2.0);
        let mut locator = SurveyLocator::new(survey.clone());
        let sample = RadioSample {
            transport: Transport::BluetoothLe,
            device_id: "beacon".into(),
            rssi_dbm: -60,
            observed_at: Duration::from_secs(1),
            details: Default::default(),
        };
        for spot in spots {
            lock(&survey).mark(spot);
            let mut frame = device_frame("beacon", dist(spot, truth));
            locator.apply(&sample, &mut frame);
        }

        let mut frame = device_frame("beacon", dist((0.0, 6.0), truth));
        locator.apply(&sample, &mut frame);
        let device = &frame.devices[0];
        let (x, y) = device.position_m.expect("three spots give a position");
        assert!(dist((x, y), truth) < 1e-3);
        assert_eq!(device.placement, PlacementMethod::Survey);
        // From the current spot (0, 6) the device is south-east: bearing between 90 and 180.
        let bearing = device.bearing_deg.unwrap();
        assert!((90.0..180.0).contains(&bearing), "bearing {bearing}");
    }

    #[test]
    fn locator_waits_for_three_spots() {
        let survey = Arc::new(Mutex::new(Survey::new()));
        let mut locator = SurveyLocator::new(survey.clone());
        let sample = RadioSample {
            transport: Transport::BluetoothLe,
            device_id: "beacon".into(),
            rssi_dbm: -60,
            observed_at: Duration::from_secs(1),
            details: Default::default(),
        };
        for spot in [(0.0, 0.0), (6.0, 0.0)] {
            lock(&survey).mark(spot);
            let mut frame = device_frame("beacon", 4.0);
            locator.apply(&sample, &mut frame);
            assert_eq!(frame.devices[0].position_m, None);
        }
    }
}
