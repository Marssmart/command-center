//! 3D view: devices placed in east/north/height space and projected with a draggable orbit.

use crate::app::nearest_click;
use core::ScreenFrame;
use eframe::egui::{self, Color32, Pos2, Sense, Stroke};

const SCALE_PX_PER_M: f32 = 15.0;
const GRID_HALF_SIZE_M: f32 = 20.0;
const GRID_STEP_M: f32 = 5.0;
const MAX_HEIGHT_M: f32 = 10.0;
const PITCH_LIMITS: (f32, f32) = (0.05, 1.5);

/// Camera orientation: yaw turns around the vertical axis, pitch tilts toward the viewer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Orbit {
    pub yaw: f32,
    pub pitch: f32,
}

impl Default for Orbit {
    fn default() -> Self {
        Self {
            yaw: 0.6,
            pitch: 0.5,
        }
    }
}

/// Orthographic projection of an (east, north, height) point to screen pixels.
pub fn project(point: [f32; 3], orbit: Orbit, origin: Pos2) -> Pos2 {
    let [east, north, height] = point;
    let (sin_yaw, cos_yaw) = orbit.yaw.sin_cos();
    let x = east * cos_yaw - north * sin_yaw;
    let y = east * sin_yaw + north * cos_yaw;

    let (sin_pitch, cos_pitch) = orbit.pitch.sin_cos();
    let screen_up = y * cos_pitch + height * sin_pitch;
    Pos2::new(origin.x + x * SCALE_PX_PER_M, origin.y - screen_up * SCALE_PX_PER_M)
}

/// Height above the ground plane, from signal strength: stronger signals sit higher.
pub fn signal_height_m(rssi_dbm: i16) -> f32 {
    ((f32::from(rssi_dbm) + 100.0) / 70.0).clamp(0.0, 1.0) * MAX_HEIGHT_M
}

pub fn show(
    ui: &mut egui::Ui,
    orbit: &mut Orbit,
    frame: Option<&ScreenFrame>,
    selected: Option<&str>,
) -> Option<String> {
    let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
    let delta = response.drag_delta();
    orbit.yaw += delta.x * 0.01;
    orbit.pitch = (orbit.pitch - delta.y * 0.01).clamp(PITCH_LIMITS.0, PITCH_LIMITS.1);

    let origin = response.rect.center();
    let grid = Stroke::new(1.0, Color32::DARK_GRAY);
    let steps = (GRID_HALF_SIZE_M / GRID_STEP_M) as i32;
    for i in -steps..=steps {
        let offset = i as f32 * GRID_STEP_M;
        let edge = GRID_HALF_SIZE_M;
        painter.line_segment(
            [
                project([offset, -edge, 0.0], *orbit, origin),
                project([offset, edge, 0.0], *orbit, origin),
            ],
            grid,
        );
        painter.line_segment(
            [
                project([-edge, offset, 0.0], *orbit, origin),
                project([edge, offset, 0.0], *orbit, origin),
            ],
            grid,
        );
    }

    let frame = frame?;
    let mut points = Vec::new();
    for device in &frame.devices {
        match (device.position_m, device.distance_m) {
            (Some((east, north)), _) => {
                let height = signal_height_m(device.last_rssi_dbm);
                let point = project([east as f32, north as f32, height], *orbit, origin);
                let is_selected = selected == Some(device.device_id.as_str());
                let (radius, fill) = if is_selected {
                    (7.0, Color32::WHITE)
                } else {
                    (5.0, Color32::LIGHT_BLUE)
                };
                painter.circle_filled(point, radius, fill);
                points.push((device.device_id.clone(), point));
                painter.text(
                    point + egui::vec2(8.0, 0.0),
                    egui::Align2::LEFT_CENTER,
                    &device.device_id,
                    egui::FontId::proportional(12.0),
                    Color32::WHITE,
                );
            }
            // Distance without bearing: a ground ring with no angle, so it implies no direction.
            (None, Some(distance)) => {
                draw_ground_ring(&painter, *orbit, origin, distance as f32);
            }
            (None, None) => {}
        }
    }
    nearest_click(&response, &points, 12.0)
}

fn draw_ground_ring(painter: &egui::Painter, orbit: Orbit, origin: Pos2, radius_m: f32) {
    const SEGMENTS: usize = 48;
    let stroke = Stroke::new(1.5, Color32::LIGHT_GREEN);
    let point = |k: usize| {
        let angle = k as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
        project(
            [radius_m * angle.cos(), radius_m * angle.sin(), 0.0],
            orbit,
            origin,
        )
    };
    for k in 0..SEGMENTS {
        painter.line_segment([point(k), point(k + 1)], stroke);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat(yaw: f32) -> Orbit {
        Orbit { yaw, pitch: 0.0 }
    }

    #[test]
    fn east_and_north_are_right_and_up_when_unrotated() {
        let origin = Pos2::new(100.0, 100.0);
        let east = project([1.0, 0.0, 0.0], flat(0.0), origin);
        let north = project([0.0, 1.0, 0.0], flat(0.0), origin);
        assert!((east.x - (100.0 + SCALE_PX_PER_M)).abs() < 1e-4 && (east.y - 100.0).abs() < 1e-4);
        assert!((north.x - 100.0).abs() < 1e-4 && (north.y - (100.0 - SCALE_PX_PER_M)).abs() < 1e-4);
    }

    #[test]
    fn quarter_turn_moves_east_onto_north_axis() {
        let origin = Pos2::new(0.0, 0.0);
        let p = project([1.0, 0.0, 0.0], flat(std::f32::consts::FRAC_PI_2), origin);
        assert!(p.x.abs() < 1e-4);
        assert!((p.y + SCALE_PX_PER_M).abs() < 1e-4);
    }

    #[test]
    fn height_moves_point_vertically_only_when_tilted() {
        let origin = Pos2::new(0.0, 0.0);
        let level = project([0.0, 0.0, 1.0], flat(0.0), origin);
        assert!(level.x.abs() < 1e-4 && level.y.abs() < 1e-4);

        let tilted = Orbit { yaw: 0.0, pitch: std::f32::consts::FRAC_PI_2 };
        let up = project([0.0, 0.0, 1.0], tilted, origin);
        assert!(up.x.abs() < 1e-4);
        assert!((up.y + SCALE_PX_PER_M).abs() < 1e-4);
    }

    #[test]
    fn signal_height_spans_zero_to_max() {
        assert_eq!(signal_height_m(-100), 0.0);
        assert!((signal_height_m(-30) - MAX_HEIGHT_M).abs() < 1e-4);
        assert_eq!(signal_height_m(-200), 0.0);
    }
}
