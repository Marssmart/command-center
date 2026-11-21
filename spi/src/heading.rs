//! Source of the sensor's facing direction. Bearings are measured against it.

use std::sync::Mutex;

/// Reports the direction the sensor faces, in degrees clockwise from the reference direction.
///
/// The reference is north for a real compass, and "forward" for a manual dial. Bearings computed
/// from the source are in the same frame, so the UI must say which one is in use.
pub trait HeadingSource: Send + Sync {
    /// Current facing in degrees, or `None` when unknown. Unknown must not be guessed.
    fn heading_deg(&self) -> Option<f64>;
}

/// Facing set by the user, for Macs with no compass.
#[derive(Debug, Default)]
pub struct ManualFacing {
    facing: Mutex<Option<f64>>,
}

impl ManualFacing {
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the facing in degrees. Values are wrapped into [0, 360).
    pub fn set(&self, facing_deg: Option<f64>) {
        *self.facing.lock().expect("facing lock poisoned") = facing_deg.map(|d| d.rem_euclid(360.0));
    }
}

impl HeadingSource for ManualFacing {
    fn heading_deg(&self) -> Option<f64> {
        *self.facing.lock().expect("facing lock poisoned")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unset_facing_is_unknown() {
        assert_eq!(ManualFacing::new().heading_deg(), None);
    }

    #[test]
    fn set_facing_is_wrapped_into_one_turn() {
        let facing = ManualFacing::new();
        facing.set(Some(-90.0));
        assert_eq!(facing.heading_deg(), Some(270.0));
        facing.set(None);
        assert_eq!(facing.heading_deg(), None);
    }
}
