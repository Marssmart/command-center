//! Playback: a bounded history of frames with pause, step, and inspect.

use crate::model::ScreenFrame;
use std::collections::VecDeque;

/// Records frames as they arrive. While live it shows the newest frame. Pausing freezes the
/// view on one frame, and stepping moves through the retained history.
pub struct Playback {
    capacity: usize,
    history: VecDeque<ScreenFrame>,
    paused: bool,
    cursor: usize,
}

impl Playback {
    /// `capacity` is the maximum number of frames retained; it must be at least 1.
    pub fn new(capacity: usize) -> Self {
        assert!(capacity >= 1, "playback needs room for at least one frame");
        Self {
            capacity,
            history: VecDeque::with_capacity(capacity),
            paused: false,
            cursor: 0,
        }
    }

    /// Records a new frame. Pushing while paused keeps the view on the same frame, unless that
    /// frame was evicted, in which case the view moves to the oldest retained frame.
    pub fn push(&mut self, frame: ScreenFrame) {
        if self.history.len() == self.capacity {
            self.history.pop_front();
            self.cursor = self.cursor.saturating_sub(1);
        }
        self.history.push_back(frame);
        if !self.paused {
            self.cursor = self.history.len() - 1;
        }
    }

    pub fn pause(&mut self) {
        self.paused = true;
    }

    /// Resumes live view at the newest frame.
    pub fn resume(&mut self) {
        self.paused = false;
        self.cursor = self.history.len().saturating_sub(1);
    }

    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Pauses and moves one frame back, stopping at the oldest retained frame.
    pub fn step_back(&mut self) {
        self.paused = true;
        self.cursor = self.cursor.saturating_sub(1);
    }

    /// Pauses and moves one frame forward, stopping at the newest retained frame.
    pub fn step_forward(&mut self) {
        self.paused = true;
        if self.cursor + 1 < self.history.len() {
            self.cursor += 1;
        }
    }

    /// The frame currently shown, or `None` if nothing has been recorded.
    pub fn current(&self) -> Option<&ScreenFrame> {
        self.history.get(self.cursor)
    }

    /// Reads a retained frame by index (0 is the oldest) without moving the view.
    pub fn inspect(&self, index: usize) -> Option<&ScreenFrame> {
        self.history.get(index)
    }

    pub fn len(&self) -> usize {
        self.history.len()
    }

    pub fn is_empty(&self) -> bool {
        self.history.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn frame(secs: u64) -> ScreenFrame {
        ScreenFrame {
            at: Duration::from_secs(secs),
            devices: Vec::new(),
        }
    }

    fn shown(playback: &Playback) -> Option<u64> {
        playback.current().map(|f| f.at.as_secs())
    }

    #[test]
    fn live_view_follows_newest_frame() {
        let mut playback = Playback::new(10);
        assert!(playback.is_empty());
        playback.push(frame(1));
        playback.push(frame(2));
        assert_eq!(shown(&playback), Some(2));
    }

    #[test]
    fn pause_freezes_view_while_history_grows() {
        let mut playback = Playback::new(10);
        playback.push(frame(1));
        playback.push(frame(2));
        playback.pause();
        playback.push(frame(3));
        assert_eq!(shown(&playback), Some(2));
        assert_eq!(playback.len(), 3);

        playback.resume();
        assert_eq!(shown(&playback), Some(3));
    }

    #[test]
    fn stepping_moves_within_bounds() {
        let mut playback = Playback::new(10);
        for secs in 1..=3 {
            playback.push(frame(secs));
        }
        playback.step_back();
        assert!(playback.is_paused());
        assert_eq!(shown(&playback), Some(2));
        playback.step_back();
        playback.step_back();
        assert_eq!(shown(&playback), Some(1));

        playback.step_forward();
        playback.step_forward();
        playback.step_forward();
        assert_eq!(shown(&playback), Some(3));
    }

    #[test]
    fn history_is_bounded_and_view_follows_eviction() {
        let mut playback = Playback::new(2);
        playback.push(frame(1));
        playback.push(frame(2));
        playback.step_back(); // showing frame 1
        playback.push(frame(3)); // frame 1 is evicted

        assert_eq!(playback.len(), 2);
        assert_eq!(playback.inspect(0).map(|f| f.at.as_secs()), Some(2));
        assert_eq!(shown(&playback), Some(2));
    }

    #[test]
    fn empty_playback_shows_nothing() {
        let playback = Playback::new(3);
        assert_eq!(shown(&playback), None);
        assert_eq!(playback.inspect(0).map(|f| f.at), None);
    }
}
