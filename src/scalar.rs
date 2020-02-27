use std::cmp::max;

fn lerp(a: f64, b: f64, s: f64) -> f64 {
    return (1. - s) * a + s * b;
}

pub struct FloatRange {
    start_inclusive: f64,
    end_inclusive: f64,
    step_count: u32,
    current_index: u32,
}

impl FloatRange {
    pub fn from_step_count(start_inclusive: f64, end_inclusive: f64, step_count: u32) -> Self {
        Self {
            start_inclusive,
            end_inclusive,
            step_count,
            current_index: 0,
        }
    }

    pub fn from_step_size(start_inclusive: f64, end_inclusive: f64, step_size: f64) -> Self {
        let range_size = (end_inclusive - start_inclusive).abs();
        if range_size == 0. {
            return Self::from_step_count(start_inclusive, end_inclusive, 1);
        }
        let count = max(2, (range_size / step_size) as u32);
        Self::from_step_count(start_inclusive, end_inclusive, count)
    }
}

impl Iterator for FloatRange {
    type Item = f64;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current_index >= self.step_count {
            return None
        }
        let s = if self.step_count <= 1 {
            0.5
        } else {
            (self.current_index as f64) / (self.step_count as f64 - 1.)
        };
        self.current_index += 1;
        Some(lerp(self.start_inclusive, self.end_inclusive, s))
    }
}
