use std::cmp::{max, Ordering};
use std::process::exit;

use crate::scalar::RangeValue::{AfterEnd, BeforeStart, Inside};

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
            return None;
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


pub struct DenseNumberLine<T> {
    buckets: Vec<T>,
    start: f64,
    end: f64,
}

impl<T: Clone> DenseNumberLine<T> {
    pub fn new<F>(start: f64, end: f64, resolution: f64, initial: F) -> Self
        where F: Fn(f64) -> T {
        let buckets = if start < end {
            FloatRange::from_step_size(start, end, resolution)
                .map(initial)
                .collect()
        } else {
            vec![]
        };
        Self {
            start,
            end,
            buckets,
        }
    }

    pub fn merge<F>(&mut self, interval: &Interval, value: T, merge_func: F)
        where F: Fn(&T, &T) -> T {
        if interval.is_empty() {
            return;
        }
        let a = match self.bucket_index(interval.start) {
            AfterEnd => return,
            BeforeStart => 0,
            Inside(i) => i,
        };

        let b = match self.bucket_index(interval.end) {
            AfterEnd => self.buckets.len() - 1,
            BeforeStart => return,
            Inside(i) => i,
        };

        for i in a..(b + 1) {
            self.buckets[i] = (merge_func)(&self.buckets[i], &value);
        }
    }

    pub fn set(&mut self, interval: &Interval, value: T) {
        self.merge(interval, value, |old_v, new_v| new_v.clone());
    }

    pub fn get(&self, pos: f64) -> Option<&T> {
        if let Inside(i) = self.bucket_index(pos) {
            Some(&self.buckets[i])
        } else {
            None
        }
    }

    pub fn intervals_matching<F>(&self, match_func: F) -> IntervalMatchIter<T, F>
        where F: Fn(&T) -> bool {
        IntervalMatchIter::new(&self, match_func)
    }

    pub fn values(&self) -> &Vec<T> {
        &self.buckets
    }

    fn bucket_index(&self, pos: f64) -> RangeValue<usize> {
        if self.is_empty() {
            return AfterEnd;
        }
        if pos >= self.end {
            return AfterEnd;
        }
        if pos < self.start {
            return BeforeStart;
        }
        Inside((self.buckets.len() as f64 * (pos - self.start) / (self.end - self.start)) as usize)
    }

    fn is_empty(&self) -> bool {
        self.buckets.is_empty()
    }
}

pub struct Interval {
    pub start: f64,
    pub end: f64,
}

impl Interval {
    pub fn new(start: f64, end: f64) -> Self {
        Self {
            start,
            end,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.start >= self.end
    }

    pub fn center(&self) -> f64 {
        (self.start + self.end) / 2.
    }

    pub fn length(&self) -> f64 {
        return self.end - self.start;
    }
}

enum RangeValue<T> {
    BeforeStart,
    Inside(T),
    AfterEnd,
}

pub struct IntervalMatchIter<'a, T, F> where F: Fn(&T) -> bool {
    match_func: F,
    index: usize,
    number_line: &'a DenseNumberLine<T>,
}

impl<'a, T, F> IntervalMatchIter<'a, T, F> where F: Fn(&T) -> bool {
    fn new(number_line: &'a DenseNumberLine<T>, match_func: F) -> Self {
        Self {
            number_line,
            match_func,
            index: 0,
        }
    }

    fn index_to_pos(&self, index: usize) -> f64 {
        let a = self.number_line.start;
        let b = self.number_line.end;
        a + (b - a) * (index as f64) / (self.number_line.buckets.len() as f64)
    }
}

impl<'a, T, F> Iterator for IntervalMatchIter<'a, T, F> where F: Fn(&T) -> bool {
    type Item = Interval;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.number_line.buckets.len() {
            return None;
        }

        let mut interval: Option<Interval> = None;
        for i in self.index..self.number_line.buckets.len() {
            self.index = i + 1;
            let pos = self.index_to_pos(i);
            if (self.match_func)(&self.number_line.buckets[i]) {
                if interval.is_none() {
                    interval = Some(Interval::new(pos, pos));
                } else {
                    interval.as_mut().unwrap().end = self.index_to_pos(i + 1);
                }
            } else if interval.is_some() {
                interval.as_mut().unwrap().end = pos;
                break;
            }
        }
        interval
    }
}
