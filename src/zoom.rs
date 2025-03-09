use crate::barnes_hut::Rectangle;
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::ops::Range;

pub const ZOOM_STEP: f32 = 1.2;
pub const ZOOM_RANGE: Range<f32> = 0.3..5.0;

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Zoom {
    pub zoom: f32,
}

impl Zoom {
    pub fn get_rect(&self) -> Rectangle {
        let size = Complex::new(
            screen_width() as f64 / self.zoom as f64,
            screen_height() as f64 / self.zoom as f64,
        );

        Rectangle {
            top_left: Complex::new(
                screen_width() as f64 / 2.0 - size.re() / 2.0,
                screen_height() as f64 / 2.0 - size.im() / 2.0,
            ),
            bottom_right: Complex::new(
                screen_width() as f64 / 2.0 + size.re() / 2.0,
                screen_height() as f64 / 2.0 + size.im() / 2.0,
            ),
        }
    }
}
