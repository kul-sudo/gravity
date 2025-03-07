use crate::barnes_hut::Rectangle;
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};

pub struct Zoom {
    pub zoom: f32,
}

impl Zoom {
    pub fn get_rect(&self) -> Rectangle {
        let size = Complex::new(screen_width() / self.zoom, screen_height() / self.zoom);

        Rectangle {
            top_left: Complex::new(
                screen_width() / 2.0 - size.re() / 2.0,
                screen_height() / 2.0 - size.im() / 2.0,
            ),
            bottom_right: Complex::new(
                screen_width() / 2.0 + size.re() / 2.0,
                screen_height() / 2.0 + size.im() / 2.0,
            ),
        }
    }
}
