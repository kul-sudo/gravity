use crate::{Body, BodyID, body::get_rectangle};
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::{HashMap, HashSet},
    time::Instant,
};

pub const TAU: f32 = 0.3;

#[derive(Clone)]
struct Cell {
    pub bodies: HashSet<BodyID>,
    pub total_mass: f32,
    pub pos: Complex<f32>,
}

impl Cell {
    pub fn add_body(&mut self, body_id: Instant, bodies: &HashMap<BodyID, Body>) {
        self.bodies.insert(body_id);

        let body = bodies.get(&body_id).unwrap();
        self.total_mass += body.mass;
        self.pos += body.mass * body.pos;
    }

    pub fn set_pos(&mut self) {
        if self.total_mass != 0.0 {
            self.pos /= self.total_mass;
        }
    }

    pub fn adjust_speed(bodies: &mut HashMap<BodyID, Body>) {
        let rectangle = get_rectangle(bodies);

        let width = rectangle.bottom_right.re() - rectangle.top_left.re();
        let height = rectangle.bottom_right.im() - rectangle.top_left.im();

        let target_size = ((TAU * width * height) / (bodies.len() as f32).sqrt()).sqrt();

        let rows_n = ((height / target_size).round() as usize).max(1);
        let cell_height = height / (rows_n as f32);

        let columns_n = ((width / cell_height).round() as usize).max(1);
        let cell_width = width / (columns_n as f32);

        let mut cells = vec![
            vec![
                Cell {
                    bodies: HashSet::with_capacity(bodies.len() / (rows_n * columns_n)),
                    total_mass: 0.0,
                    pos: Complex::ZERO,
                };
                columns_n
            ];
            rows_n
        ];
    }
}
