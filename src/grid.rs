use crate::{Body, BodyID, body::get_rectangle};
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::{HashMap, HashSet},
    time::Instant,
};

pub const TAU: f32 = 0.3;

#[derive(Clone)]
pub struct Cell {
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
}
