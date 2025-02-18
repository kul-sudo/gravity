use crate::{Body, BodyID};
use macroquad::prelude::*;
use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

pub struct Direct;

impl Direct {
    pub const COLOR: Color = GREEN;

    pub fn handle(bodies: &mut HashMap<BodyID, Body>) -> Duration {
        let start = Instant::now();

        let bodies_clone = bodies.clone();
        let bodies_keys = bodies.keys().cloned().collect::<Vec<_>>();

        for lhs_id in &bodies_keys {
            for rhs_id in &bodies_keys {
                if lhs_id != rhs_id {
                    let lhs = bodies.get_mut(lhs_id).unwrap();
                    let rhs = bodies_clone.get(rhs_id).unwrap();

                    lhs.adjust_speed(rhs.pos, rhs.mass);
                }
            }
        }

        start.elapsed()
    }
}
