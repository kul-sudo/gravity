use crate::{DT, G, INITIAL_MASS, barnes_hut::Rectangle};
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{collections::HashMap, num::NonZero, time::Instant};

pub const BODIES_N: NonZero<usize> = NonZero::new(
    //500 // Recommended for watching the deterministic chaos
    1500,
)
.unwrap();

pub type BodyID = Instant;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Body {
    pub pos: Complex<f64>,
    pub speed: Complex<f64>,
    pub mass: f64,
    pub radius: f64,
}

pub fn get_rectangle(bodies: &mut HashMap<BodyID, Body>) -> Rectangle {
    let mut topmost = f64::INFINITY;
    let mut bottommost = f64::NEG_INFINITY;

    let mut leftmost = f64::INFINITY;
    let mut rightmost = f64::NEG_INFINITY;

    for body in bodies.values() {
        topmost = topmost.min(body.pos.im());
        bottommost = bottommost.max(body.pos.im() + body.radius);

        leftmost = leftmost.min(body.pos.re());
        rightmost = rightmost.max(body.pos.re() + body.radius);
    }

    Rectangle {
        top_left: Complex::new(leftmost, topmost),
        bottom_right: Complex::new(rightmost, bottommost),
    }
}

impl Body {
    pub fn get_radius(mass: f64) -> f64 {
        mass.powf(1.0 / 3.0)
    }

    pub fn adjust_momentum(bodies: &mut HashMap<BodyID, Body>) {
        let total_momentum = bodies
            .values()
            .map(|body| body.mass * body.speed)
            .sum::<Complex<f64>>();
        let delta = -total_momentum / (BODIES_N.get() as f64 * INITIAL_MASS);
        for body in bodies.values_mut() {
            body.speed += delta;
        }
    }

    pub fn connect(pair: [BodyID; 2], bodies: &mut HashMap<BodyID, Body>) {
        let mass = pair
            .iter()
            .map(|body_id| bodies.get(body_id).unwrap().mass)
            .sum::<f64>();
        let pos = pair
            .iter()
            .map(|body_id| {
                let body = bodies.get(body_id).unwrap();
                body.mass * body.pos
            })
            .sum::<Complex<f64>>()
            / mass;
        let speed = pair
            .iter()
            .map(|body_id| {
                let body = bodies.get(body_id).unwrap();
                body.mass * body.speed
            })
            .sum::<Complex<f64>>()
            / mass;

        bodies.remove(&pair[0]);
        bodies.remove(&pair[1]);

        bodies.insert(
            BodyID::now(),
            Body {
                pos,
                speed,
                mass,
                radius: Self::get_radius(mass),
            },
        );
    }

    pub fn connect_all(bodies: &mut HashMap<BodyID, Body>) {
        loop {
            let mut deepest_connection_depth = f64::NEG_INFINITY;
            let mut deepest_connection_pair: Option<[BodyID; 2]> = None;

            for (lhs_body_id, lhs_body) in bodies.iter() {
                for (rhs_body_id, rhs_body) in bodies.iter() {
                    if lhs_body_id == rhs_body_id {
                        continue;
                    }

                    let depth =
                        lhs_body.radius + rhs_body.radius - (lhs_body.pos - rhs_body.pos).abs();

                    if depth >= 0.0 && depth > deepest_connection_depth {
                        deepest_connection_depth = depth;
                        deepest_connection_pair = Some([*lhs_body_id, *rhs_body_id]);
                    }
                }
            }

            match deepest_connection_pair {
                Some(pair) => {
                    Self::connect(pair, bodies);
                }
                None => break,
            }
        }
    }

    pub fn get_earliest_collision(
        time_lower_bound: f64,
        bodies: &mut HashMap<BodyID, Body>,
    ) -> Option<(f64, [BodyID; 2])> {
        let mut earliest_collision_time = f64::INFINITY;
        let mut earliest_collision_pair: Option<[BodyID; 2]> = None;

        for (lhs_body_id, lhs_body) in bodies.iter() {
            for (rhs_body_id, rhs_body) in bodies.iter() {
                if lhs_body_id == rhs_body_id {
                    continue;
                }

                let dspeed = lhs_body.speed - rhs_body.speed;
                let a = dspeed.abs().powi(2);

                if a != 0.0 {
                    let dpos = lhs_body.pos - rhs_body.pos;

                    let r = lhs_body.radius + rhs_body.radius;

                    let b = (dpos.re() * dspeed.re() + dpos.im() * dspeed.im()) * 2.0;
                    let c = dpos.abs().powi(2) - r.powi(2);
                    let d = b.powi(2) - 4.0 * a * c;

                    let d_sqrt = d.sqrt();
                    if !d_sqrt.is_nan() // sqrt(negative n) = NaN
                    && d_sqrt >= b
                    {
                        let t_min = -(b + d_sqrt) / (2.0 * a);

                        if t_min <= time_lower_bound && t_min < earliest_collision_time {
                            earliest_collision_time = t_min;
                            earliest_collision_pair = Some([*lhs_body_id, *rhs_body_id]);
                        }
                    }
                }
            }
        }

        earliest_collision_pair.map(|pair| (earliest_collision_time, pair))
    }

    pub fn update_bodies(lambda: f64, bodies: &mut HashMap<BodyID, Body>) {
        let collision = Self::get_earliest_collision(lambda, bodies);
        match collision {
            Some((time, pair)) => {
                for body in bodies.values_mut() {
                    body.pos += body.speed * time;
                }

                Self::connect(pair, bodies);
                Self::connect_all(bodies);

                if time < lambda {
                    Self::update_bodies(lambda - time, bodies)
                }
            }
            None => {
                for body in bodies.values_mut() {
                    body.pos += body.speed * lambda;
                }
            }
        }
    }

    pub fn adjust_speed(&mut self, pos: Complex<f64>, mass: f64) {
        let r = pos - self.pos;
        self.speed += DT * G * mass * r / r.abs().powi(3);
    }
}
