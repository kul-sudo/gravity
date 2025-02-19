use crate::{DT, G, barnes_hut::Rectangle};
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::{HashMap, HashSet},
    time::Instant,
};

pub const BODIES_N: usize = 2400;

pub type BodyID = Instant;

#[derive(Clone, Copy)]
pub struct Body {
    pub pos: Complex<f32>,
    pub speed: Complex<f32>,
    pub mass: f32,
    pub lived_in_one_step: f32,
}

pub fn get_rectangle(bodies: &mut HashMap<BodyID, Body>) -> Rectangle {
    let mut topmost = f32::INFINITY;
    let mut bottommost = f32::NEG_INFINITY;

    let mut leftmost = f32::INFINITY;
    let mut rightmost = f32::NEG_INFINITY;

    for body in bodies.values() {
        let radius = body.get_radius();

        topmost = topmost.min(body.pos.im());
        bottommost = bottommost.max(body.pos.im() + radius);

        leftmost = leftmost.min(body.pos.re());
        rightmost = rightmost.max(body.pos.re() + radius);
    }

    Rectangle {
        top_left: Complex::new(leftmost, topmost),
        bottom_right: Complex::new(rightmost, bottommost),
    }
}

impl Body {
    pub fn get_radius(&self) -> f32 {
        self.mass.powf(1.0 / 3.0)
    }

    pub fn update_bodies(bodies: &mut HashMap<BodyID, Body>) {
        let dt = *DT.read().unwrap();

        for body in bodies.values_mut() {
            body.lived_in_one_step = dt;
        }

        let mut all_collisions: HashMap<BodyID, Vec<BodyID>> = HashMap::new();

        let bodies_keys = bodies.keys().cloned().collect::<Vec<_>>();

        for lhs_body_id in &bodies_keys {
            let lhs_body_radius = bodies.get_mut(lhs_body_id).unwrap().get_radius();
            for rhs_body_id in &bodies_keys {
                if lhs_body_id == rhs_body_id {
                    continue;
                }

                let [Some(lhs_body), Some(rhs_body)] =
                    bodies.get_disjoint_mut([&lhs_body_id, &rhs_body_id])
                else {
                    panic!()
                };

                let dpos = lhs_body.pos - rhs_body.pos;
                let dspeed = lhs_body.speed - rhs_body.speed;

                let r = lhs_body_radius + rhs_body.get_radius();

                let a = dspeed.abs().powi(2);
                let b = (dpos.re() * dspeed.re() + dpos.im() * dspeed.im()) * 2.0;
                let c = dpos.abs().powi(2) - r.powi(2);
                let d = b.powi(2) - 4.0 * a * c;

                if a == 0.0 && c <= 0.0 {
                    all_collisions
                        .entry(*lhs_body_id)
                        .and_modify(|value| value.push(*rhs_body_id))
                        .or_insert(vec![*rhs_body_id]);
                    all_collisions
                        .entry(*rhs_body_id)
                        .and_modify(|value| value.push(*lhs_body_id))
                        .or_insert(vec![*lhs_body_id]);

                    lhs_body.lived_in_one_step = 0.0;
                    rhs_body.lived_in_one_step = 0.0;
                } else if a != 0.0 && d >= 0.0 {
                    let d_sqrt = d.sqrt();
                    let t_min = -(b + d_sqrt) / (a * 2.0);

                    if d_sqrt >= b && t_min <= dt {
                        all_collisions
                            .entry(*lhs_body_id)
                            .and_modify(|value| value.push(*rhs_body_id))
                            .or_insert(vec![*rhs_body_id]);
                        all_collisions
                            .entry(*rhs_body_id)
                            .and_modify(|value| value.push(*lhs_body_id))
                            .or_insert(vec![*lhs_body_id]);

                        let t = t_min * (t_min >= 0.0) as usize as f32;
                        lhs_body.lived_in_one_step = lhs_body.lived_in_one_step.min(t);
                        rhs_body.lived_in_one_step = rhs_body.lived_in_one_step.min(t);
                    }
                }
            }
        }

        for body in bodies.values_mut() {
            body.pos += body.lived_in_one_step * body.speed;
        }

        let mut visited = HashSet::new();
        let mut add = vec![];

        for (body_id, collisions) in all_collisions {
            if visited.contains(&body_id) {
                continue;
            }

            let chain = Self::collect_chain(body_id, &collisions, &mut visited);

            let new_mass = chain
                .iter()
                .map(|body_id| bodies.get(body_id).unwrap().mass)
                .sum::<f32>();
            let new_pos = chain
                .iter()
                .map(|body_id| {
                    let value = bodies.get(body_id).unwrap();
                    value.mass * value.pos
                })
                .sum::<Complex<f32>>()
                / new_mass;
            let new_speed = chain
                .iter()
                .map(|body_id| {
                    let value = bodies.get(body_id).unwrap();
                    value.mass * value.speed
                })
                .sum::<Complex<f32>>()
                / new_mass;

            add.push(Body {
                pos: new_pos,
                speed: new_speed,
                mass: new_mass,
                lived_in_one_step: 0.0,
            });
        }

        for body_id in &visited {
            bodies.remove(body_id);
        }

        for instance in add {
            bodies.insert(BodyID::now(), instance);
        }
    }

    fn collect_chain(
        id: BodyID,
        collisions: &Vec<BodyID>,
        visited: &mut HashSet<BodyID>,
    ) -> Vec<BodyID> {
        let mut chain = vec![id];
        visited.insert(id);

        for child_id in collisions {
            if visited.contains(&child_id) {
                continue;
            }

            chain.push(*child_id);

            for grandchild_id in Self::collect_chain(*child_id, collisions, visited) {
                if visited.contains(&grandchild_id) {
                    continue;
                }
                chain.push(grandchild_id);
            }
        }

        chain
    }

    pub fn adjust_speed(&mut self, pos: Complex<f32>, mass: f32) {
        let dt = *DT.read().unwrap();
        let r = pos - self.pos;
        self.speed += dt * G * mass * r / r.abs().powi(3);
    }
}
