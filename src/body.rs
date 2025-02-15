use crate::{G, NodeID, QuadtreeNode, Square};
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::{HashMap, HashSet},
    time::{Duration, Instant},
};

pub const BODIES_N: usize = 3600;

pub type BodyID = Instant;

#[derive(Clone, Copy)]
pub struct Body {
    pub pos: Complex<f32>,
    pub speed: Complex<f32>,
    pub mass: f32,
}

impl Body {
    pub fn get_radius(&self) -> f32 {
        self.mass.powf(1.0 / 3.0)
    }

    fn get_closest(&self, id: BodyID, bodies: &HashMap<BodyID, Body>) -> Vec<BodyID> {
        let mut closest = vec![];
        let radius = self.get_radius();

        for (body_id, body) in bodies {
            if *body_id != id && (body.pos - self.pos).abs() <= radius + body.get_radius() {
                closest.push(*body_id)
            }
        }

        closest
    }

    pub fn update_bodies(bodies: &mut HashMap<BodyID, Body>) {
        for body in bodies.values_mut() {
            body.pos += body.speed
        }

        let mut visited = HashSet::new();
        let mut add = vec![];

        for (body_id, body) in bodies.iter() {
            if visited.contains(body_id) {
                continue;
            }

            let chain = body.collect_chain(*body_id, bodies, &mut visited);

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
        &self,
        id: BodyID,
        bodies: &HashMap<BodyID, Body>,
        visited: &mut HashSet<BodyID>,
    ) -> Vec<BodyID> {
        let mut chain = vec![id];
        visited.insert(id);

        let closest = self.get_closest(id, bodies);

        for neighbor_id in closest {
            if visited.contains(&neighbor_id) {
                continue;
            }

            let neighbor = bodies.get(&neighbor_id).unwrap();
            let subchain = neighbor.collect_chain(neighbor_id, bodies, visited);

            for chain_member_id in subchain {
                chain.push(chain_member_id);
            }
        }

        chain
    }

    pub fn handle_direct_method(bodies: &mut HashMap<BodyID, Self>) -> Duration {
        let start = Instant::now();

        let bodies_keys = bodies.keys().cloned().collect::<Vec<_>>();
        for lhs_id in &bodies_keys {
            for rhs_id in &bodies_keys {
                if lhs_id != rhs_id {
                    let [Some(lhs), Some(rhs)] = bodies.get_disjoint_mut([&lhs_id, &rhs_id]) else {
                        panic!()
                    };

                    lhs.adjust_speed(rhs.pos, rhs.mass);
                }
            }
        }

        start.elapsed()
    }

    pub fn adjust_speed(&mut self, pos: Complex<f32>, mass: f32) {
        let r = pos - self.pos;
        self.speed += G * mass * r / r.abs().powi(3);
    }

    pub fn handle_barnes_hut(
        bodies: &mut HashMap<BodyID, Self>,
        quadtree_nodes: &mut HashMap<NodeID, QuadtreeNode>,
    ) -> Duration {
        let start = Instant::now();

        quadtree_nodes.clear();

        let square = Square {
            top_left: Complex::new(0.0, -(screen_width() - screen_height()) / 2.0),
            size: screen_width(),
        };
        let mut bodies_in_root = HashSet::new();
        let mut bodies_not_in_root = HashSet::new();
        for (body_id, body) in bodies.iter() {
            if square.contains(body.pos) {
                bodies_in_root.insert(body_id.clone());
            } else {
                bodies_not_in_root.insert(body_id.clone());
            }
        }

        let root_id = NodeID::now();
        let root = QuadtreeNode {
            children: None,
            data: bodies_in_root.clone(),
            square,
            total_mass: 0.0,
            pos: Complex::new(0.0, 0.0),
        };
        quadtree_nodes.insert(root_id, root);
        QuadtreeNode::split(root_id, bodies, quadtree_nodes);

        for body_id in bodies_in_root.iter() {
            QuadtreeNode::adjust_speed(root_id, *body_id, bodies, quadtree_nodes);
        }

        let end = start.elapsed();

        let bodies_keys = bodies.keys().cloned().collect::<Vec<_>>();
        for lhs_id in bodies_not_in_root.iter() {
            for rhs_id in &bodies_keys {
                if lhs_id != rhs_id {
                    let [Some(lhs), Some(rhs)] = bodies.get_disjoint_mut([&lhs_id, &rhs_id]) else {
                        panic!()
                    };

                    lhs.adjust_speed(rhs.pos, rhs.mass);
                    rhs.adjust_speed(lhs.pos, lhs.mass);
                }
            }
        }

        //QuadtreeNode::draw(root_id, quadtree_nodes);

        return end;
    }
}
