use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::{HashMap, HashSet},
    time::Instant,
};

use crate::body::{Body, BodyID, BODIES_N};
use std::sync::{LazyLock, RwLock};

pub type NodeID = Instant;

const BORDER_THICKNESS: f32 = 1.0;
static THETA: LazyLock<RwLock<f32>> = LazyLock::new(|| RwLock::new(1.0));

#[derive(Clone)]
pub struct Square {
    pub top_left: Complex<f32>,
    pub size: f32,
}

impl Square {
    pub fn contains(&self, pos: Complex<f32>) -> bool {
        (Rectangle {
            top_left: self.top_left,
            bottom_right: self.top_left + Complex::new(self.size, self.size),
        })
        .contains(pos)
    }
}

#[derive(Clone)]
pub struct Rectangle {
    pub top_left: Complex<f32>,
    pub bottom_right: Complex<f32>,
}

impl Rectangle {
    pub fn contains(&self, pos: Complex<f32>) -> bool {
        (self.top_left.re()..self.bottom_right.re()).contains(&pos.re())
            && (self.top_left.im()..self.bottom_right.im()).contains(&pos.im())
    }
}

#[derive(Clone)]
pub struct QuadtreeNode {
    pub children: Option<[[NodeID; 2]; 2]>,
    pub data: HashSet<BodyID>,
    pub square: Square,
    pub total_mass: f32,
    pub pos: Complex<f32>,
}

impl QuadtreeNode {
    pub fn draw(id: NodeID, quadtree_nodes: &mut HashMap<NodeID, Self>) {
        let current_node = quadtree_nodes.get(&id).unwrap();

        if let Some(children) = current_node.children {
            draw_line(
                current_node.square.top_left.re(),
                current_node.square.top_left.im() + current_node.square.size / 2.0,
                current_node.square.top_left.re() + current_node.square.size,
                current_node.square.top_left.im() + current_node.square.size / 2.0,
                BORDER_THICKNESS,
                GREEN,
            );

            draw_line(
                current_node.square.top_left.re() + current_node.square.size / 2.0,
                current_node.square.top_left.im(),
                current_node.square.top_left.re() + current_node.square.size / 2.0,
                current_node.square.top_left.im() + current_node.square.size,
                BORDER_THICKNESS,
                GREEN,
            );

            for child in children.iter().flatten() {
                //dbg!(quadtree_nodes.get(child).unwrap().children);
                Self::draw(*child, quadtree_nodes);
            }
        }
    }

    pub fn adjust_speed(
        id: NodeID,
        body_id: BodyID,
        bodies: &mut HashMap<BodyID, Body>,
        quadtree_nodes: &mut HashMap<NodeID, Self>,
    ) {
        let current_node = quadtree_nodes.get(&id).unwrap();
        let body = bodies.get_mut(&body_id).unwrap();

        match current_node.data.len() {
            0 => return,
            1 => {
                if !current_node.data.contains(&body_id) {
                    body.adjust_speed(current_node.pos, current_node.total_mass);
                }
            }
            _ => {
                let r = (current_node.pos - body.pos).abs();
                if current_node.square.size / r <= *THETA.read().unwrap()
                    && !current_node.data.contains(&body_id)
                {
                    body.adjust_speed(current_node.pos, current_node.total_mass);
                } else {
                    for child in current_node.children.unwrap().iter().flatten() {
                        Self::adjust_speed(*child, body_id, bodies, quadtree_nodes);
                    }
                }
            }
        }
    }

    pub fn split(
        id: NodeID,
        bodies: &HashMap<BodyID, Body>,
        quadtree_nodes: &mut HashMap<NodeID, Self>,
    ) {
        let current_node = quadtree_nodes.get(&id).unwrap();

        if current_node.data.len() <= 1 {
            return;
        }

        let mut new_quadtree_nodes = HashMap::new();

        let child_size = current_node.square.size / 2.0;

        let children: [[NodeID; 2]; 2] = core::array::from_fn(|i| {
            core::array::from_fn(|j| {
                let top_left = current_node.square.top_left
                    + Complex::new(j as f32 * child_size, i as f32 * child_size);

                let child_id = NodeID::now();
                let mut child = Self {
                    children: None,
                    data: HashSet::new(),
                    square: Square {
                        top_left,
                        size: current_node.square.size / 2.0,
                    },
                    total_mass: 0.0,
                    pos: Complex::new(0.0, 0.0),
                };

                for body_id in &current_node.data {
                    let body = bodies.get(body_id).unwrap();
                    if child.square.contains(body.pos) {
                        child.data.insert(*body_id);
                        child.total_mass += body.mass;
                    }
                }

                if child.total_mass != 0.0 {
                    child.pos = child
                        .data
                        .iter()
                        .map(|body_id| {
                            let value = bodies.get(body_id).unwrap();

                            value.mass * value.pos
                        })
                        .sum::<Complex<f32>>()
                        / child.total_mass;
                }

                new_quadtree_nodes.insert(child_id, child.clone());

                child_id
            })
        });

        quadtree_nodes.extend(new_quadtree_nodes);

        for child_id in children.iter().flatten() {
            Self::split(*child_id, bodies, quadtree_nodes);
        }

        let current_node_mut = quadtree_nodes.get_mut(&id).unwrap();
        current_node_mut.children = Some(children);
    }
}
