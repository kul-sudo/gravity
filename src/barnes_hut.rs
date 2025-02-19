use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::{HashMap, HashSet},
    sync::{LazyLock, RwLock},
    time::{Duration, Instant},
};
use std::array::from_fn;

use crate::{
    BORDER_COLOR, BORDER_THICKNESS,
    body::{Body, BodyID, get_rectangle},
};

pub type NodeID = Instant;

static THETA: LazyLock<RwLock<f32>> = LazyLock::new(|| RwLock::new(7.0));

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
    pub bodies: HashSet<BodyID>,
    pub square: Square,
    pub total_mass: f32,
    pub pos: Complex<f32>,
}

impl QuadtreeNode {
    pub fn draw(id: NodeID, quadtree_nodes: &mut HashMap<NodeID, Self>, zoom: f32) {
        let current_node = quadtree_nodes.get(&id).unwrap();

        let border = BORDER_THICKNESS / zoom;

        if let Some(children) = current_node.children {
            draw_line(
                current_node.square.top_left.re(),
                current_node.square.top_left.im() + current_node.square.size / 2.0,
                current_node.square.top_left.re() + current_node.square.size,
                current_node.square.top_left.im() + current_node.square.size / 2.0,
                border,
                BORDER_COLOR,
            );

            draw_line(
                current_node.square.top_left.re() + current_node.square.size / 2.0,
                current_node.square.top_left.im(),
                current_node.square.top_left.re() + current_node.square.size / 2.0,
                current_node.square.top_left.im() + current_node.square.size,
                border,
                BORDER_COLOR,
            );

            for child in children.iter().flatten() {
                Self::draw(*child, quadtree_nodes, zoom);
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

        match current_node.bodies.len() {
            0 => (),
            1 => {
                if !current_node.bodies.contains(&body_id) {
                    body.adjust_speed(current_node.pos, current_node.total_mass);
                }
            }
            _ => {
                let r = (current_node.pos - body.pos).abs();
                if current_node.square.size / r <= *THETA.read().unwrap()
                    && !current_node.bodies.contains(&body_id)
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

        if current_node.bodies.len() <= 1 {
            return;
        }

        let mut new_quadtree_nodes = HashMap::new();
        let mut added_bodies = HashSet::new();

        let child_size = current_node.square.size / 2.0;

        let children: [[NodeID; 2]; 2] = from_fn(|i| {
            from_fn(|j| {
                let top_left = current_node.square.top_left
                    + Complex::new(j as f32 * child_size, i as f32 * child_size);

                let child_id = NodeID::now();
                let mut child = Self {
                    children: None,
                    bodies: HashSet::new(),
                    square: Square {
                        top_left,
                        size: current_node.square.size / 2.0,
                    },
                    total_mass: 0.0,
                    pos: Complex::ZERO,
                };

                for body_id in &current_node.bodies {
                    if added_bodies.contains(body_id) {
                        continue;
                    }

                    let body = bodies.get(body_id).unwrap();
                    if child.square.contains(body.pos) {
                        child.bodies.insert(*body_id);
                        child.total_mass += body.mass;

                        added_bodies.insert(body_id);
                    }
                }

                if child.total_mass != 0.0 {
                    child.pos = child
                        .bodies
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

pub struct BarnesHut;

impl BarnesHut {
    pub const DRAW: bool = true;
    pub const COLOR: Color = RED;

    pub fn handle(bodies: &mut HashMap<BodyID, Body>, zoom: f32) -> Duration {
        let start = Instant::now();

        let mut quadtree_nodes: HashMap<NodeID, QuadtreeNode> = HashMap::new();

        let rectangle = get_rectangle(bodies);

        let width = rectangle.bottom_right.re() - rectangle.top_left.re();
        let height = rectangle.bottom_right.im() - rectangle.top_left.im();

        let top_left;
        let size;

        if width >= height {
            top_left = Complex::new(
                rectangle.top_left.re(),
                rectangle.top_left.im() - (width - height) / 2.0,
            );
            size = width;
        } else {
            top_left = Complex::new(
                rectangle.top_left.re() - (height - width) / 2.0,
                rectangle.top_left.im(),
            );
            size = height;
        }

        let square = Square { top_left, size };

        let bodies_keys = bodies.keys().cloned().collect::<HashSet<_>>();

        let root_id = NodeID::now();
        let root = QuadtreeNode {
            children: None,
            bodies: bodies_keys.clone(),
            square,
            total_mass: 0.0,
            pos: Complex::ZERO,
        };
        quadtree_nodes.insert(root_id, root.clone());
        QuadtreeNode::split(root_id, bodies, &mut quadtree_nodes);

        for body_id in bodies_keys {
            QuadtreeNode::adjust_speed(root_id, body_id, bodies, &mut quadtree_nodes);
        }

        let end = start.elapsed();

        if Self::DRAW {
            let border = BORDER_THICKNESS / zoom;

            draw_rectangle_lines(
                root.square.top_left.re(),
                root.square.top_left.im(),
                root.square.size,
                root.square.size,
                border,
                BORDER_COLOR,
            );

            QuadtreeNode::draw(root_id, &mut quadtree_nodes, zoom);
        }

        end
    }
}
