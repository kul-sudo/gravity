use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::array::from_fn;
use std::{
    collections::{HashMap, HashSet},
    sync::{LazyLock, RwLock},
    time::{Duration, Instant},
};

use crate::{
    BORDER_COLOR, BORDER_THICKNESS,
    body::{Body, BodyID, get_rectangle},
};

pub type NodeID = Instant;

pub static THETA: LazyLock<RwLock<f32>> = LazyLock::new(|| RwLock::new(0.0));
const DELTA_THETA: f32 = 0.01;

#[derive(Clone)]
pub struct Square {
    pub top_left: Complex<f32>,
    pub size: f32,
}

#[derive(Clone)]
pub struct Rectangle {
    pub top_left: Complex<f32>,
    pub bottom_right: Complex<f32>,
}

#[derive(Clone)]
pub enum QuadtreeNodeBodies {
    All,
    Bodies(HashSet<BodyID>),
}

#[derive(Clone)]
pub struct QuadtreeNode {
    pub children: Option<[[NodeID; 2]; 2]>,
    pub bodies: QuadtreeNodeBodies,
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
        let bodies_len = bodies.len();
        let body = bodies.get_mut(&body_id).unwrap();

        match match &current_node.bodies {
            QuadtreeNodeBodies::All => bodies_len,
            QuadtreeNodeBodies::Bodies(node_bodies) => node_bodies.len(),
        } {
            0 => (),
            1 => {
                if !match &current_node.bodies {
                    QuadtreeNodeBodies::All => true,
                    QuadtreeNodeBodies::Bodies(node_bodies) => node_bodies.contains(&body_id),
                } {
                    body.adjust_speed(current_node.pos, current_node.total_mass);
                }
            }
            _ => {
                let r = (current_node.pos - body.pos).abs();
                if current_node.square.size / r <= *THETA.read().unwrap()
                    && !match &current_node.bodies {
                        QuadtreeNodeBodies::All => true,
                        QuadtreeNodeBodies::Bodies(node_bodies) => node_bodies.contains(&body_id),
                    }
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

        if match &current_node.bodies {
            QuadtreeNodeBodies::All => bodies.len(),
            QuadtreeNodeBodies::Bodies(node_bodies) => node_bodies.len(),
        } <= 1
        {
            return;
        }

        let child_size = current_node.square.size / 2.0;

        let mut children: [[(NodeID, QuadtreeNode); 2]; 2] = from_fn(|i| {
            from_fn(|j| {
                (
                    NodeID::now(),
                    Self {
                        children: None,
                        bodies: QuadtreeNodeBodies::Bodies(HashSet::new()),
                        square: Square {
                            top_left: current_node.square.top_left
                                + Complex::new(j as f32 * child_size, i as f32 * child_size),
                            size: current_node.square.size / 2.0,
                        },
                        total_mass: 0.0,
                        pos: Complex::ZERO,
                    },
                )
            })
        });
        match &current_node.bodies {
            QuadtreeNodeBodies::All => {
                for body_id in bodies.keys() {
                    let body = bodies.get(body_id).unwrap();

                    let i = ((body.pos.im() - current_node.square.top_left.im()) / child_size)
                        .floor() as usize;
                    let j = ((body.pos.re() - current_node.square.top_left.re()) / child_size)
                        .floor() as usize;

                    let child = &mut children[i][j];

                    if let QuadtreeNodeBodies::Bodies(node_bodies) = &mut child.1.bodies {
                        node_bodies.insert(*body_id);
                    } else {
                        unreachable!()
                    }

                    child.1.total_mass += body.mass;
                    child.1.pos += body.pos * body.mass;
                }
            }
            QuadtreeNodeBodies::Bodies(node_bodies) => {
                for body_id in node_bodies {
                    let body = bodies.get(body_id).unwrap();

                    let i = ((body.pos.im() - current_node.square.top_left.im()) / child_size)
                        .floor() as usize;
                    let j = ((body.pos.re() - current_node.square.top_left.re()) / child_size)
                        .floor() as usize;

                    let child = &mut children[i][j];

                    if let QuadtreeNodeBodies::Bodies(node_bodies) = &mut child.1.bodies {
                        node_bodies.insert(*body_id);
                    } else {
                        unreachable!()
                    }

                    child.1.total_mass += body.mass;
                    child.1.pos += body.pos * body.mass;
                }
            }
        }

        for (child_id, child) in children.iter_mut().flatten() {
            if child.total_mass != 0.0 {
                child.pos /= child.total_mass;
            }

            quadtree_nodes.insert(*child_id, child.clone());
            Self::split(*child_id, bodies, quadtree_nodes);
        }

        let current_node_mut = quadtree_nodes.get_mut(&id).unwrap();
        current_node_mut.children = Some(from_fn(|i| from_fn(|j| children[i][j].0)));
    }
}

pub enum ThetaAdjustment {
    Increase = 1,
    Decrease = -1,
}

pub struct BarnesHut;

impl BarnesHut {
    pub const DRAW: bool = false;
    pub const COLOR: Color = RED;

    pub fn adjust_theta(adjustment: ThetaAdjustment) {
        let mut write = THETA.write().unwrap();
        *write += DELTA_THETA * adjustment as isize as f32;
        *write = write.max(0.0);
    }

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
        quadtree_nodes.insert(
            root_id,
            QuadtreeNode {
                children: None,
                bodies: QuadtreeNodeBodies::All,
                square,
                total_mass: 0.0,
                pos: Complex::ZERO,
            },
        );
        QuadtreeNode::split(root_id, bodies, &mut quadtree_nodes);

        for body_id in bodies_keys {
            QuadtreeNode::adjust_speed(root_id, body_id, bodies, &mut quadtree_nodes);
        }

        let end = start.elapsed();

        if Self::DRAW {
            let root = quadtree_nodes.get(&root_id).unwrap();
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
