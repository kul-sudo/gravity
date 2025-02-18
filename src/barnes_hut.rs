use crate::{
    BORDER_COLOR, BORDER_THICKNESS, Body, BodyID, NodeID, QuadtreeNode, Square, body::get_rectangle,
};
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::{HashMap, HashSet},
    time::{Duration, Instant},
};

pub struct BarnesHut;

impl BarnesHut {
    pub const DRAW: bool = false;
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
