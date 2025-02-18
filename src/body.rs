use crate::{
    G, NodeID, QuadtreeNode, Square,
    grid::{Cell, TAU},
    quadtree::{BORDER_COLOR, BORDER_THICKNESS, Rectangle},
};
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::{HashMap, HashSet},
    time::{Duration, Instant},
};

pub const BODIES_N: usize = 1200;

pub type BodyID = Instant;

#[derive(Clone, Copy)]
pub struct Body {
    pub pos: Complex<f32>,
    pub speed: Complex<f32>,
    pub mass: f32,
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

    pub fn adjust_speed(&mut self, pos: Complex<f32>, mass: f32) {
        let r = pos - self.pos;
        self.speed += G * mass * r / r.abs().powi(3);
    }

    pub fn handle_barnes_hut(bodies: &mut HashMap<BodyID, Self>, zoom: f32) -> Duration {
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

        let border = BORDER_THICKNESS / zoom;

        //draw_rectangle_lines(
        //    root.square.top_left.re(),
        //    root.square.top_left.im(),
        //    root.square.size,
        //    root.square.size,
        //    border,
        //    BORDER_COLOR,
        //);

        //QuadtreeNode::draw(root_id, &mut quadtree_nodes, zoom);

        return end;
    }

    pub fn handle_grid(bodies: &mut HashMap<BodyID, Self>) -> Duration {
        let start = Instant::now();

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

        for (body_id, body) in bodies.iter() {
            cells[((body.pos.im() - rectangle.top_left.im()) / cell_height) as usize]
                [((body.pos.re() - rectangle.top_left.re()) / cell_width) as usize]
                .add_body(*body_id, bodies);
        }

        for cell in cells.iter_mut().flatten() {
            cell.set_pos()
        }

        let bodies_clone = bodies.clone();
        for i in 0..rows_n {
            for j in 0..columns_n {
                for lhs_body_id in &cells[i][j].bodies {
                    let lhs_body = bodies.get_mut(&lhs_body_id).unwrap();
                    for m in 0..rows_n {
                        for n in 0..columns_n {
                            let cell = &cells[m][n];

                            if (i.saturating_sub(1)..=(i + 1).min(rows_n)).contains(&m)
                                && (j.saturating_sub(1)..=(j + 1).min(columns_n)).contains(&n)
                            {
                                for rhs_body_id in &cell.bodies {
                                    if lhs_body_id != rhs_body_id {
                                        let rhs_body = bodies_clone.get(&rhs_body_id).unwrap();

                                        lhs_body.adjust_speed(rhs_body.pos, rhs_body.mass)
                                    }
                                }
                            } else {
                                lhs_body.adjust_speed(cell.pos, cell.total_mass)
                            }
                        }
                    }
                }
            }
        }

        let end = start.elapsed();

        //for i in 0..=rows_n {
        //    draw_line(
        //        rectangle.top_left.re(),
        //        rectangle.top_left.im() + i as f32 * cell_height,
        //        rectangle.bottom_right.re(),
        //        rectangle.top_left.im() + i as f32 * cell_height,
        //        5.0,
        //        GREEN,
        //    );
        //}

        //for j in 0..=columns_n {
        //    draw_line(
        //        rectangle.top_left.re() + j as f32 * cell_width,
        //        rectangle.top_left.im(),
        //        rectangle.top_left.re() + j as f32 * cell_width,
        //        rectangle.bottom_right.im(),
        //        5.0,
        //        GREEN,
        //    );
        //}

        return end;
    }
}
