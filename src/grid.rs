use crate::{BORDER_COLOR, BORDER_THICKNESS, Body, BodyID, Zoom, body::get_rectangle};
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::{HashMap, HashSet},
    time::{Duration, Instant},
};

pub const TAU: f64 = 0.3;

#[derive(Clone)]
pub struct Cell {
    pub bodies: HashSet<BodyID>,
    pub total_mass: f64,
    pub pos: Complex<f64>,
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

pub struct Grid;

impl Grid {
    pub const DRAW: bool = false;
    pub const COLOR: Color = BLUE;

    pub fn handle(bodies: &mut HashMap<BodyID, Body>, zoom: &Zoom) -> Duration {
        let start = Instant::now();

        let rectangle = get_rectangle(bodies);

        let width = rectangle.bottom_right.re() - rectangle.top_left.re();
        let height = rectangle.bottom_right.im() - rectangle.top_left.im();

        let target_size = ((TAU * width * height) / (bodies.len() as f64).sqrt()).sqrt();

        let rows_n = ((height / target_size).round() as usize).max(1);
        let cell_height = height / (rows_n as f64);

        let columns_n = ((width / cell_height).round() as usize).max(1);
        let cell_width = width / (columns_n as f64);

        let mut cells = vec![
            vec![
                Cell {
                    bodies: HashSet::with_capacity((TAU * (bodies.len() as f64).sqrt()) as usize),
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
                    let lhs_body = bodies.get_mut(lhs_body_id).unwrap();
                    for (m, row) in cells.iter().enumerate() {
                        for (n, cell) in row.iter().enumerate() {
                            if (i.saturating_sub(1)..=(i + 1).min(rows_n - 1)).contains(&m)
                                && (j.saturating_sub(1)..=(j + 1).min(columns_n - 1)).contains(&n)
                            {
                                for rhs_body_id in &cell.bodies {
                                    if lhs_body_id != rhs_body_id {
                                        let rhs_body = bodies_clone.get(rhs_body_id).unwrap();

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

        if Self::DRAW {
            let border = BORDER_THICKNESS / zoom.zoom;

            for i in 0..=rows_n {
                draw_line(
                    rectangle.top_left.re() as f32,
                    rectangle.top_left.im() as f32 + i as f32 * cell_height as f32,
                    rectangle.bottom_right.re() as f32,
                    rectangle.top_left.im() as f32 + i as f32 * cell_height as f32,
                    border,
                    BORDER_COLOR,
                );
            }

            for j in 0..=columns_n {
                draw_line(
                    rectangle.top_left.re() as f32 + j as f32 * cell_width as f32,
                    rectangle.top_left.im() as f32,
                    rectangle.top_left.re() as f32 + j as f32 * cell_width as f32,
                    rectangle.bottom_right.im() as f32,
                    border,
                    BORDER_COLOR,
                );
            }
        }

        end
    }
}
