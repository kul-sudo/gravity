mod barnes_hut;
mod body;
mod direct;
mod grid;
mod zoom;

use ::rand::{Rng, SeedableRng, rngs::StdRng};
use barnes_hut::{BarnesHut, ThetaAdjustment};
use body::{BODIES_N, Body, BodyID};
use direct::Direct;
use grid::Grid;
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::HashMap,
    f64::consts::{PI, SQRT_2},
};
use zoom::Zoom;
use zoom::{ZOOM_RANGE, ZOOM_STEP};

const MAX_AVERAGE_LENGTH: usize = 100;

const G: f64 = 0.05;
const INITIAL_MASS: f64 = 1.0;
const INITIAL_ABS_SPEED: f64 = 0.05;

const FONT_SIZE: u16 = 50;

const DT: f64 = 1.0;

pub const BORDER_THICKNESS: f32 = 2.0;
pub const BORDER_COLOR: Color = GREEN;

fn window_conf() -> Conf {
    Conf {
        window_title: "gravity".to_owned(),
        fullscreen: true,
        platform: miniquad::conf::Platform {
            linux_backend: miniquad::conf::LinuxBackend::WaylandWithX11Fallback,
            ..Default::default()
        },
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut rng = StdRng::from_os_rng();

    for _ in 0..8 {
        set_fullscreen(true);
        next_frame().await;
    }

    let mut zoom = Zoom { zoom: 1.0 };

    let mut camera =
        Camera2D::from_display_rect(Rect::new(0.0, 0.0, screen_width(), screen_height()));

    let mut bodies = HashMap::with_capacity(BODIES_N);

    let center = Complex::new(screen_width() as f64 / 2.0, screen_height() as f64 / 2.0);
    let initial_body_radius = Body::get_radius(INITIAL_MASS);
    let cell_side = initial_body_radius * SQRT_2;
    let rows_n = (screen_height() as f64 / cell_side).ceil() as usize;
    let columns_n = (screen_width() as f64 / cell_side).ceil() as usize;

    let mut cells: Vec<Vec<Option<Complex<f64>>>> = vec![vec![None; columns_n]; rows_n];

    for _ in 0..BODIES_N {
        'main: loop {
            let radius = center.re() * rng.random_range(0.0..1.0).sqrt();
            let angle = rng.random_range(0.0..2.0 * PI);
            let pos = center
                + Complex::new(
                    radius * angle.cos(),
                    center.im() / center.re() * radius * angle.sin(),
                );

            let i = (pos.im() / cell_side) as usize;
            let j = (pos.re() / cell_side) as usize;

            for m in i.saturating_sub(2)..(i + 3).min(rows_n) {
                for n in j.saturating_sub(2)..(j + 3).min(columns_n) {
                    if let Some(cell_pos) = cells[m][n] {
                        if (cell_pos - pos).abs() <= initial_body_radius * 2.0 {
                            continue 'main;
                        }
                    }
                }
            }

            cells[i][j] = Some(pos);

            let body = Body {
                pos,
                speed: Complex::from_polar(INITIAL_ABS_SPEED, rng.random_range(0.0..2.0 * PI)),
                mass: INITIAL_MASS,
                radius: initial_body_radius,
            };
            bodies.insert(BodyID::now(), body);

            break;
        }
    }

    Body::adjust_momentum(&mut bodies);

    let mut barnes_hut_bodies = bodies.clone();
    let mut grid_bodies = bodies.clone();

    let mut direct_durations = Vec::with_capacity(MAX_AVERAGE_LENGTH);
    let mut barnes_hut_durations = direct_durations.clone();
    let mut grid_durations = direct_durations.clone();

    let mut always_use_direct = false;

    loop {
        let mut update = false;
        let mut new_zoom = None;

        if is_key_down(KeyCode::Minus) {
            new_zoom = Some(Zoom {
                zoom: zoom.zoom / ZOOM_STEP,
            });
            update = true;
        } else if is_key_down(KeyCode::Equal) {
            new_zoom = Some(Zoom {
                zoom: zoom.zoom * ZOOM_STEP,
            });
            update = true;
        } else if is_key_pressed(KeyCode::Key0) {
            zoom.zoom = 1.0;
            update = true;
        } else if is_key_pressed(KeyCode::Space) {
            always_use_direct = true;
        }

        if update {
            if let Some(new_zoom) = new_zoom {
                if ZOOM_RANGE.contains(&new_zoom.zoom) {
                    zoom = new_zoom
                }
            }

            camera.zoom = vec2(
                2.0 / screen_width() * zoom.zoom,
                2.0 / screen_height() * zoom.zoom,
            );

            set_camera(&camera);
        }

        // Direct
        Body::update_bodies(DT, &mut bodies);
        //Body::adjust_momentum(&mut bodies);

        let duration_direct = Direct::handle(&mut bodies).as_nanos() as f64 / bodies.len() as f64;
        if direct_durations.len() == MAX_AVERAGE_LENGTH {
            direct_durations.clear();
        }
        direct_durations.push(duration_direct);

        let direct_average = direct_durations.iter().sum::<f64>() / direct_durations.len() as f64;

        // Barnes-Hut
        Body::update_bodies(DT, &mut barnes_hut_bodies);
        Body::adjust_momentum(&mut barnes_hut_bodies);

        let duration_barnes_hut = if always_use_direct {
            Direct::handle(&mut barnes_hut_bodies)
        } else {
            BarnesHut::handle(&mut barnes_hut_bodies, &zoom)
        }
        .as_nanos() as f64
            / barnes_hut_bodies.len() as f64;

        if barnes_hut_durations.len() == MAX_AVERAGE_LENGTH {
            barnes_hut_durations.clear();
        }
        barnes_hut_durations.push(duration_barnes_hut);

        let barnes_hut_average =
            barnes_hut_durations.iter().sum::<f64>() / barnes_hut_durations.len() as f64;

        // Grid
        Body::update_bodies(DT, &mut grid_bodies);
        Body::adjust_momentum(&mut grid_bodies);

        let duration_grid = if always_use_direct {
            Direct::handle(&mut grid_bodies)
        } else {
            Grid::handle(&mut grid_bodies, &zoom)
        }
        .as_nanos() as f64
            / grid_bodies.len() as f64;

        if grid_durations.len() == MAX_AVERAGE_LENGTH {
            grid_durations.clear();
        }
        grid_durations.push(duration_grid);

        let grid_average = grid_durations.iter().sum::<f64>() / grid_durations.len() as f64;

        if !always_use_direct {
            BarnesHut::adjust_theta(if duration_barnes_hut <= duration_grid {
                ThetaAdjustment::Decrease
            } else {
                ThetaAdjustment::Increase
            });
        }

        for (hashmap, color) in [
            (&grid_bodies, Grid::COLOR),
            (&barnes_hut_bodies, BarnesHut::COLOR),
            (&bodies, Direct::COLOR),
        ] {
            for body in hashmap.values() {
                draw_circle(
                    body.pos.re() as f32,
                    body.pos.im() as f32,
                    body.radius as f32,
                    color,
                );
            }
        }

        let rect = zoom.get_rect();
        let mut measured = None;
        for (index, (name, color, average)) in [
            ("Direct", Direct::COLOR, direct_average),
            ("Barnes-Hut", BarnesHut::COLOR, barnes_hut_average),
            ("Grid", Grid::COLOR, grid_average),
        ]
        .iter()
        .enumerate()
        {
            if measured.is_none() {
                measured = Some(measure_text(
                    &direct_average.to_string(),
                    None,
                    FONT_SIZE,
                    1.0,
                ));
            }

            draw_text_ex(
                &format!("{}: {}", name, *average as usize),
                rect.top_left.re() as f32,
                rect.top_left.im() as f32
                    + measured.unwrap().height * (index + 1) as f32 / zoom.zoom,
                TextParams {
                    font: None,
                    font_size: FONT_SIZE,
                    font_scale: 1.0 / zoom.zoom,
                    font_scale_aspect: 1.0,
                    rotation: 0.0,
                    color: *color,
                },
            );
        }

        let text = &format!("Always use direct: {}", always_use_direct);
        let measured = measure_text(text, None, FONT_SIZE, 1.0);
        draw_text_ex(
            text,
            rect.bottom_right.re() as f32 - measured.width / zoom.zoom,
            rect.top_left.im() as f32 + measured.height / zoom.zoom,
            TextParams {
                font: None,
                font_size: FONT_SIZE,
                font_scale: 1.0 / zoom.zoom,
                font_scale_aspect: 1.0,
                rotation: 0.0,
                color: WHITE,
            },
        );

        next_frame().await;
    }
}
