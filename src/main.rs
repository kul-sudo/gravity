mod barnes_hut;
mod body;
mod direct;
mod grid;

use ::rand::{Rng, SeedableRng, rngs::StdRng};
use barnes_hut::BarnesHut;
use body::{BODIES_N, Body, BodyID};
use direct::Direct;
use grid::Grid;
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{collections::HashMap, f32::consts::PI};

const G: f32 = 0.05;
const INITIAL_MASS: f32 = 1.0;
const INITIAL_ABS_SPEED: f32 = 0.05;

const ZOOM_STEP: f32 = 1.2;
const FONT_SIZE: u16 = 50;

pub const BORDER_THICKNESS: f32 = 2.0;
pub const BORDER_COLOR: Color = GREEN;

fn window_conf() -> Conf {
    Conf {
        window_title: "gravity".to_owned(),
        fullscreen: true,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut rng = StdRng::from_os_rng();

    set_fullscreen(true);
    next_frame().await;

    let mut zoom = 1.0;

    let mut camera =
        Camera2D::from_display_rect(Rect::new(0.0, 0.0, screen_width(), screen_height()));

    let mut bodies = HashMap::with_capacity(BODIES_N);

    let center = Complex::new(screen_width() / 2.0, screen_height() / 2.0);

    let mut total_speed = Complex::ZERO;
    for _ in 0..BODIES_N {
        let radius = center.re() * rng.random_range(0.0..1.0).sqrt();
        let fi = 2.0 * PI * rng.random_range(0.0..1.0);

        let body = Body {
            pos: center + Complex::new(radius * fi.cos(), center.im() / center.re() * radius * fi.sin()),
            speed: Complex::from_polar(INITIAL_ABS_SPEED, rng.random_range(0.0..2.0 * PI)),
            mass: INITIAL_MASS,
        };

        bodies.insert(BodyID::now(), body);

        total_speed += body.speed;
    }

    let delta_speed = -total_speed / BODIES_N as f32;

    for body in bodies.values_mut() {
        body.speed += delta_speed;
    }

    let mut barnes_hut_bodies = bodies.clone();
    let mut grid_bodies = bodies.clone();

    loop {
        if is_key_released(KeyCode::Minus) {
            zoom /= ZOOM_STEP;
        } else if is_key_released(KeyCode::Equal) {
            zoom *= ZOOM_STEP;
        }

        camera.zoom = vec2(2.0 / screen_width() * zoom, 2.0 / screen_height() * zoom);

        set_camera(&camera);

        // Direct
        Body::update_bodies(&mut bodies);

        let duration = Direct::handle(&mut bodies).as_millis().to_string();
        let measured = measure_text(&duration, None, FONT_SIZE, 1.0);

        draw_text(
            &format!("Direct: {} ms", duration),
            0.0,
            measured.height,
            FONT_SIZE as f32,
            Direct::COLOR,
        );

        // Barnes-Hut
        Body::update_bodies(&mut barnes_hut_bodies);

        let duration = BarnesHut::handle(&mut barnes_hut_bodies, zoom)
            .as_millis()
            .to_string();

        draw_text(
            &format!("Barnes-Hut: {} ms", duration),
            0.0,
            measured.height * 2.0,
            FONT_SIZE as f32,
            BarnesHut::COLOR,
        );

        // Grid
        Body::update_bodies(&mut grid_bodies);

        let duration = Grid::handle(&mut grid_bodies, zoom).as_millis().to_string();

        draw_text(
            &format!("Grid: {} ms", duration),
            0.0,
            measured.height * 3.0,
            FONT_SIZE as f32,
            Grid::COLOR,
        );

        for (hashmap, color) in [
            (&grid_bodies, Grid::COLOR),
            (&barnes_hut_bodies, BarnesHut::COLOR),
            (&bodies, Direct::COLOR),
        ] {
            for body in hashmap.values() {
                draw_circle(body.pos.re(), body.pos.im(), body.get_radius(), color);
            }
        }

        next_frame().await
    }
}
