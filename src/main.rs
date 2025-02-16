mod body;
mod grid;
mod quadtree;

use ::rand::{Rng, SeedableRng, rngs::StdRng};
use body::{BODIES_N, Body, BodyID};
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use quadtree::{NodeID, QuadtreeNode, Square};
use std::{collections::HashMap, f32::consts::PI, time::Instant};

const G: f32 = 0.05;
const INITIAL_MASS: f32 = 0.5;
const INITIAL_ABS_SPEED: f32 = 0.05;

const FONT_SIZE: u16 = 50;

const DIRECT_COLOR: Color = GREEN;
const BARNES_HUT_COLOR: Color = RED;

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

    let mut quadtree_nodes: HashMap<NodeID, QuadtreeNode> = HashMap::new();
    let mut bodies = HashMap::with_capacity(BODIES_N);

    let eccentricity = (1.0 - (screen_height() / screen_width()).powi(2)).sqrt();

    let center = Complex::new(screen_width() / 2.0, screen_height() / 2.0);

    let mut total_speed = Complex::ZERO;
    for _ in 0..BODIES_N {
        let angle = rng.random_range(0.0..2.0 * PI);
        let radius =
            (screen_height() / 2.0) / ((1.0 - (eccentricity * angle.cos()).powi(2)) as f32).sqrt();
        let distance_from_center = radius * (rng.random_range(0.0..1.0).sqrt());
        let body = Body {
            pos: center + Complex::from_polar(distance_from_center, angle),
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
            zoom /= 2.0;
        } else if is_key_released(KeyCode::Equal) {
            zoom *= 2.0;
        }

        camera.zoom = vec2(
            1.0 / screen_width() * 2.0 * zoom,
            1.0 / screen_height() * 2.0 * zoom,
        );

        set_camera(&camera);

        Body::update_bodies(&mut bodies);

        let duration = Body::handle_direct_method(&mut bodies)
            .as_millis()
            .to_string();
        let measured = measure_text(&duration, None, FONT_SIZE, 1.0);

        draw_text(
            &format!("Direct: {} ms", duration),
            0.0,
            measured.height,
            FONT_SIZE as f32,
            DIRECT_COLOR,
        );

        Body::update_bodies(&mut barnes_hut_bodies);

        let duration = Body::handle_barnes_hut(&mut barnes_hut_bodies, &mut quadtree_nodes, zoom)
            .as_millis()
            .to_string();

        draw_text(
            &format!("Barnes-Hut: {} ms", duration),
            0.0,
            measured.height * 2.5,
            FONT_SIZE as f32,
            BARNES_HUT_COLOR,
        );

        for (hashmap, color) in [
            (&barnes_hut_bodies, BARNES_HUT_COLOR),
            (&bodies, DIRECT_COLOR),
        ] {
            for body in hashmap.values() {
                draw_circle(body.pos.re(), body.pos.im(), body.get_radius(), color);
            }
        }

        next_frame().await
    }
}
