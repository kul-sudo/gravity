mod body;
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

    let mut quadtree_nodes: HashMap<NodeID, QuadtreeNode> = HashMap::new();

    let mut bodies = HashMap::with_capacity(BODIES_N);

    let mut total_speed = Complex::new(0.0, 0.0);
    for _ in 0..BODIES_N {
        let body = Body {
            pos: Complex::new(
                rng.random_range(0.0..screen_width()),
                rng.random_range(0.0..screen_height()),
            ),
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

    loop {
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

        let duration = Body::handle_barnes_hut(&mut barnes_hut_bodies, &mut quadtree_nodes)
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
