mod body;
mod quadtree;

use ::rand::{Rng, SeedableRng, rngs::StdRng};
use body::{BODIES_N, Body, BodyID};
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use quadtree::{NodeID, QuadtreeNode, Square};
use std::{collections::HashMap, f32::consts::PI, time::Instant};

const G: f32 = 0.05;
const INITIAL_MASS: f32 = 1.0;
const INITIAL_ABS_SPEED: f32 = 0.05;

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
        Body::update_bodies(&mut barnes_hut_bodies);

        let duration_barnes_hut =
            Body::handle_barnes_hut(&mut barnes_hut_bodies, &mut quadtree_nodes);
        draw_text(
            &duration_barnes_hut.as_millis().to_string(),
            20.0,
            20.0,
            25.0,
            WHITE,
        );

        Body::update_bodies(&mut bodies);

        let duration_direct = Body::handle_direct_method(&mut bodies);
        draw_text(&duration_direct.as_millis().to_string(), 20.0, 40.0, 25.0, WHITE);

        for (hashmap, color) in [
            (&barnes_hut_bodies, RED),
            (&bodies, GREEN)
        ] {
            for body in hashmap.values() {
                draw_circle(body.pos.re(), body.pos.im(), body.get_radius(), color);
            }
        }

        next_frame().await
    }
}
