mod body;

use ::rand::{Rng, SeedableRng, rngs::StdRng};
use body::{BODIES_N, Body, BodyID, INITIAL_ABS_SPEED, INITIAL_MASS, RADIUS_MULTIPLIER};
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::HashMap,
    f64::consts::{PI, SQRT_2},
};

const DT: f64 = 1.0;

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

    'global: loop {
        let mut bodies_a = HashMap::with_capacity(BODIES_N.get());

        let center = Complex::new(screen_width() as f64 / 2.0, screen_height() as f64 / 2.0);
        let initial_body_radius = Body::get_radius(INITIAL_MASS);
        let cell_side = initial_body_radius * SQRT_2;
        let rows_n = (screen_height() as f64 / cell_side).ceil() as usize;
        let columns_n = (screen_width() as f64 / cell_side).ceil() as usize;

        let mut cells: Vec<Vec<Option<Complex<f64>>>> = vec![vec![None; columns_n]; rows_n];

        for _ in 0..BODIES_N.get() {
            'main: loop {
                let radius = center.re() * rng.random_range(0.0..0.5).sqrt();
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

                let id = BodyID::now();
                bodies_a.insert(id, body);

                let (impossible_positions, _) = Body::get_collision_info(&bodies_a);
                if impossible_positions {
                    bodies_a.remove(&id);
                    continue 'main;
                }

                break;
            }
        }

        let mut bodies_b = bodies_a.clone();

        loop {
            let mut to_remove = Vec::new();

            for (hashmap, lambda, color) in [
                (&mut bodies_b, None, BLUE),
                (&mut bodies_a, Some(DT), WHITE),
            ] {
                if Body::update_bodies(lambda, hashmap) {
                    continue 'global;
                };

                for (body_id, body) in hashmap.iter() {
                    if !(0.0..screen_width() as f64).contains(&body.pos.re())
                        || !(0.0..screen_height() as f64).contains(&body.pos.im())
                    {
                        to_remove.push(*body_id);
                    }
                    draw_circle(
                        body.pos.re() as f32,
                        body.pos.im() as f32,
                        (body.radius * RADIUS_MULTIPLIER) as f32,
                        color,
                    );
                }
            }

            for body_id in &to_remove {
                for hashmap in [&mut bodies_a, &mut bodies_b] {
                    hashmap.remove(body_id);
                }
            }

            next_frame().await;
        }
    }
}
