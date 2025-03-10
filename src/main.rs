mod body;

use ::rand::{Rng, SeedableRng, rngs::StdRng};
use body::{BODIES_N, Body, BodyID};
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{
    collections::HashMap,
    f64::consts::{PI, SQRT_2},
    num::NonZero,
};

const INITIAL_MASS: f64 = 1.0;
const INITIAL_ABS_SPEED: f64 = 0.05;

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

    let mut bodies = HashMap::with_capacity(BODIES_N.get());

    let center = Complex::new(screen_width() as f64 / 2.0, screen_height() as f64 / 2.0);
    let initial_body_radius = Body::get_radius(INITIAL_MASS);
    let cell_side = initial_body_radius * SQRT_2;
    let rows_n = (screen_height() as f64 / cell_side).ceil() as usize;
    let columns_n = (screen_width() as f64 / cell_side).ceil() as usize;

    let mut cells: Vec<Vec<Option<Complex<f64>>>> = vec![vec![None; columns_n]; rows_n];

    for _ in 0..BODIES_N.get() {
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
                color: WHITE,
            };
            bodies.insert(BodyID::now(), body);

            break;
        }
    }

    loop {
        for body in bodies.values() {
            draw_circle(
                body.pos.re() as f32,
                body.pos.im() as f32,
                body.radius as f32,
                body.color,
            );
        }

        next_frame().await;
    }
}
