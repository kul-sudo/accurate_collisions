use crate::DT;
use macroquad::prelude::*;
use num_complex::{Complex, ComplexFloat};
use std::{collections::HashMap, num::NonZero, time::Instant};

pub const BODIES_N: NonZero<usize> = NonZero::new(300).unwrap();

pub const INITIAL_MASS: f64 = 0.05;
pub const LINE_THICKNESS: f32 = 5.0;
pub const INITIAL_ABS_SPEED: f64 = 0.9;

pub const RADIUS_MULTIPLIER: f64 = 5.0;

pub type BodyID = Instant;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Body {
    pub pos: Complex<f64>,
    pub speed: Complex<f64>,
    pub mass: f64,
    pub radius: f64,
}

impl Body {
    pub fn get_radius(mass: f64) -> f64 {
        mass.powf(1.0 / 3.0)
    }

    pub fn connect(pair: [BodyID; 2], bodies: &mut HashMap<BodyID, Body>) {
        let mass = pair
            .iter()
            .map(|body_id| bodies.get(body_id).unwrap().mass)
            .sum::<f64>();
        let pos = pair
            .iter()
            .map(|body_id| {
                let body = bodies.get(body_id).unwrap();
                body.mass * body.pos
            })
            .sum::<Complex<f64>>()
            / mass;
        let speed = pair
            .iter()
            .map(|body_id| {
                let body = bodies.get(body_id).unwrap();
                body.mass * body.speed
            })
            .sum::<Complex<f64>>()
            / mass;

        bodies.remove(&pair[0]);
        bodies.remove(&pair[1]);

        bodies.insert(
            BodyID::now(),
            Body {
                pos,
                speed,
                mass,
                radius: Self::get_radius(mass),
            },
        );
    }

    pub fn connect_all(bodies: &mut HashMap<BodyID, Body>) {
        loop {
            let mut deepest_connection_depth = f64::MIN;
            let mut deepest_connection_pair: Option<[BodyID; 2]> = None;

            let keys = bodies.keys().collect::<Vec<_>>();
            for (lhs_index, lhs_body_id) in keys.iter().enumerate() {
                let lhs_body = bodies.get(lhs_body_id).unwrap();
                for rhs_body_id in &keys[lhs_index + 1..] {
                    let rhs_body = bodies.get(rhs_body_id).unwrap();

                    let depth =
                        lhs_body.radius + rhs_body.radius - (lhs_body.pos - rhs_body.pos).abs();

                    if depth >= 0.0 && depth > deepest_connection_depth {
                        deepest_connection_depth = depth;
                        deepest_connection_pair = Some([**lhs_body_id, **rhs_body_id]);
                    }
                }
            }

            match deepest_connection_pair {
                Some(pair) => {
                    Self::connect(pair, bodies);
                }
                None => break,
            }
        }
    }

    pub fn get_collision_info(
        bodies: &HashMap<BodyID, Body>,
    ) -> (bool, Option<(f64, [BodyID; 2])>) {
        let mut earliest_collision_time = f64::INFINITY;
        let mut earliest_collision_pair: Option<[BodyID; 2]> = None;

        let mut impossible_positions = false;

        let keys = bodies.keys().collect::<Vec<_>>();
        for (lhs_index, lhs_body_id) in keys.iter().enumerate() {
            let lhs_body = bodies.get(lhs_body_id).unwrap();
            for rhs_body_id in &keys[lhs_index + 1..] {
                let rhs_body = bodies.get(rhs_body_id).unwrap();

                if lhs_body.speed != rhs_body.speed {
                    let dspeed = lhs_body.speed - rhs_body.speed;
                    let a = dspeed.abs().powi(2);

                    let dpos = lhs_body.pos - rhs_body.pos;

                    let r = lhs_body.radius + rhs_body.radius;

                    let b = (dpos.re() * dspeed.re() + dpos.im() * dspeed.im()) * 2.0;
                    let c = dpos.abs().powi(2) - r.powi(2);
                    let d = b.powi(2) - 4.0 * a * c;

                    let d_sqrt = d.sqrt();
                    if !d_sqrt.is_nan() {
                        // sqrt(n < 0) = NaN
                        if d_sqrt >= b {
                            let t_min = -(b + d_sqrt) / (2.0 * a);

                            if t_min < earliest_collision_time {
                                earliest_collision_time = t_min;
                                earliest_collision_pair = Some([**lhs_body_id, **rhs_body_id]);
                            }
                        } else {
                            impossible_positions = true;
                        }
                    }
                }
            }
        }

        (
            impossible_positions,
            earliest_collision_pair.map(|pair| (earliest_collision_time, pair)),
        )
    }

    pub fn update_bodies(maybe_lambda: Option<f64>, bodies: &mut HashMap<BodyID, Body>) -> bool {
        match maybe_lambda {
            Some(lambda) => {
                let (_, earliest_collision) = Self::get_collision_info(bodies);

                match earliest_collision {
                    Some((time, pair)) => {
                        // Only handles one collisions at a time, because the chance of multiple
                        // collisions occurring at the same moment is extremely low
                        for body_id in pair {
                            let body = bodies.get(&body_id).unwrap();
                            let collision_pos = body.pos + body.speed * time;
                            draw_line(
                                body.pos.re() as f32,
                                body.pos.im() as f32,
                                collision_pos.re() as f32,
                                collision_pos.im() as f32,
                                LINE_THICKNESS,
                                RED,
                            );
                        }

                        if time <= lambda {
                            for body in bodies.values_mut() {
                                body.pos += body.speed * time;
                            }

                            Self::connect(pair, bodies);
                            Self::connect_all(bodies);

                            if time < lambda {
                                return Self::update_bodies(Some(lambda - time), bodies);
                            }
                        } else {
                            for body in bodies.values_mut() {
                                body.pos += body.speed * lambda;
                            }
                        }
                    }
                    None => {
                        for body in bodies.values_mut() {
                            body.pos += body.speed * lambda;
                        }

                        return true;
                    }
                }
            }
            None => {
                for body in bodies.values_mut() {
                    body.pos += body.speed * DT;
                }

                Self::connect_all(bodies);
            }
        }

        false
    }
}
