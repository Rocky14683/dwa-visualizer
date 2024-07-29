use rand::{random, Rng};
use crate::point::{Point, Pose};
use raylib::drawing::*;
use raylib::color::Color;

#[derive(Clone)]
pub struct Barrier {
    pub point: Point,
    pub radius: f64,
}

impl Barrier {
    pub fn new(x: f64, y: f64, radius: f64) -> Barrier {
        Barrier {
            point: Point { x, y },
            radius,
        }
    }

    pub fn get_point(&self) -> Point {
        self.point
    }

    pub fn get_distance(&self, other: Point) -> f64 {
        self.point.get_distance(other)
    }

    pub fn draw(&self, d: &mut RaylibDrawHandle, color: Color) {
        d.draw_circle(self.point.x as i32, self.point.y as i32, self.radius as f32, color);
    }
}

pub struct BarrierManager {
    pub barriers: Vec<Barrier>,
    target_idx: usize,
    window_width: i32,
    window_height: i32,
}

impl BarrierManager {
    pub fn new(size: usize, radius: f64, window_width: i32, window_height: i32) -> BarrierManager {
        let mut barriers = Vec::new();
        let mut rng = rand::thread_rng();
        for _ in 0..size {
            let x = rng.gen_range(radius as i32..=(window_width - radius as i32)) as f64;
            let y = rng.gen_range(radius as i32..=(window_height - radius as i32)) as f64;
            let barrier = Barrier::new(x, y, radius);
            barriers.push(barrier);
        }
        let target_idx = random::<usize>() % size;

        BarrierManager {
            barriers,
            target_idx,
            window_width,
            window_height,
        }
    }
    pub fn get_shortest_to_barrier(&self, position: Pose) -> f64 {
        let mut shortest = f64::MAX;
        for barrier in self.barriers.iter().skip(self.target_idx) {
            let distance = barrier.get_distance(position.position.clone());
            if distance < shortest {
                shortest = distance;
            }
        }
        shortest
    }

    pub fn get_size(&self) -> usize {
        self.barriers.len()
    }

    pub fn set_random_as_target(&mut self) {
        let mut rng = rand::thread_rng();
        self.target_idx = rng.gen_range(0..self.get_size());
        println!("random target idx set: {}", self.target_idx);
    }

    pub fn get_target(&self) -> Barrier {
        self.barriers[self.target_idx].clone()
    }

    pub fn draw_barriers(&self, d: &mut RaylibDrawHandle, color: Color, target_color: Color) {
        for barrier in self.barriers.iter().skip(self.target_idx) {
            barrier.draw(d, color);
        }
        self.barriers[self.target_idx].draw(d, target_color);
    }
}