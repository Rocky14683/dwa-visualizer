use std::vec;
use raylib::color::Color;
use raylib::drawing::{RaylibDraw, RaylibDrawHandle};
use crate::point::{Point, Pose};
use crate::barrier::{BarrierManager};

pub struct Robot {
    size_radius: f64,
    max_velocity: f64,
    max_acceleration: f64,
    pub pose: Pose,
    location_history: Vec<Pose>,
    dwa_path: Vec<Motion>,
}

#[derive(Clone)]
pub enum Motion {
    GENERAL(f64, f64, f64),
    TRANSLATION(f64),
    ROTATION(f64),
}

#[derive(Clone)]
pub struct DwaWeight {
    pub obstacle_weight: f64,
    pub safe_distance: f64,
    pub fwd_weight: f64,
}

impl Robot {
    pub fn new(size_radius: f64, max_velocity: f64, max_acceleration: f64, start_pose: Pose) -> Robot {
        Robot {
            size_radius,
            max_velocity,
            max_acceleration,
            pose: start_pose,
            location_history: vec![start_pose],
            dwa_path: Vec::new()
        }
    }

    pub fn predict(&self, left_v: f64, right_v: f64, dt: f64) -> (Pose, Motion) {
        const DECIMALS: i32 = 3; // Number of decimal places to round to
        let factor = 10f64.powi(DECIMALS);
        let left_v_rounded = (left_v * factor).round() / factor;
        let right_v_rounded = (right_v * factor).round() / factor;
        let mut new_pose = self.pose.clone();
        if left_v_rounded == right_v_rounded {
            // go straight(pure translation) motion
            new_pose.position.x += left_v_rounded * dt * self.pose.orientation.cos();
            new_pose.position.y += left_v_rounded * dt * self.pose.orientation.sin();
            new_pose.orientation = self.pose.orientation;
            return (new_pose, Motion::TRANSLATION(left_v_rounded * dt));
        } else if -left_v_rounded == right_v_rounded {
            // pure turn motion
            new_pose.orientation = self.pose.orientation + ((left_v_rounded - right_v_rounded) * dt / (2.0 * self.size_radius));
            return (new_pose, Motion::ROTATION(0.0));
        }
        // general motion
        let r = self.size_radius * (left_v + right_v) / (right_v - left_v);
        let d_theta = (right_v - left_v) * dt / (2.0 * self.size_radius);
        new_pose.position.x += r * ((d_theta + self.pose.orientation).sin() - self.pose.orientation.sin());
        new_pose.position.y -= r * ((d_theta + self.pose.orientation).cos() - self.pose.orientation.cos());
        new_pose.orientation += d_theta;

        let mut start_angle = self.pose.orientation + std::f64::consts::PI / 2.0;
        if r > 0.0 {
            start_angle -= std::f64::consts::PI;
        }
        let stop_angle = start_angle + d_theta;
        return (new_pose, Motion::GENERAL(r, start_angle, stop_angle));
    }


    pub fn update_pose(&mut self, new_pose: Pose) {
        self.location_history.push(self.pose);
        if self.location_history.len() > 300 {
            self.location_history.remove(0);
        }
        self.pose = new_pose;
    }

    pub fn draw_robot(&self, handle: &mut RaylibDrawHandle, color: Color) {
        // draw wheels
        const WHEEL_BLOB: f64 = 4.0;
        let wheel_1 = Point {
            x: self.pose.position.x - self.size_radius * self.pose.orientation.sin(),
            y: self.pose.position.y + self.size_radius * self.pose.orientation.cos()
        };
        let wheel_2 = Point {
            x: self.pose.position.x + self.size_radius * self.pose.orientation.sin(),
            y: self.pose.position.y - self.size_radius * self.pose.orientation.cos()
        };
        handle.draw_circle_v(wheel_1.to_vector2(), WHEEL_BLOB as f32, Color::BLUE);
        handle.draw_circle_v(wheel_2.to_vector2(), WHEEL_BLOB as f32, Color::BLUE);
        // draw body
        handle.draw_circle_v(self.pose.position.to_vector2(), self.size_radius as f32, color);
    }

    pub fn draw_history(&self, handle: &mut RaylibDrawHandle, color: Color) {
        for pose in &self.location_history {
            handle.draw_circle(pose.position.x as i32, pose.position.y as i32, 3f32, color);
        }
    }

    pub fn draw_dwa_path(&self, handle: &mut RaylibDrawHandle, color: Color) {
        for motion in self.dwa_path.clone() {
            match motion {
                Motion::GENERAL(radius, angle_1, angle_2) => {
                    let (mut start_angle, mut end_angle) = if angle_1 < angle_2 {
                        (angle_1, angle_2)
                    } else {
                        (angle_2, angle_1)
                    };

                    if start_angle < 0.0 {
                        start_angle += 2.0 * std::f64::consts::PI;
                        end_angle += 2.0 * std::f64::consts::PI;
                    }
                    handle.draw_circle_sector_lines(self.pose.position.to_vector2(),
                                                    radius as f32,
                                                    start_angle as f32,
                                                    end_angle as f32, 100, color);
                }
                Motion::TRANSLATION(dist) => {
                    let end_point = Point {
                        x: self.pose.position.x + dist * self.pose.orientation.cos(),
                        y: self.pose.position.y + dist * self.pose.orientation.sin(),
                    };
                    handle.draw_line_v(self.pose.position.to_vector2(), end_point.to_vector2(), color);
                }
                Motion::ROTATION(_) => { continue; }
            }
        }
    }

    pub fn dwa(&mut self, left_v: f64, right_v: f64, dt: f64, barriers: &BarrierManager, weight: DwaWeight, step_plan_ahead: u32) -> (f64, f64) {
        self.dwa_path.clear();
        let left_possible_v = [left_v - self.max_acceleration * dt, left_v, left_v + self.max_acceleration * dt];
        let right_possible_v = [right_v - self.max_acceleration * dt, right_v, right_v + self.max_acceleration * dt];
        let target = barriers.get_target().point;
        let mut left_v_chosen = 0.0;
        let mut right_v_chosen = 0.0;
        let mut best_benefit = f64::MIN;
        for left_v in left_possible_v {
            for right_v in right_possible_v {
                if left_v <= self.max_velocity && left_v >= -self.max_velocity && right_v <= self.max_velocity && right_v >= -self.max_velocity {
                    let (new_pose, motion) = self.predict(left_v, right_v, dt * step_plan_ahead as f64);
                    self.dwa_path.push(motion.clone());

                    let distance_to_barrier = barriers.get_shortest_to_barrier(new_pose);
                    let prev_target_dist = self.pose.to_point().get_distance(target.clone());
                    let new_target_dist = new_pose.to_point().get_distance(target.clone());

                    let fwd_dist = prev_target_dist - new_target_dist;

                    let distance_benefit = weight.fwd_weight * fwd_dist;

                    let obstacle_cost = if distance_to_barrier < weight.safe_distance {
                        weight.obstacle_weight * (weight.safe_distance - distance_to_barrier)
                    } else {
                        0.0
                    };

                    let total_benefit = distance_benefit - obstacle_cost;
                    if total_benefit > best_benefit {
                        best_benefit = total_benefit;
                        left_v_chosen = left_v;
                        right_v_chosen = right_v;
                    }
                }
            }
        }
        println!("size: {}", self.dwa_path.len());
        (left_v_chosen, right_v_chosen)
    }

    pub fn check_target_reached(&self, barrier_manager: &mut BarrierManager) {
        let target = barrier_manager.get_target();
        let dist = self.pose.position.get_distance(target.point);
        if dist < self.size_radius + target.radius {
            barrier_manager.set_random_as_target();
        }
    }
}