use raylib::prelude::*;
use crate::barrier::BarrierManager;
use crate::robot::{DwaWeight, Robot};
use crate::point::*;

mod robot;
mod point;
mod barrier;

const WIDTH: i32 = 1200;
const HEIGHT: i32 = 750;


fn main() {
    let (mut rl, thread) = init()
        .size(WIDTH, HEIGHT)
        .title("MAP")
        .build();

    let mut robot = Robot::new(20.0, 100.0, 5.0,
                               Pose {
                                   position: Point { x: (WIDTH / 2) as f64, y: (HEIGHT / 2) as f64 },
                                   orientation: std::f64::consts::PI / 2.0,
                               });

    let mut barriers = BarrierManager::new(20, 20.0, WIDTH, HEIGHT);
    let (mut left_v,mut right_v) = (0.0, 0.0);

    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::WHITE);

        (left_v, right_v) = robot.dwa(left_v, right_v, 0.01, &barriers, DwaWeight{
            obstacle_weight: 1000.0,
            safe_distance: 50.0,
            fwd_weight: 100.0,
            angle_weight: 200.0,
        }, 10);

        println!("left_v: {}, right_v: {}", left_v.clone(), right_v.clone());

        robot.check_target_reached(&mut barriers);
        let (new_pose, _) = robot.predict(left_v, right_v, 0.01);
        robot.update_pose(new_pose);



        robot.draw_history(&mut d, Color::GRAY.alpha(0.2));
        robot.draw_robot(&mut d, Color::ORANGE);
        robot.draw_dwa_path(&mut d, Color::RED);
        barriers.draw_barriers(&mut d, Color::DARKCYAN, Color::YELLOWGREEN);
    }
}
