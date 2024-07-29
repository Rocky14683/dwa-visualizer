#[derive(Copy, Clone)]
pub struct Point {
    pub x: f64,
    pub y: f64
}

impl Point {
    pub fn get_distance(&self, other: Point) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    pub fn get_angle(&self, other: Point) -> f64 {
        (other.y - self.y).atan2(other.x - self.x)
    }

    pub fn to_vector2(&self) -> raylib::math::Vector2 {
        raylib::math::Vector2 { x: self.x as f32, y: self.y as f32 }
    }
}




#[derive(Copy, Clone)]
pub struct Pose {
    pub position: Point,
    pub orientation: f64
}

impl Pose {

    pub fn get_distance(&self, other: Pose) -> f64 {
        self.position.get_distance(other.position)
    }

    pub fn get_angle(&self, other: Pose) -> f64 {
        self.position.get_angle(other.position)
    }

    pub fn to_point(&self) -> Point {
        self.position
    }
}
