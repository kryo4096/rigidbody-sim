use std::fmt::{Debug, Display, Formatter};
use std::ops::Deref;
use bevy::math::{dquat, mat3, Mat3, quat, Quat, vec3};
use bevy::prelude::Vec3;
use crate::GlobalTransform;

pub fn skew(w: Vec3) -> Mat3 {
    mat3(vec3(0.0, w.z, -w.y), vec3(-w.z, 0.0, w.x), vec3(w.y, -w.x, 0.0))
}

pub fn qmul(q1: Quat, q2: Quat) -> Quat {
    let r1 = q1.w;
    let r2 = q2.w;
    let v1 = Vec3::new(q1.x, q1.y, q1.z);
    let v2 = Vec3::new(q2.x, q2.y, q2.z);

    let r = r1 * r2 - Vec3::dot(v1, v2);
    let v = r1 * v2 + r2 * v1 + Vec3::cross(v1, v2);

    quat(v.x, v.y, v.z, r)
}

pub fn cwise_max(v1: Vec3, v2: Vec3) -> Vec3 {
    vec3(v1.x.max(v2.x), v1.y.max(v2.y), v1.z.max(v2.z))
}

pub fn cwise_min(v1: Vec3, v2: Vec3) -> Vec3 {
    vec3(v1.x.min(v2.x), v1.y.min(v2.y), v1.z.min(v2.z))
}

pub fn cwise_mul(v1: Vec3, v2: Vec3) -> Vec3 {
    vec3(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z)
}

#[derive(Copy, Clone)]
pub struct Interval {
    min: f32,
    max: f32,
}
impl Interval {
    pub fn new(min: f32, max: f32) -> Self {
        assert!(max >= min);
        Self {
            min, max
        }
    }

    pub fn vertex_projection(vertices: &[Vec3], axis: Vec3) -> Interval {
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;

        for &vertex in vertices {
            let dp = Vec3::dot(vertex, axis);

            min = f32::min(dp, min);
            max = f32::max(dp, max)
        }

        Interval::new(min, max)
    }

    pub fn length(&self) -> f32{
        self.max - self.min
    }

    pub fn intersection(self, other: Interval) -> Option<Interval> {
        let i = Interval {
            min: f32::max(self.min, other.min),
            max: f32::min(self.max, other.max)
        };

        if i.length() < 0.0 {
            None
        } else {
            Some(i)
        }
    }

    pub fn min(&self) -> f32 {
        self.min
    }

    pub fn max(&self) -> f32 {
        self.max
    }
}

impl Display for Interval {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}; {}]", self.min, self.max)
    }
}

impl Debug for Interval {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "Interval[{}; {}]", self.min, self.max)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AABB {
    pub min: Vec3,
    pub max: Vec3,
}

impl AABB {
    pub fn fit_vertices(vertices: impl Iterator<Item=Vec3> + Clone) -> AABB {
        let mut min = vec3(f32::INFINITY, f32::INFINITY, f32::INFINITY);
        let mut max = -min;

        for vertex in vertices {
            min = cwise_min(min, vertex);
            max = cwise_max(max, vertex);
        }

        AABB {
            min,
            max
        }
    }

    pub fn from_intervals(interval_x: Interval, interval_y: Interval, interval_z: Interval) -> Self {
        AABB {
            min: vec3(interval_x.min, interval_y.min, interval_z.min),
            max: vec3(interval_x.max, interval_y.max, interval_z.max)
        }
    }

    pub fn axis_interval(&self, dimension: usize) -> Interval {
        Interval::new(self.min[dimension], self.max[dimension])
    }
}


#[cfg(test)]
mod tests {
    use crate::math::{Interval, qmul};
    use crate::{quat, vec3};

    #[test]
    fn it_works() {
        let quat1 = quat(1.0,1.0,1.0,1.0);
        assert_eq!(qmul(quat1, quat1.conjugate()), quat(0.0, 0.0, 0.0, 4.0));
    }

    #[test]
    fn test_quat_mul() {
        let q = quat(1.0, 1.0, 1.0, 1.0);
        assert_eq!(q * 2.0, quat(2.0,2.0,2.0, 2.0));
    }
    #[test]
    fn test_vec3() {
        let a = vec3(0.0,1.0,0.0);

        assert_eq!(a[1], 1.0);
    }

    #[test]
    fn test_intersection() {
        let i_a = Interval::new(0.0, 1.0);
        let i_b = Interval::new(-1.0, 2.0);

        let i_ab = Interval::intersection(i_a, i_b);

        dbg!(i_ab);
    }
}

