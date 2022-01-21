use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
    math::{Mat3, Vec3},
};
use bevy::math::{dquat, mat3, quat, vec3};
use crate::collision::Collider;

use crate::math;
use crate::math::qmul;

#[derive(Component)]
pub struct RigidBody {
    pub mass: f32,
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    pub force: Vec3,
    pub torque: Vec3,
}

impl RigidBody {
    pub fn add_force(&mut self, offset: Vec3, force: Vec3) {
        self.force += force;
        self.torque += Vec3::cross(offset, force);
    }
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            mass: 1.0,
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
        }
    }
}

#[derive(Default)]
pub struct PhysicsWorld {
    pub dt: f32,
    pub g: Vec3,
    pub t: f32,
}

pub(crate) fn update(mut phys_world: ResMut<PhysicsWorld>, mut query: Query<(&mut RigidBody, &Collider, &mut GlobalTransform)>) {
    for (mut rb, coll, mut t) in query.iter_mut() {
        let a = rb.force / rb.mass + phys_world.g;

        rb.velocity += a * phys_world.dt;

        let mut w = rb.angular_velocity;

        let I_b = coll.I_b();
        let I_b_inv = I_b.inverse();

        let R = Mat3::from_quat(t.rotation);
        let I = R * I_b * R.inverse();
        let I_inv = R * I_b.inverse() * R.inverse();


        let wb = R.inverse() * w;
        let f = Vec3::cross(wb, I_b * wb) * phys_world.dt;
        let J = I_b + phys_world.dt * (math::skew(wb) * I_b - math::skew(I_b * wb));
        let dwb = J.inverse() * f;
        w = R * (wb - dwb);

        // explicit  w -= I * Vec3::cross(w, I * w) * phys_world.dt;

        w += I_inv * rb.torque * phys_world.dt;

        rb.angular_velocity = w;
        let d_rot = qmul(quat(w.x, w.y, w.z, 0.0), t.rotation) * 0.5 * phys_world.dt;
        t.rotation = (t.rotation + d_rot).normalize();
        t.translation += rb.velocity * phys_world.dt;

        rb.force = Vec3::ZERO;
        rb.torque = Vec3::ZERO;
    }
    phys_world.t += phys_world.dt;
}