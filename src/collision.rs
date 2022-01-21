use std::cmp::min;
use std::collections::HashSet;
use std::fmt::{Debug, Formatter};
use std::iter::once;

use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    math::{Mat3, Vec3},
    prelude::*,
};
use bevy::math::{const_vec3, dquat, mat3, quat, vec3};
use bevy::utils::HashMap;

use crate::math::{AABB, cwise_max, cwise_min, cwise_mul, Interval};

#[derive(Clone, Copy, Debug)]
pub enum ColliderType {
    Box{extents: Vec3}
}

const BOX_VERTICES: &'static [Vec3] = &[const_vec3!([-1.0, -1.0, -1.0]), const_vec3!([-1.0, -1.0, 1.0]), const_vec3!([-1.0, 1.0, -1.0]), const_vec3!([-1.0, 1.0, 1.0]),
    const_vec3!([1.0, -1.0, -1.0]), const_vec3!([1.0, -1.0, 1.0]), const_vec3!([1.0, 1.0, -1.0]), const_vec3!([1.0, 1.0, 1.0])];

#[derive(Component, Copy, Clone, Debug)]
pub struct Collider {
    ty: ColliderType,
}

impl Collider {
    pub fn I_b(&self) -> Mat3 {
        match self.ty {
            ColliderType::Box{extents: v} => Mat3::from_diagonal(vec3(v.y * v.y + v.z * v.z, v.x * v.x + v.z * v.z, v.x * v.x + v.y * v.y) / 12.0),
            _ => Mat3::ZERO
        }
    }

    pub fn new_box(extents: Vec3) -> Self {
        Self { ty: ColliderType::Box{extents} }
    }

    pub fn with_transform<'c, 'gt>(&'c self, transform: &'gt GlobalTransform) -> TransformedCollider<'c, 'gt> {
        TransformedCollider::new(self, transform)
    }
}


#[derive(Clone, Copy)]
pub struct TransformedCollider<'c, 'gt> {
    collider: &'c Collider,
    transform: &'gt GlobalTransform,
}

impl<'c, 'gt> TransformedCollider<'c, 'gt> {
    fn new(collider: &'c Collider, transform: &'gt GlobalTransform) -> Self {
        Self {
            collider,
            transform
        }
    }

    pub fn aabb(&self) -> AABB {
        match self.collider.ty {
            ColliderType::Box{extents} => {
                AABB::fit_vertices(BOX_VERTICES.iter().map(|v| self.transform.mul_vec3(0.5 * cwise_mul(extents, *v))))
            }
        }
    }

    pub fn collide(self, other: Self) -> Option<CollisionData> {
        match (self.collider.ty, other.collider.ty) {
            (ColliderType::Box{extents: self_extents}, ColliderType::Box{extents: other_extents}) => {

                let self_axes = [self.transform.right(), self.transform.up(), self.transform.forward()];
                let other_axes = [other.transform.right(), other.transform.up(), other.transform.forward()];

                let mut axes = vec!();

                axes.extend(self_axes);
                axes.extend(other_axes);

                for i in 0..3 {
                    for j in i+1..3 {
                        axes.push(Vec3::cross(self_axes[i], self_axes[j]));
                    }
                }

                let mut self_vertices = [Vec3::ZERO; 6];
                let mut other_vertices = [Vec3::ZERO; 6];

                for i in 0..6 {
                    self_vertices[i] = self.transform.mul_vec3(0.5 * cwise_mul(self_extents, BOX_VERTICES[i]));
                    other_vertices[i] = self.transform.mul_vec3(0.5 * cwise_mul(other_extents, BOX_VERTICES[i]));
                }

                let mut normal = Vec3::ZERO;
                let mut minimal_intersection : Option<Interval> = None;

                for axis in axes {

                    if axis.length() < 1e-6 {
                        continue;
                    }

                    let self_interval = Interval::vertex_projection(&self_vertices[..], axis);
                    let other_interval = Interval::vertex_projection(&other_vertices[..], axis);

                    if let Some(intersection) = self_interval.intersection(other_interval) {
                        if minimal_intersection.is_none() || minimal_intersection.unwrap().length() > intersection.length() {
                            minimal_intersection = Some(intersection);
                            normal = axis;
                        }
                    } else {
                        minimal_intersection = None;
                        break;
                    }
                }

                minimal_intersection.map(|i| CollisionData {
                    normal,
                    depth: i.length(),
                })
            },
            _ => todo!()
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CollisionData {
    pub normal: Vec3,
    pub depth: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Collision {
    entity_a: Entity,
    entity_b: Entity,
}

#[derive(Debug)]
pub struct CollisionStore {
    collisions: Vec<Collision>,
}

impl CollisionStore {
    pub fn add_collision(&mut self, entity_a: Entity, entity_b: Entity, data: CollisionData) {
        self.collisions.push(Collision {
            entity_a,
            entity_b
        });
    }

    pub fn clear(&mut self) {
        self.collisions.clear();
    }

    pub fn new() -> Self {
        Self {
            collisions: vec!(),
        }
    }

    pub fn has_collision(&self, entity: Entity) -> bool{
        self.collisions.iter().any(|coll| coll.entity_a == entity || coll.entity_b == entity)
    }
}

type ColliderTriple<'a> = (Entity, &'a Collider, &'a GlobalTransform);

pub fn collide(mut collision_store: ResMut<CollisionStore>, mut query: Query<ColliderTriple>) {
    collision_store.clear();

    let (aabbs, q) : (Vec<_>, Vec<_>) = query.iter().map(|(e, c, t)| (c.with_transform(t).aabb(), (e, c, t))).unzip();

    let n = aabbs.len();

    let mut per_axis_collisions = (0..3).map(|_|HashSet::new()).collect::<Vec<_>>();

    for d in 0..3 {
        let mut axis: Vec<_> = (0..aabbs.len()).collect();

        axis.sort_by(|&i, &j|
            aabbs[i].min[d].partial_cmp(&aabbs[j].min[d]).unwrap()
        );

        for i in 0..n - 1 {
            let mut j = i + 1;

            while j < n && aabbs[axis[j]].min[d] <= aabbs[axis[i]].max[d] {
                per_axis_collisions[d].insert((usize::min(axis[i], axis[j]), usize::max(axis[i], axis[j])));
                j += 1;
            }
        }
    }

    let collisions = per_axis_collisions
        .iter()
        .fold(per_axis_collisions[0].clone(), |ref s1,s2| s1 & s2);

    for &(i, j) in &collisions {
        let (e1, c1, t1) = q[i];
        let (e2, c2, t2) = q[j];

        if let Some(data) = c1.with_transform(t1).collide(c2.with_transform(t2)) {
            collision_store.add_collision(e1, e2, data);
        }
    }
}

