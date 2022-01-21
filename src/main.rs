#![allow(non_snake_case)]

mod rigidbody;
mod collision;
mod math;
mod debug;

use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
    math::{Mat3, Vec3},
};
use bevy::math::{dquat, mat3, quat, vec3};
use crate::collision::{Collider, CollisionStore};
use crate::rigidbody::{PhysicsWorld, RigidBody};


pub fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        //.add_plugin(FrameTimeDiagnosticsPlugin::default())
        //.add_plugin(LogDiagnosticsPlugin::default())
        .add_startup_system(setup)
        .add_system(rigidbody::update)
        .add_system(collision::collide)
        .add_system(debug_collision)
        .run();
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {

    let w = 1.0;
    let h = 2.0;
    let d = 1.0;

    let mesh = meshes.add(Mesh::from(shape::Box {
        min_x: -0.5 * w,
        max_x: 0.5 * w,
        min_y: -0.5 * h,
        max_y: 0.5 * h,
        min_z: -0.5 * d,
        max_z: 0.5 * d,
    }));

    let material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        metallic: 0.0,
        reflectance: 0.0,
        ..Default::default()
    });
  
    commands.spawn_bundle(PbrBundle {
        mesh: mesh.clone(),
        material: material.clone(),
        transform: Transform::from_xyz(-0.7, 0.0, 0.0),
        ..Default::default()
    }).insert(Collider::new_box(vec3(w, h, d)));

    commands.spawn_bundle(PbrBundle {
        mesh: mesh.clone(),
        material: material.clone(),
        transform: Transform::from_xyz(0.7, 0.0, 0.0),
        ..Default::default()
    }).insert(Collider::new_box(vec3(w, h, d))).insert(RigidBody {
        angular_velocity: vec3(0.0,0.0,0.5),
        ..Default::default()
    });

    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_xyz(0.0, 10.0, 10.0).looking_at(Vec3::ZERO, Vec3::Z),
        ..Default::default()
    });

    commands.spawn_bundle(DirectionalLightBundle {
        transform: Transform::from_rotation(Quat::from_axis_angle(Vec3::Z, 0.1)),
        directional_light: DirectionalLight {
            color: Color::WHITE,
            .. Default::default()
        },
        .. Default::default()
    });

    commands.insert_resource(PhysicsWorld{
        g: Vec3::ZERO,
        dt: 0.02,
        .. Default::default()
    });

    commands.insert_resource(CollisionStore::new());
}

fn debug_collision(mut materials: ResMut<Assets<StandardMaterial>>, collision_store: Res<CollisionStore>, query: Query<(Entity, &Collider, &Handle<StandardMaterial>)>) {
    for (e, _, m) in query.iter() {
        if collision_store.has_collision(e) {
            materials.get_mut(m).unwrap().base_color = Color::RED;
        } else {
            materials.get_mut(m).unwrap().base_color = Color::WHITE;
        }
    }
}

