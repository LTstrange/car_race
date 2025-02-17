use avian3d::{math::PI, prelude::*};
use bevy::{color::palettes::css::GRAY, prelude::*};

use car::CarPlugin;

mod camera;
mod car;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, CarPlugin))
        .add_plugins((PhysicsPlugins::default(), PhysicsDebugPlugin::default()))
        .add_systems(Startup, setup)
        .add_systems(Update, gizmos)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Static physics object with a collision shape
    commands.spawn((
        Name::new("Ground".to_string()),
        RigidBody::Static,
        Collider::cylinder(200.0, 0.1),
        Mesh3d(meshes.add(Cylinder::new(200.0, 0.1))),
        MeshMaterial3d(materials.add(Color::WHITE)),
        Transform::from_xyz(0.0, -0.05, 0.0),
    ));

    commands.spawn((
        Name::new("Rod".to_string()),
        RigidBody::Static,
        Collider::cylinder(0.1, 4.0),
        Mesh3d(meshes.add(Cylinder::new(0.1, 4.0))),
        MeshMaterial3d(materials.add(Color::WHITE)),
        Transform::from_xyz(0.0, 0.0, 2.0).with_rotation(Quat::from_rotation_z(PI / 2.0)),
    ));

    commands.spawn((
        Name::new("Rod".to_string()),
        RigidBody::Static,
        Collider::cylinder(0.1, 4.0),
        Mesh3d(meshes.add(Cylinder::new(0.1, 4.0))),
        MeshMaterial3d(materials.add(Color::WHITE)),
        Transform::from_xyz(0.0, 0.0, -2.0).with_rotation(Quat::from_rotation_z(PI / 2.0)),
    ));

    // Light
    commands.spawn((
        Name::new("Light".to_string()),
        PointLight {
            shadows_enabled: true,
            ..Default::default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));
}

fn gizmos(mut gizmos: Gizmos) {
    gizmos.axes(Transform::IDENTITY, 10.0);
    gizmos.grid(
        Isometry3d::from_rotation(Quat::from_rotation_x(PI / 2.0)),
        UVec2::new(200, 200),
        Vec2::splat(1.0),
        GRAY.with_alpha(0.5),
    );
}
