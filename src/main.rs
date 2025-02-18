use avian3d::{math::PI, prelude::*};
use bevy::{
    color::palettes::css::{BLACK, GRAY},
    prelude::*,
};
use bevy_inspector_egui::quick::WorldInspectorPlugin;

use car::CarPlugin;
use follow_cam::FollowCameraPlugin;

mod car;
mod follow_cam;
mod orbit_camera;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, CarPlugin, FollowCameraPlugin))
        .add_plugins((
            PhysicsPlugins::default(),
            // PhysicsDebugPlugin::default(),
            WorldInspectorPlugin::new(),
        ))
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

    // Sunlight
    commands.spawn((
        DirectionalLight {
            illuminance: light_consts::lux::OVERCAST_DAY,
            shadows_enabled: true,
            ..default()
        },
        Transform {
            translation: Vec3::new(0.0, 2.0, 0.0),
            rotation: Quat::from_rotation_x(-PI / 4.),
            ..default()
        },
    ));
}

fn gizmos(mut gizmos: Gizmos) {
    gizmos.axes(Transform::IDENTITY, 10.0);
    gizmos.grid(
        Isometry3d::from_rotation(Quat::from_rotation_x(PI / 2.0)),
        UVec2::new(20, 20),
        Vec2::splat(10.0),
        BLACK.with_alpha(0.5),
    );
}
