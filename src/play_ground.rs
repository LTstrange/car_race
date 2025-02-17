use bevy::{math::VectorSpace, prelude::*};
// use bevy_rapier3d::prelude::*;
use avian3d::prelude::*;

pub struct PlayGroundPlugin;

impl Plugin for PlayGroundPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup);
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let parent = commands
        .spawn((
            Name::new("Cube".to_string()),
            RigidBody::Kinematic,
            Collider::cuboid(0.6, 0.25, 1.465),
            Transform::from_xyz(0.0, 2.0, 0.0),
            AngularVelocity(Vec3::new(0.0, 4.0, 0.0)),
        ))
        .id();

    let ray = commands
        .spawn((
            RayCaster::new(Vec3::ZERO, Dir3::NEG_Y)
                .with_max_distance(0.8)
                .with_max_hits(1)
                .with_query_filter(SpatialQueryFilter::from_excluded_entities([parent])),
            Transform::from_xyz(0.6, 0.0, 0.884),
        ))
        .id();

    commands.entity(parent).add_children(&[ray]);
}
