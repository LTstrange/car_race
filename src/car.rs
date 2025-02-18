use avian3d::{math::PI, prelude::*};
use bevy::{input::common_conditions::input_just_pressed, prelude::*};

use crate::follow_cam::FollowCamera;

pub struct CarPlugin;

impl Plugin for CarPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup).add_systems(
            FixedUpdate,
            (
                (suspension, steering, acceleration).chain(),
                turn_system,
                reset_car.run_if(input_just_pressed(KeyCode::Space)),
            ),
        );
        // app.add_systems(Update, gizmos);
    }
}

struct CarConfig {
    car_mesh_handle: Handle<Mesh>,
    car_mat_handle: Handle<StandardMaterial>,
    front_wheel_drive: bool,
    suspension: SuspensionConfig,
    steering: SteeringConfig,
    acceleration: AccelerationConfig,
}

impl CarConfig {
    fn new(
        mesh_handle: Handle<Mesh>,
        mat_handle: Handle<StandardMaterial>,
        suspension: SuspensionConfig,
        steering: SteeringConfig,
        acceleration: AccelerationConfig,
    ) -> Self {
        Self {
            car_mesh_handle: mesh_handle,
            car_mat_handle: mat_handle,
            front_wheel_drive: true,
            suspension,
            steering,
            acceleration,
        }
    }

    #[allow(unused)]
    fn with_front_wheel_drive(mut self) -> Self {
        self.front_wheel_drive = true;
        self
    }

    #[allow(unused)]
    fn with_rear_wheel_drive(mut self) -> Self {
        self.front_wheel_drive = false;
        self
    }

    fn construct(self, commands: &mut Commands) -> Entity {
        let rest_dist = self.suspension.rest_dist;
        let buffer = self.suspension.buffer;
        let parent = commands
            .spawn((
                Name::new("Car".to_string()),
                Car,
                self.suspension,
                self.steering,
                self.acceleration,
                Mesh3d(self.car_mesh_handle),
                MeshMaterial3d(self.car_mat_handle),
                TransformInterpolation,
                RigidBody::Dynamic,
                Collider::cuboid(1.2, 0.5, 2.93),
                ColliderDensity(5.0),
                ExternalForce::ZERO.with_persistence(false),
            ))
            .id();

        let y_offset = -0.1;

        let fr_t = Tire.construct(
            commands,
            Vec3::new(0.6, y_offset, -0.884),
            rest_dist + buffer,
            parent,
            true,
            self.front_wheel_drive,
        );
        let fl_t = Tire.construct(
            commands,
            Vec3::new(-0.6, y_offset, -0.884),
            rest_dist + buffer,
            parent,
            true,
            self.front_wheel_drive,
        );
        let rr_t = Tire.construct(
            commands,
            Vec3::new(0.6, y_offset, 0.884),
            rest_dist + buffer,
            parent,
            false,
            !self.front_wheel_drive,
        );
        let rl_t = Tire.construct(
            commands,
            Vec3::new(-0.6, y_offset, 0.884),
            rest_dist + buffer,
            parent,
            false,
            !self.front_wheel_drive,
        );

        commands
            .entity(parent)
            .add_children(&[fr_t, fl_t, rr_t, rl_t]);

        parent
    }
}

#[derive(Component)]
struct SuspensionConfig {
    rest_dist: f32,
    spring_strength: f32,
    damper: f32,
    buffer: f32,
}

#[derive(Component)]
struct SteeringConfig {
    tire_grip_factor: f32,
}

#[derive(Component)]
struct AccelerationConfig {
    forward_force: f32,
    backward_force: f32,
    friction: f32,
}

#[derive(Component)]
struct Car;

#[derive(Component)]
struct Tire;

impl Tire {
    fn construct(
        self,
        commands: &mut Commands,
        origin: Vec3,
        ray_dist: f32,
        parent: Entity,
        is_front_tire: bool,
        is_drive_wheel: bool,
    ) -> Entity {
        let mut c = commands.spawn((
            Tire,
            Name::new("Tire".to_string()),
            Transform::from_translation(origin),
            RayCaster::new(Vec3::ZERO, Dir3::NEG_Y)
                .with_max_distance(ray_dist)
                .with_max_hits(1)
                .with_query_filter(SpatialQueryFilter::from_excluded_entities([parent])),
        ));

        if is_front_tire {
            c.insert(FrontTire);
        }

        if is_drive_wheel {
            c.insert(DriveWheel);
        }

        c.id()
    }
}

#[derive(Component)]
struct FrontTire;
#[derive(Component)]
struct DriveWheel;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Dynamic physics object with a collision shape and initial angular velocity
    let mesh_h = meshes.add(Cuboid::from_size(Vec3::new(1.2, 0.5, 2.93)));
    let mat_h = materials.add(Color::srgba_u8(124, 144, 255, 200));

    let suspension = SuspensionConfig {
        rest_dist: 0.6,
        spring_strength: 100.0,
        damper: 25.0,
        buffer: 0.1,
    };
    let steering = SteeringConfig {
        tire_grip_factor: 0.3,
    };

    let acceleration = AccelerationConfig {
        forward_force: 50.0,
        backward_force: -50.0,
        friction: 0.1,
    };

    let car_config =
        CarConfig::new(mesh_h, mat_h, suspension, steering, acceleration).with_front_wheel_drive();

    let car = car_config.construct(&mut commands);

    commands.spawn(FollowCamera(car, Vec3::new(0.0, 3.0, 9.0)));
}

// const GIZMOS_FORCE_FACTOER: f32 = 0.1;

fn get_point_velocity(point: Vec3, linear_vel: Vec3, angular_vel: Vec3, center: Vec3) -> Vec3 {
    linear_vel + angular_vel.cross(point - center)
}

fn suspension(
    mut cars: Query<
        (
            &Transform,
            &LinearVelocity,
            &AngularVelocity,
            &mut ExternalForce,
            &SuspensionConfig,
            &Children,
        ),
        With<Car>,
    >,
    rays: Query<(&RayCaster, &RayHits)>,
    // mut gizmos: Gizmos,
) {
    for (car_trans, l_vel, a_vel, mut eforce, config, children) in &mut cars {
        let center_mass = car_trans.translation;
        let up_dir = car_trans.up();
        for child in children.iter() {
            // find ray_casters
            if let Ok((ray, hits)) = rays.get(*child) {
                // max hit = 1, so we can use iter().next()
                if let Some(hit) = hits.iter().next() {
                    let tire_global_vel =
                        get_point_velocity(ray.global_origin(), l_vel.0, a_vel.0, center_mass);

                    let offset = config.rest_dist - hit.distance;

                    let up_vel = up_dir.dot(tire_global_vel);

                    let up_force = (config.spring_strength * offset) - (config.damper * up_vel);

                    // gizmos.arrow(
                    //     ray.global_origin(),
                    //     ray.global_origin() + up_dir * up_force * GIZMOS_FORCE_FACTOER,
                    //     GREEN,
                    // );

                    eforce.apply_force_at_point(
                        up_dir * up_force,
                        ray.global_origin(),
                        center_mass,
                    );
                }
            }
        }
    }
}

fn steering(
    mut cars: Query<
        (
            &Transform,
            &LinearVelocity,
            &AngularVelocity,
            &mut ExternalForce,
            &SteeringConfig,
            &Children,
        ),
        With<Car>,
    >,
    rays: Query<(&Transform, &RayCaster, &RayHits)>,
    fixed_time: Res<Time<Fixed>>,
    // mut gizmos: Gizmos,
) {
    for (car_trans, l_vel, a_vel, mut eforce, config, children) in &mut cars {
        let center_mass = car_trans.translation;
        for child in children.iter() {
            // find ray_casters
            if let Ok((ray_trans, ray, hits)) = rays.get(*child) {
                // max hit = 1, so we can use iter().next()
                if let Some(_) = hits.iter().next() {
                    let right_dir = car_trans.rotation * ray_trans.right();
                    let tire_global_vel =
                        get_point_velocity(ray.global_origin(), l_vel.0, a_vel.0, center_mass);

                    let steering_vel = right_dir.dot(tire_global_vel);

                    let desired_vel_change = -steering_vel * config.tire_grip_factor;
                    let desired_acc = desired_vel_change / fixed_time.delta_secs();

                    // gizmos.arrow(
                    //     ray.global_origin(),
                    //     ray.global_origin() + right_dir * desired_acc * GIZMOS_FORCE_FACTOER,
                    //     RED,
                    // );

                    eforce.apply_force_at_point(
                        right_dir * desired_acc,
                        ray.global_origin(),
                        center_mass,
                    );
                }
            }
        }
    }
}

fn acceleration(
    mut cars: Query<
        (
            &Transform,
            &LinearVelocity,
            &AngularVelocity,
            &mut ExternalForce,
            &AccelerationConfig,
            &Children,
        ),
        With<Car>,
    >,
    rays: Query<(&Transform, &RayCaster, &RayHits), With<DriveWheel>>,
    fixed_time: Res<Time<Fixed>>,
    keys: Res<ButtonInput<KeyCode>>,
    // mut gizmos: Gizmos,
) {
    for (car_trans, l_vel, a_vel, mut eforce, config, children) in &mut cars {
        let center_mass = car_trans.translation;
        for child in children.iter() {
            if let Ok((ray_trans, ray, hits)) = rays.get(*child) {
                if let Some(_) = hits.iter_sorted().next() {
                    let tire_global_vel =
                        get_point_velocity(ray.global_origin(), l_vel.0, a_vel.0, center_mass);

                    let forward_dir = car_trans.rotation * ray_trans.forward();

                    let forward_force = if keys.pressed(KeyCode::KeyW) {
                        config.forward_force
                        // forward
                    } else if keys.pressed(KeyCode::KeyS) {
                        // backward
                        config.backward_force
                    } else {
                        let forward_vel = forward_dir.dot(tire_global_vel);
                        let desired_vel_change = -forward_vel * config.friction;
                        let desired_acc = desired_vel_change / fixed_time.delta_secs();
                        desired_acc.clamp(-5.0, 5.0)
                    };

                    // gizmos.arrow(
                    //     ray.global_origin(),
                    //     ray.global_origin() + forward_dir * forward_force * GIZMOS_FORCE_FACTOER,
                    //     BLUE,
                    // );

                    eforce.apply_force_at_point(
                        forward_dir * forward_force,
                        ray.global_origin(),
                        center_mass,
                    );
                }
            }
        }
    }
}

const TURN_SPEED: f32 = PI / 3.0;
const MAX_TURN_ANGLE: f32 = PI / 6.0;
fn turn_system(
    mut cars: Query<&Children, With<Car>>,
    mut rays: Query<&mut Transform, (With<RayCaster>, With<FrontTire>)>,
    time: Res<Time<Fixed>>,
    keys: Res<ButtonInput<KeyCode>>,
) {
    let dt = time.delta_secs();

    for children in &mut cars {
        for child in children.iter() {
            if let Ok(mut tire_trans) = rays.get_mut(*child) {
                let rot = if keys.pressed(KeyCode::KeyA) {
                    Quat::from_rotation_y(MAX_TURN_ANGLE)
                } else if keys.pressed(KeyCode::KeyD) {
                    Quat::from_rotation_y(-MAX_TURN_ANGLE)
                } else {
                    Quat::IDENTITY
                };
                tire_trans.rotation = tire_trans.rotation.rotate_towards(rot, TURN_SPEED * dt);
            }
        }
    }
}

fn reset_car(
    mut cars: Query<
        (
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &mut ExternalForce,
        ),
        With<Car>,
    >,
) {
    for (mut trans, mut l_vel, mut a_vel, mut eforce) in &mut cars {
        *trans = Transform::from_xyz(0.0, 4.0, 0.0).with_rotation(Quat::from_rotation_y(PI / 6.0));
        eforce.clear();
        l_vel.0 = Vec3::ZERO;
        a_vel.0 = Vec3::ZERO;
    }
}
