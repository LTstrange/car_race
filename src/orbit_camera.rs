use bevy::input::gestures::PinchGesture;
use bevy::input::mouse::MouseWheel;
use bevy::prelude::*;

const SCALE_FACTOR: f32 = 0.8;
const DRAG_FACTOR: f32 = 0.005;

pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup)
            .add_systems(Update, orbit_camera_system);
    }
}

#[derive(Component)]
struct OrbitCamera {
    radius: f32,
    /// 方位角（绕y轴的旋转）
    azimuthal_angle: f32,
    /// 极角（与赤道夹角）
    polar_angle: f32,
}

fn setup(mut commands: Commands) {
    // Camera
    commands.spawn((
        Name::new("Camera".to_string()),
        Camera3d::default(),
        OrbitCamera {
            radius: 20.0,
            azimuthal_angle: 0.0,
            polar_angle: std::f32::consts::PI / 4.0,
        },
    ));
}

fn orbit_camera_system(
    mut pinch_events: EventReader<PinchGesture>,
    mut mouse_wheel_events: EventReader<MouseWheel>,
    mut query: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    let (mut orbit_camera, mut transform) = query.single_mut();

    // z-axis rotation
    for event in mouse_wheel_events.read() {
        orbit_camera.azimuthal_angle -= event.x * DRAG_FACTOR;
        orbit_camera.polar_angle += event.y * DRAG_FACTOR;

        // 限制极角在合理范围内，防止翻转
        orbit_camera.polar_angle = orbit_camera.polar_angle.clamp(
            -std::f32::consts::PI / 2.0 + 0.01,
            std::f32::consts::PI / 2.0 - 0.01,
        );
    }

    for event in pinch_events.read() {
        orbit_camera.radius *= 1. - SCALE_FACTOR * event.0;

        orbit_camera.radius = orbit_camera.radius.clamp(1.0, 50.0); // 限制相机的距离在合理范围内，防止相机超出地球表面
    }

    // 计算相机的新位置
    let x =
        orbit_camera.radius * orbit_camera.polar_angle.cos() * orbit_camera.azimuthal_angle.cos();
    let y = orbit_camera.radius * orbit_camera.polar_angle.sin();
    let z =
        -orbit_camera.radius * orbit_camera.polar_angle.cos() * orbit_camera.azimuthal_angle.sin();

    // 设置相机的位置，并使其始终朝向地球（即原点）
    transform.translation = Vec3::new(x, y, z);
    transform.look_at(Vec3::ZERO, Vec3::Y);
}
