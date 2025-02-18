use bevy::prelude::*;

pub struct FollowCameraPlugin;

impl Plugin for FollowCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedUpdate, follow_camera);
    }
}

#[derive(Component)]
#[require(Camera3d)]
pub struct FollowCamera(pub Entity, pub Vec3);

pub fn follow_camera(
    mut cameras: Query<(&mut Transform, &FollowCamera)>,
    targets: Query<&Transform, Without<FollowCamera>>,
) {
    for (mut cam_trans, FollowCamera(target_entity, offset)) in &mut cameras {
        if let Ok(target) = targets.get(*target_entity) {
            let target_pos = target.translation + target.rotation * (*offset);
            *cam_trans = cam_trans
                .with_translation(cam_trans.translation.lerp(target_pos, 0.1))
                .looking_at(target.translation, Dir3::Y);
        }
    }
}
