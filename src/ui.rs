use bevy::{
    color::palettes::css::GOLD,
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    prelude::*,
};

pub struct UIPlugin;

impl Plugin for UIPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((FrameTimeDiagnosticsPlugin,))
            .add_systems(Startup, setup)
            .add_systems(Update, (text_update_system,));
    }
}
#[derive(Component)]
struct FpsText;

fn setup(mut commands: Commands) {
    // Text with multiple sections
    commands
        .spawn((
            // Create a Text with multiple child spans.
            Text::new("FPS: "),
            TextFont {
                font_size: 20.0,
                ..default()
            },
        ))
        .with_child((
            TextSpan::default(),
            TextFont {
                font_size: 20.0,
                ..default()
            },
            TextColor(GOLD.into()),
            FpsText,
        ));
}

fn text_update_system(
    diagnostics: Res<DiagnosticsStore>,
    mut query: Query<&mut TextSpan, With<FpsText>>,
) {
    for mut span in &mut query {
        if let Some(fps) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(value) = fps.smoothed() {
                // Update the value of the second section
                **span = format!("{value:.2}");
            }
        }
    }
}
