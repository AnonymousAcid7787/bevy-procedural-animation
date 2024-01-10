use bevy::{
    prelude::{Resource, Res, ResMut, FrameCountPlugin, Update}, 
    time::TimePlugin, 
    log::LogPlugin, 
    diagnostic::{LogDiagnosticsPlugin, FrameTimeDiagnosticsPlugin}, math::Vec3
};
use bevy_rapier3d::na::Vector3;

/// A plugin that logs fps 
#[derive(Default)]
pub struct LogFramesPlugin {
    /// Setting this to true makes this plugin use bevy's frame diagnostic plugins instead.
    pub use_bevy_fps_logger: bool
}

impl bevy::prelude::Plugin for LogFramesPlugin {
    fn build(&self, app: &mut bevy::prelude::App) {

        //if bevy fps logger is requested, use bevy frame diagnostics and return
        if self.use_bevy_fps_logger {
            if !app.is_plugin_added::<TimePlugin>() {
                app.add_plugins(TimePlugin::default());
            }
            if !app.is_plugin_added::<LogPlugin>() {
                app.add_plugins(LogPlugin::default());
            }
            if !app.is_plugin_added::<FrameCountPlugin>() {
                app.add_plugins(FrameCountPlugin::default());
            }
            if !app.is_plugin_added::<LogDiagnosticsPlugin>() {
                app.add_plugins(LogDiagnosticsPlugin::default());
            }
            if !app.is_plugin_added::<FrameTimeDiagnosticsPlugin>() {
                app.add_plugins(FrameTimeDiagnosticsPlugin::default());
            }
            return;
        }

        //else, use the plugin's own frame diagnostic implementation

        app
            .insert_resource(FrameDiagnostics::default())
            .add_systems(Update, frame_diagnostics_system);

        if !app.is_plugin_added::<TimePlugin>() { app.add_plugins(TimePlugin::default()); }
        
    }
}

#[derive(Resource, Default)]
struct FrameDiagnostics {
    /// The amount of time passed since the last log. When this reaches 1 second, the data will be logged and a new diagnostic cycle will occur.
    pub time_passed: f32,
    // Amount of frames since last log
    pub frames: u32,
}

fn frame_diagnostics_system (
    time: Res<bevy::time::Time>,
    mut diagnostics: ResMut<FrameDiagnostics>,
) {
    diagnostics.frames += 1;
    diagnostics.time_passed += time.delta_seconds();

    if diagnostics.time_passed >= 1. {
        println!("----------------------------------");
        println!("fps {}", diagnostics.frames);
        println!(
            "frame time {0}ms, (avg {1}ms)",
            time.delta_seconds() * 1000.,
            (diagnostics.time_passed / (diagnostics.frames as f32)) * 1000.
        );
        println!("----------------------------------");

        diagnostics.time_passed = 0.;
        diagnostics.frames = 0;
    }
}


pub unsafe fn immutable_ref_to_mutable<T>(reference: &T) -> &mut T {
    let const_ptr = reference as *const T;
    let mut_ptr = const_ptr as *mut T;
    return &mut *mut_ptr;
}
