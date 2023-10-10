use bevy::{
    DefaultPlugins,
    prelude::App,
};
use utils::LogFramesPlugin;

mod utils;

fn main() {
    let mut app = App::new();

    app.add_plugins((
        DefaultPlugins,
        LogFramesPlugin::default()
    ));

    app.run();
}
