use std::error::Error;

use self_update::{backends::github, cargo_crate_version, Status};

/// Updates the app, returns true if it has been updated, false if it has not.
pub fn update_version() -> Result<bool, Box<dyn Error>> {
    let status = github::Update::configure()
        .repo_owner("IvoteSligte")
        .repo_name("ball_game")
        .bin_path_in_archive("ball_game/ball_game")
        .bin_name("ball_game")
        .current_version(cargo_crate_version!())
        .no_confirm(true)
        .show_output(false)
        .show_download_progress(false)
        .build()?
        .update()?;
    
    match status {
        Status::Updated(_) => Ok(true),
        Status::UpToDate(_) => Ok(false),
    }
}