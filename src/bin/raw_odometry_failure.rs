use std::env;

use particle_filter::{BitGridMap, MapUpdate, PoseEstimate};
use particle_filter_create3::transcript_from;

fn main() {
    let args = env::args().collect::<Vec<_>>();
    if args.len() != 2 {
        println!("Usage: raw_odometry_failure fileneme");
        return;
    }

    let transcript_filename = args[1].as_str();
    println!("Opening {transcript_filename}...");
    let (transcript, _) = match transcript_from(transcript_filename) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("Error {e} when trying to read transcript from {transcript_filename}");
            return;
        }
    };

    let mut map = BitGridMap::new(0.1, 0.2032);
    let mut estimate = PoseEstimate::default();
    for (i, input) in transcript.iter().enumerate() {
        let map_update = MapUpdate::new(&input.denoised(), &mut estimate);
        map.add_map_update(&map_update);
        if !map.is_consistent() {
            println!("Failed at step {i}/{}", transcript.len());
            break;
        }
    }
}
