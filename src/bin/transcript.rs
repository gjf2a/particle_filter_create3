use std::env;

use particle_filter_create3::odometry_transcripts::Transcript;

fn main() {
    let args = env::args().collect::<Vec<_>>();
    if args.len() < 2 {
        println!("Usage: transcript filename");
    } else {
        let transcript = Transcript::from_transcript(args[1].as_str()).unwrap();
        println!("Time: {:.2}s", transcript.total_time_seconds());
        println!("Distance: {:.2}m", transcript.total_distance_recorded());
    }
}
