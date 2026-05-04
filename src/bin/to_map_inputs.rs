use particle_filter_create3::odometry_transcripts::Transcript;

fn main() -> anyhow::Result<()> {
    for arg in std::env::args().skip(1) {
        if let Some(suffix) = arg.rfind('.') {
            let transcript = Transcript::from_transcript(&arg)?;
            let map_filename = format!("{}.mi", &arg[..suffix]);
            let map_strs = transcript
                .map_inputs()
                .iter()
                .map(|mi| format!("{mi}"))
                .collect::<Vec<_>>();
            std::fs::write(map_filename, map_strs.join("\n"))?;
        }
    }
    Ok(())
}
