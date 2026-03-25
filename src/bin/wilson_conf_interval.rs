use std::env;

use stats_ci::{Confidence, proportion};

fn main() {
    let args = env::args().collect::<Vec<_>>();
    if args.len() < 4 {
        println!("Usage: wilson_conf_interval confidence trials successes [-multiplier=m]");
    } else {
        let multiplier = if args.len() >= 5 {
            args[4]
                .split('=')
                .skip(1)
                .next()
                .unwrap()
                .parse::<usize>()
                .unwrap()
        } else {
            1
        };
        let confidence_level = args[1].parse::<f64>().unwrap();
        let trials = multiplier * args[2].parse::<usize>().unwrap();
        let successes = multiplier * args[3].parse::<usize>().unwrap();
        let confidence = Confidence::new(confidence_level);
        match proportion::ci(confidence, trials, successes) {
            Ok(ci) => {
                println!("Confidence interval at {confidence_level}.");
                println!("{trials} trials, {successes} successes");
                println!("{ci:?}");
            }
            Err(e) => {
                eprintln!(
                    "Error {e} when computing confidence interval at level {confidence_level} for {successes}/{trials}"
                );
            }
        }
    }
}
