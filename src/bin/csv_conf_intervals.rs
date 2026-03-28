use std::{env, str::FromStr};
use stats_ci::{Confidence, StatisticsOps, mean, proportion, quantile};

fn main() {
    let args = env::args().collect::<Vec<_>>();
    if args.len() < 2 {
        println!("Usage: csv_conf_intervals [-ci=confidence] file1.csv [file2.csv...]");
    } else {
        let mut confidence = 0.95;
        println!("| File | Success | # Iterations | Transcript Length | Duration | Mean Iteration | Max Iteration | Closest Particle Rank | Closest to Actual | Best to Actual | Farthest to Actual | Odometry to Actual |");
        println!("|------|---------|--------------|-------------------|----------|----------------|---------------|-----------------------|-------------------|----------------|--------------------|--------------------|");
        for filename in args.iter().skip(1) {
            if filename.starts_with("-ci") {
                confidence = filename.split('=').skip(1).next().unwrap().parse::<f64>().unwrap();
            } else if let Err(e) = handle_filename(filename, confidence) {
                eprintln!("Error {e} processing {filename}");
            }
        }
    }
}

fn handle_filename(filename: &str, confidence: f64) -> anyhow::Result<()> {
    let contents = std::fs::read_to_string(&filename)?;
    let rows: Vec<Row> = contents.lines().skip_while(|line| !(line.starts_with("0,") || line.starts_with("1,"))).take_while(|line| line.starts_with("0,") || line.starts_with("1,")).map(|line| line.parse::<Row>()).collect::<Result<Vec<Row>, _>>()?;
    print!("| {filename} | ");
    show_conf_intervals(&rows, confidence)?;
    Ok(())
}

fn show_conf_intervals(rows: &Vec<Row>, confidence: f64) -> anyhow::Result<()> {
    let (lo, hi) = bimodal_conf_interval(confidence, rows);
    if lo == hi {
        print!("{lo} | ");
    } else {
        print!("[{lo}:{hi}] | ");
    }
    let (lo, hi) = median_conf_interval(confidence, rows, |r| Some(r.num_iterations))?;
    print!("[{lo}:{hi}] | {} | ", rows[0].transcript_len);
    let (lo, hi) = mean_conf_interval(confidence, rows, |r| Some(r.duration))?;
    print!("[{lo:.2}:{hi:.2}] | ");
    let (lo, hi) = mean_conf_interval(confidence, rows, |r| Some(r.mean_iteration_time))?;
    print!("[{lo:.2}:{hi:.2}] | ");
    let (lo, hi) = mean_conf_interval(confidence, rows, |r| Some(r.max_iteration_time))?;
    print!("[{lo:.2}:{hi:.2}] | ");
    let (lo, hi) = median_conf_interval(confidence, rows, |r| if r.success {Some(r.closest_particle_rank)} else {None})?;
    print!("[{lo}:{hi}] | ");
    let (lo, hi) = mean_conf_interval(confidence, rows, |r| if r.success {Some(r.closest_particle_to_actual)} else {None})?;
    print!("[{lo:.2}:{hi:.2}] | ");
    let (lo, hi) = mean_conf_interval(confidence, rows, |r| if r.success {Some(r.best_particle_to_actual)} else {None})?;
    print!("[{lo:.2}:{hi:.2}] | ");
    let (lo, hi) = mean_conf_interval(confidence, rows, |r| if r.success {Some(r.farthest_particle_to_actual)} else {None})?;
    print!("[{lo:.2}:{hi:.2}] | ");
    if let Some(odometry_to_actual) = odometry_to_actual(rows) {
        print!("{odometry_to_actual:.2} |");
    }
    println!();
    Ok(())
}

fn odometry_to_actual(rows: &Vec<Row>) -> Option<f64> {
    rows.iter().find(|r| r.success).map(|r| r.odometry_to_actual)
}

fn bimodal_conf_interval(confidence: f64, rows: &Vec<Row>) -> (usize,usize) {
    let confidence = Confidence::new_two_sided(confidence);
    let successes = rows.iter().filter(|r| r.success).count();
    match proportion::ci(confidence, rows.len(), successes) {
        Err(_) => (successes, successes),
        Ok(interval) => ((interval.low_f() * rows.len() as f64) as usize, (interval.high_f() * rows.len() as f64) as usize)
    }
}

fn median_conf_interval<N: Copy + PartialOrd, F:Fn(&Row)->Option<N>>(confidence: f64, rows: &Vec<Row>, selector: F) -> anyhow::Result<(N, N)> {
    let values = rows.iter().filter_map(|r| selector(r)).collect::<Vec<_>>();
    let confidence = Confidence::new_two_sided(confidence);
    let interval = quantile::ci(confidence, &values, 0.5)?;
    Ok((interval.low().unwrap(), interval.high().unwrap()))
}

fn mean_conf_interval<F:Fn(&Row)->Option<f64>>(confidence: f64, rows: &Vec<Row>, selector: F) -> anyhow::Result<(f64, f64)> {
    let values = rows.iter().filter_map(|r| selector(r)).collect::<Vec<_>>();
    let confidence = Confidence::new_two_sided(confidence);
    let arith = mean::Arithmetic::from_iter(&values)?;
    let interval = arith.ci_mean(confidence)?;
    Ok((interval.low_f(), interval.high_f()))
}

#[derive(Default, Copy, Clone, Debug)]
struct Row {
    success: bool,
    num_iterations: usize,
    transcript_len: usize,
    duration: f64,
    mean_iteration_time: f64,
    max_iteration_time: f64, 
    closest_particle_rank: usize,
    closest_particle_to_actual: f64,
    best_particle_to_actual: f64,
    farthest_particle_to_actual: f64,
    odometry_to_actual: f64,
}

impl FromStr for Row {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let parts = s.split(',').collect::<Vec<_>>();
        if parts.len() < 6 {
           return Err(anyhow::anyhow!("Too few values: {parts:?}"));
        }
        let mut result = Self::default();
        result.success = match parts[0] {
            "0" => false,
            "1" => true,
            _ => return Err(anyhow::anyhow!("Bad success value: {parts:?}"))
        };
        result.num_iterations = parts[1].parse::<usize>()?;
        result.transcript_len = parts[2].parse::<usize>()?;
        result.duration = parts[3].parse::<f64>()?;
        result.mean_iteration_time = parts[4].parse::<f64>()?;
        result.max_iteration_time = parts[5].parse::<f64>()?;
        if result.success {
            result.closest_particle_rank = parts[6].parse::<usize>()?;
            result.closest_particle_to_actual = parts[7].parse::<f64>()?;
            result.best_particle_to_actual = parts[8].parse::<f64>()?;
            result.farthest_particle_to_actual = parts[9].parse::<f64>()?;
            result.odometry_to_actual = parts[10].parse::<f64>()?;
        }
        Ok(result)
    }
}