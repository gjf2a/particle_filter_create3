use hash_histogram::HashHistogram;
use stats_ci::{Confidence, StatisticsOps, mean, proportion, quantile};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    str::FromStr,
};

fn main() {
    let args = arg_vals::ArgVals::env();
    let confidence: f64 = args.get_value("-ci").unwrap_or(0.95);
    let pattern = args.get_str_value("-pat").ok();
    let args = env::args().collect::<Vec<_>>();
    if args.len() >= 2 && args[1].starts_with("-h") {
        println!("Usage: csv_conf_intervals [-ci=confidence] [-pat=filename_substring_pattern]");
        return;
    }

    match get_headers_rows(pattern) {
        Err(e) => {
            println!("Error when processing rows: {e}");
        }
        Ok(rows) => {
            if let Err(e) = print_rows(confidence, &rows) {
                println!("Error when printing rows: {e}");
            }
        }
    }
}

fn get_headers_rows(pattern: Option<&String>) -> anyhow::Result<BTreeMap<String, Vec<Row>>> {
    let mut rows = BTreeMap::new();
    for file in std::fs::read_dir(".")? {
        let file = file?;
        if let Ok(filename) = file.file_name().into_string() {
            if filename.ends_with(".csv") && pattern.map_or(true, |p| filename.contains(p)) {
                if let Err(e) = process_file(&filename, &mut rows) {
                    println!("Error when processing file '{filename}': {e}");
                }
            }
        }
    }

    Ok(rows)
}

fn process_file(filename: &str, rows: &mut BTreeMap<String, Vec<Row>>) -> anyhow::Result<()> {
    let contents = std::fs::read_to_string(&filename)?;
    let expr_src = &filename[..filename.find(".out").ok_or(anyhow::anyhow!("No clear source file for {filename}"))?];
    let mut lines = contents.lines();
    let header = lines.by_ref().skip(1).next().unwrap();
    let header = format!("{expr_src},{}", pop_trailing_commas(header));
    match rows.get_mut(&header) {
        None => {
            let mut value = vec![];
            add_rows(lines, &mut value)?;
            rows.insert(header.to_string(), value);
        }
        Some(value) => {
            add_rows(lines, value)?;
        }
    }
    Ok(())
}

fn pop_trailing_commas(header: &str) -> String {
    let mut result = header.to_string();
    while result.ends_with(",") {
        result.pop();
    }
    result
}

fn print_rows(confidence: f64, rows: &BTreeMap<String, Vec<Row>>) -> anyhow::Result<()> {
    println!(
        "| Category | Count | Success | # Iterations | Transcript Length | Duration | Mean Iteration | Max Iteration | Closest Particle Rank | Closest to Actual | Best to Actual | Farthest to Actual | Odometry to Actual |"
    );
    println!(
        "|----------|-------|---------|--------------|-------------------|----------|----------------|---------------|-----------------------|-------------------|----------------|--------------------|--------------------|"
    );

    let headers = rows.keys().collect::<BTreeSet<_>>();
    let header2descriptor = descriptors(&headers);
    for (header, rows) in rows.iter() {
        let descriptor = header2descriptor.get(header).unwrap();
        if let Err(e) = print_row(descriptor, rows, confidence) {
            println!("Error when processing {descriptor}: {e}")
        }
    }
    Ok(())
}

fn print_row(descriptor: &str, rows: &Vec<Row>, confidence: f64) -> anyhow::Result<()> {
    print!("| {descriptor} | {} |", rows.len());
    show_conf_intervals(&rows, confidence)?;
    Ok(())
}

fn descriptors(headers: &BTreeSet<&String>) -> BTreeMap<String, String> {
    let mut field_counts: HashHistogram<String, usize> = HashHistogram::new();
    for header in headers.iter() {
        for field in header.split(',') {
            field_counts.bump(&field.to_string());
        }
    }
    let interesting_fields = field_counts
        .iter()
        .filter(|(_, c)| **c < headers.len())
        .map(|(f, _)| f.clone())
        .collect::<BTreeSet<_>>();
    let mut result = BTreeMap::new();
    for header in headers.iter() {
        let interesting = header
            .split(',')
            .filter(|s| interesting_fields.contains(*s))
            .collect::<Vec<_>>();
        result.insert((*header).clone(), interesting.join(","));
    }
    result
}

fn is_data_row(row: &str) -> bool {
    row.starts_with("0,") || row.starts_with("1,")
}

fn add_rows<'a, I: Iterator<Item = &'a str>>(lines: I, rows: &mut Vec<Row>) -> anyhow::Result<()> {
    for line in lines
        .skip_while(|r| !is_data_row(r))
        .take_while(|r| is_data_row(r))
    {
        rows.push(line.parse::<Row>()?);
    }
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
    let (lo, hi) = median_conf_interval(confidence, rows, |r| {
        if r.success {
            Some(r.closest_particle_rank)
        } else {
            None
        }
    })?;
    print!("[{lo}:{hi}] | ");
    let (lo, hi) = mean_conf_interval(confidence, rows, |r| {
        if r.success {
            Some(r.closest_particle_to_actual)
        } else {
            None
        }
    })?;
    print!("[{lo:.2}:{hi:.2}] | ");
    let (lo, hi) = mean_conf_interval(confidence, rows, |r| {
        if r.success {
            Some(r.best_particle_to_actual)
        } else {
            None
        }
    })?;
    print!("[{lo:.2}:{hi:.2}] | ");
    let (lo, hi) = mean_conf_interval(confidence, rows, |r| {
        if r.success {
            Some(r.farthest_particle_to_actual)
        } else {
            None
        }
    })?;
    print!("[{lo:.2}:{hi:.2}] | ");
    if let Some(odometry_to_actual) = odometry_to_actual(rows) {
        print!("{odometry_to_actual:.2} |");
    }
    println!();
    Ok(())
}

fn odometry_to_actual(rows: &Vec<Row>) -> Option<f64> {
    rows.iter()
        .find(|r| r.success)
        .map(|r| r.odometry_to_actual)
}

fn bimodal_conf_interval(confidence: f64, rows: &Vec<Row>) -> (usize, usize) {
    let confidence = Confidence::new_two_sided(confidence);
    let successes = rows.iter().filter(|r| r.success).count();
    match proportion::ci(confidence, rows.len(), successes) {
        Err(_) => (successes, successes),
        Ok(interval) => (
            (interval.low_f() * rows.len() as f64) as usize,
            (interval.high_f() * rows.len() as f64) as usize,
        ),
    }
}

fn median_conf_interval<N: Copy + PartialOrd, F: Fn(&Row) -> Option<N>>(
    confidence: f64,
    rows: &Vec<Row>,
    selector: F,
) -> anyhow::Result<(N, N)> {
    let values = rows.iter().filter_map(|r| selector(r)).collect::<Vec<_>>();
    let confidence = Confidence::new_two_sided(confidence);
    let interval = quantile::ci(confidence, &values, 0.5)?;
    Ok((interval.low().unwrap(), interval.high().unwrap()))
}

fn mean_conf_interval<F: Fn(&Row) -> Option<f64>>(
    confidence: f64,
    rows: &Vec<Row>,
    selector: F,
) -> anyhow::Result<(f64, f64)> {
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
        let parts = s.split(',').filter(|s| s.trim().len() > 0).collect::<Vec<_>>();
        if parts.len() < 6 {
            return Err(anyhow::anyhow!("Too few values: '{parts:?}'"));
        }
        let mut result = Self::default();
        result.success = match parts[0] {
            "0" => false,
            "1" => true,
            _ => return Err(anyhow::anyhow!("Bad success value: {parts:?}")),
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
