use std::env;

use meshproc::load_mesh_stl;

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 1 {
        println!("An stl file is required as the first parameter.");
        std::process::exit(1);
    }
    println!("Hello, world!");
}
