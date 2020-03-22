use std::cmp::Ordering;
use std::cmp::Ordering::Equal;
use std::collections::{HashMap, HashSet};
use std::env;
use std::fs;
use std::io::Write;
use std::rc::Rc;
use std::sync::{Arc, RwLock};

use futures::executor;
use futures::io::{Error, ErrorKind};
use futures::task::SpawnExt;

use meshproc::{geom, threed, walk_faces};
use meshproc::csg::{BlenderCsgObj, CsgObj, ToCsg};
use meshproc::geom::{Cube, Facecast, FaceLike, Polygon, Shape};
use meshproc::load_mesh_stl;
use meshproc::mesh::{Mesh, MeshBuilder};
use meshproc::scalar::{DenseNumberLine, FloatRange, Interval};
use meshproc::threed::{Pt3, Ray3, Vec3};
use meshproc::VisitResult::{Continue, Skip, Stop};
use std::panic::resume_unwind;

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 1 {
        println!("An stl file is required as the first parameter.");
        std::process::exit(1);
    }

    match &args[0] as &str {
        "cube-normal-test" => return cube_normal_test(),
        "cube-mesh-test" => return cube_mesh_test(),
        _ => {}
    }

    let stl_path = &match get_path(&args[1]) {
        Ok(s) => s,
        Err(e) => {
            println!("Error canonicalizing path: {:#?}", e);
            std::process::exit(1);
        }
    };

    println!("Loading {}", stl_path);

    let mesh = load_mesh_stl(stl_path);
    if let Err(e) = mesh {
        println!("Unable to load mesh: {}", e);
        std::process::exit(1);
    }
    let mesh = mesh.unwrap();

    println!("Loaded: mesh has {} faces.", mesh.face_count());

    let bounds = &mesh.bounds;
    println!("Bounds: {} to {}", bounds.0, bounds.1);

    let mut csg = mesh.to_csg();
    let mesh = Arc::new(mesh);


    match &args[0] as &str {
        "internal-pillars" => {
            for pillar in generate_internal_pillars(Arc::clone(&mesh)) {
                csg = csg.difference(&pillar.to_csg());
            }
        }
        "internal-plateaus" => {
            for plateau in generate_plateau_supports(Arc::clone(&mesh)) {
                csg = csg.difference(&plateau.to_csg());
            }
        }
        "internal-planes" => {
            for structure in generate_support_planes(Arc::clone(&mesh)) {
                csg = csg.difference(&structure.to_csg());
            }
        }
        "signed-distance-test" => {
            for structure in signed_distance_test(Arc::clone(&mesh)) {
                csg = csg.difference(&structure.to_csg());
            }
        }
        "roundtrip" => {
            // no-op, we just leave it alone.
        }
        s => {
            eprintln!("Unknown command {}", s);
            return;
        }
    }


    match csg.render_stl("test3.stl") {
        Ok(()) => {
            println!("Done.");
        }
        Err(e) => {
            println!("Unable to render mesh: {}", e);
            std::process::exit(1);
        }
    }
}

fn generate_support_planes(mesh: Arc<Mesh>) -> Vec<Box<dyn ToCsg<BlenderCsgObj>>> {
    let horizontal_resolution = 1.0;
    let vertical_resolution = 0.1;
    let support_clearance = 0.4;

    let threadpool = executor::ThreadPoolBuilder::new()
        .create()
        .expect("Unable to create thread pool.");

    let (px, rx) = std::sync::mpsc::channel();

    for i in 0..mesh.face_count() {
        let mesh = Arc::clone(&mesh);
        let px = px.clone();
        threadpool.spawn(async move {
            let face = mesh.face(i).unwrap();

            let up = Vec3::up();
            let down = Vec3::down();

            if face.normal().dot(&up).abs() < 0.99 {
                // Isn't horizontal.
                return;
            }

            if face.centroid().z < mesh.bounds.0.z + 1.0 {
                // Is too close to the bottom of the mesh to require support.
                return;
            }

            let mut poly = geom::Polygon::new(face.vertices()
                .map(|v| v.clone())
                .collect());
            for p in &mut poly.vertices {
                p.z -= 0.001;
            }

            // We want a plane between 0.5 and 1 unit below the face.
            let interval = Interval::new(
                face.centroid().z - 1.0,
                face.centroid().z - 0.5,
            );

            px.send((interval, face.area()));
        }).expect("Unable to spawn thread.");
    }
    drop(px);

    let mut plane_positions = DenseNumberLine::new(
        mesh.bounds.0.z, mesh.bounds.1.z, vertical_resolution, |_| 0.0);

    let mut interval_count = 0;
    for (interval, weight) in rx {
        plane_positions.merge(&interval, weight, |a, b| { a + b });
        eprint!("Processed interval {} ({}, {}] weight {}    \r",
                interval_count, interval.start, interval.end, weight);
        interval_count += 1;
        std::io::stderr().flush();
    }
    eprintln!();

    eprintln!("values: [{}]", plane_positions.values().iter()
        .map(|f| format!("{}", f))
        .collect::<Vec<_>>()
        .join(", "));

    let mut plane_position_weights: Vec<_> = plane_positions.values().iter().collect();
    plane_position_weights.dedup_by(|a, b| a == b);
    plane_position_weights.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Equal));
    plane_position_weights.reverse();

    let (px, rx) = std::sync::mpsc::channel();

    for weight in plane_position_weights {
        if *weight <= 0. {
            break;
        }
        eprintln!("\nFinding planes for {}", weight);
        for candidate in plane_positions.intervals_matching(|f| { f == weight }) {
            if candidate.length() < 0.1 {
                continue;
            }
            eprintln!("Candidate plane between {} and {}", candidate.start, candidate.end);
            eprintln!("  weight = {}", weight);
            let mesh = Arc::clone(&mesh);
            let px = px.clone();
            threadpool.spawn(async move {
                let min_pt = Pt3::new(
                    mesh.bounds.0.x,
                    mesh.bounds.0.y,
                    candidate.start,
                );

                let max_pt = Vec3::new(
                    mesh.bounds.1.x,
                    mesh.bounds.1.y,
                    candidate.end,
                );

                let mut voxel = Pt3::zero();
                voxel.z = candidate.center();

                let xpositions: Vec<_> = FloatRange::from_step_size(min_pt.x, max_pt.x, horizontal_resolution).collect();
                let ypositions: Vec<_> = FloatRange::from_step_size(min_pt.y, max_pt.y, horizontal_resolution).collect();

                let mut grid = vec![vec![0.; xpositions.len()]; ypositions.len()];

                let directions = vec![
                    Vec3::up(),
                    Vec3::down(),
                    Vec3::left(),
                    Vec3::right(),
                    Vec3::forward(),
                    Vec3::backward(),
                ];

                let mut ray = Ray3::new(Pt3::zero(), Vec3::zero());

                for (j, &x) in xpositions.iter().enumerate() {
                    voxel.x = x;
                    for (i, &y) in ypositions.iter().enumerate() {
                        eprint!("computing voxel sdist {}, {}\r", i, j);
                        std::io::stderr().flush();
                        voxel.y = y;

                        let mut closest_hit = None;

                        ray.origin.set(&voxel);
                        for dir in &directions {
                            ray.direction.set(&dir);
                            if let Some(hit) = mesh.raycast(&ray) {
                                if closest_hit.is_none() {
                                    closest_hit = Some(hit.distance);
                                } else {
                                    closest_hit = Some(closest_hit.unwrap().min(hit.distance));
                                }
                            } else {
                                // If we didn't hit anything, we're outside (*gasp*) and thus
                                // cannot place internal support here. So count it the same as
                                // being inside a wall.
                                closest_hit = Some(0.);
                                break;
                            }
                        }

                        grid[i][j] = -closest_hit.unwrap_or(0.);
                    }
                }

                let mut used: HashSet<(usize, usize)> = HashSet::new();

                let clearance = support_clearance;
                for row in 0..(grid.len() - 1) {
                    for col in 0..(grid[row].len() - 1) {
                        eprint!("checking voxel {}, {}\r", row, col);
                        std::io::stderr().flush();

                        if used.contains(&(row, col)) {
                            continue;
                        }
                        if grid[row][col] > -clearance {
                            // Must be inside, and at least |clearance| away from anything.
                            continue;
                        }

                        let mut max_row = row;
                        let mut max_col = col;

                        loop {
                            let mut can_expand_col = false;
                            if max_col + 1 < grid[row].len() {
                                can_expand_col = true;
                                for i in row..(max_row + 1) {
                                    let coord = (i, max_col + 1);
                                    if used.contains(&coord) || grid[coord.0][coord.1] > -clearance {
                                        can_expand_col = false;
                                        break;
                                    }
                                }
                            }

                            if can_expand_col {
                                max_col += 1;
                            }

                            let mut can_expand_row = false;
                            if max_row + 1 < grid.len() {
                                can_expand_row = true;
                                for j in col..(max_col + 1) {
                                    let coord = (max_row + 1, j);
                                    if used.contains(&coord) || grid[coord.0][coord.1] > -clearance {
                                        can_expand_row = false;
                                        break;
                                    }
                                }
                            }

                            if can_expand_row {
                                max_row += 1;
                            }

                            if !can_expand_col && !can_expand_row {
                                break;
                            }
                        }

                        if max_row == row || max_col == col {
                            // Not large enough to be useful.
                            continue;
                        }

                        for i in row..(max_row + 0) {
                            for j in col..(max_col + 0) {
                                used.insert((i, j));
                            }
                        }

                        eprintln!("\nGenerating cube from ({}, {}) to ({}, {})",
                                  row, col, max_row, max_col);

                        let cube = Cube::new(
                            Pt3::new(
                                xpositions[col],
                                ypositions[row],
                                candidate.start,
                            ),
                            Vec3::new(
                                xpositions[max_col] - xpositions[col],
                                ypositions[max_row] - ypositions[row],
                                candidate.length(),
                            ),
                            false,
                        );
                        px.send(cube);
                    }
                }
            });
        }
    }
    drop(px);

    let mut structures: Vec<Box<dyn ToCsg<BlenderCsgObj>>> = vec![];
    for cube in rx {
        structures.push(Box::new(cube));
    }

    structures
}


fn generate_plateau_supports(mesh: Arc<Mesh>) -> Vec<Box<dyn ToCsg<BlenderCsgObj>>> {
    let mut structures: Vec<Box<dyn ToCsg<BlenderCsgObj>>> = vec![];

    let clearance = 1.0;

    let up = Vec3::up();
    let down = Vec3::down();

    let faces_used = Arc::new(RwLock::new(HashSet::new()));

    let threadpool = executor::ThreadPoolBuilder::new()
        .create()
        .expect("Couldn't create a thread pool.");

    let (px, rx) = std::sync::mpsc::channel();
    let mut futures = vec![];

    for (i, face) in mesh.faces().enumerate() {
        let mesh = Arc::clone(&mesh);
        let faces_used = Arc::clone(&faces_used);
        let px = px.clone();
        futures.push(threadpool.spawn_with_handle(async move {
            let face = mesh.face(i).unwrap();
            if faces_used.read().unwrap().contains(&i) {
                return;
            }

            if face.area() < (clearance * 2.) * (clearance * 2.) {
                return;
            }

            if face.normal().dot(&up).abs() < 0.99 {
                // Isn't horizontal.
                return;
            }

            if face.centroid().z < mesh.bounds.0.z + clearance {
                // Is too close to the bottom of the mesh to require support.
                return;
            }

            let face = Rc::new(face);
            let mut poly = geom::Polygon::new(face.clone_vertices());

            let centroid = poly.centroid();

            for v in &mut poly.vertices {
                *v = *v + (centroid - *v).normalized();
                v.z -= 0.001; // Makes sure we don't collide with ourselves for raycasting.
            }

            let downcast = mesh.facecast(&poly, &down);
            if downcast.is_none() {
                // There's nothing below us, that means this face is actually the bottom surface of the
                // mesh, and we don't want to do anything with it.
                return;
            }
            let downcast = downcast.unwrap();

            if downcast.distance < clearance * 2. {
                // We're too close to existing geometry below us to warrant generating any internal
                // support structures.
                return;
            }

            eprint!("Face {} may have a plateau.\r", i);

            let face_group = RwLock::new(HashSet::new());
            {
                let mut fg = face_group.write().unwrap();
                let start_face = Rc::clone(&face);
                let face = Rc::clone(&face);
                let faces_used = faces_used.write().unwrap();
                walk_faces(&mesh, &start_face, move |f| {
                    if faces_used.contains(&f.index()) {
                        return Skip;
                    }
                    if f.normal().dot(&face.normal()) < 0.99 {
                        return Skip;
                    }
                    fg.insert(f.index());
                    Continue
                });
            }

            let face_group = face_group.read().unwrap();
            let mut faces_used = faces_used.write().unwrap();
            for face in face_group.iter() {
                faces_used.insert(*face);
            }

            px.send((poly, downcast.distance));
        }).unwrap());
    }
    drop(px);

    for (face, distance) in rx {
        // Let's generate a column under this face!
        eprintln!("Generating a plateau for {:#?}", face.clone_vertices());

        let mut mb = MeshBuilder::new();

        // Top face is just the original face shifted down by the clearance.
        let top_face: Vec<usize> = face.vertices()
            .map(|v| mb.add_vertex(v.clone() + (down * clearance)))
            .collect();
        mb.add_face(top_face.iter().map(|v| *v).collect());

        // Bottom face is original face, shifted down by the downcast distance less our clearance.
        let bottom_face: Vec<usize> = face.vertices()
            .map(|v| mb.add_vertex(v.clone() + (down * (distance - clearance))))
            .collect();
        // Note the rev() here- we have to reverse the order of the coordinates to ensure the
        // normal is correct.
        mb.add_face(bottom_face.iter().rev().map(|v| *v).collect());

        for i in 0..top_face.len() {
            let top_a = top_face[i];
            let top_b = top_face[(i + 1) % top_face.len()];
            let bot_a = bottom_face[i];
            let bot_b = bottom_face[(i + 1) % bottom_face.len()];

            mb.add_face(vec![top_a, top_b, bot_b, bot_a]);
        }

        structures.push(Box::new(mb.build()));
    }

    structures
}

fn cube_normal_test() {
    let cube = geom::Cube::new(Pt3::zero(), Vec3::new(10., 10., 10.), true);

    let mut csg = cube.to_csg();

    for face in cube.faces() {
        println!("Face: {}, {}", face.normal(), face.normal() * 10.);
        for point in face.points(1.) {
            csg = csg.union(
                &geom::Cube::new(point + (face.normal() * 3.), Vec3::new(0.5, 0.5, 0.5), true)
                    .to_csg(),
            );
        }
    }

    csg.render_stl("test-normals.stl").expect("Expected test normals to render.");
}

fn cube_mesh_test() {
    let cube = geom::Cube::new(Pt3::zero(), Vec3::new(10., 10., 10.), true);
    let csg = cube.mesh.to_csg();
    csg.render_stl("cube-mesh-test.stl").expect("Failed to render cube mesh stl.");
}

fn signed_distance_test(mesh: Arc<Mesh>) -> Vec<Box<dyn ToCsg<BlenderCsgObj>>> {
    let mut results: Vec<Box<dyn ToCsg<BlenderCsgObj>>> = vec![];
    let resolution = 5.;
    let (min, max) = mesh.bounds;

    let threadpool = executor::ThreadPoolBuilder::new().create().unwrap();
    let (px, rx) = std::sync::mpsc::channel();

    for x in FloatRange::from_step_size(min.x, max.x, resolution) {
        for y in FloatRange::from_step_size(min.y, max.y, resolution) {
            for z in FloatRange::from_step_size(min.z, max.z, resolution) {
                let px = px.clone();
                let mesh = Arc::clone(&mesh);
                threadpool.spawn(async move {
                    let pt = Pt3::new(x, y, z);
                    eprint!("Processing {:.1}      \r", pt);
                    let sd = mesh.signed_distance(&pt);
                    px.send((pt, sd));
                });
            }
        }
    }
    drop(px);

    for (pt, sd) in rx {
        let radius = 0.1_f64.max((sd.abs() + 1.0) / (max - min).mag());
        if sd < 0. {
            results.push(Box::new(geom::Cube::new(
                pt.clone(),
                Vec3::new(radius, radius, radius),
                true,
            )));
        } else {
            results.push(Box::new(geom::Sphere::new(
                pt.clone(),
                radius,
            )));
        }
    }

    eprintln!("\nDone.");

    results
}

fn generate_internal_pillars(mesh: Arc<Mesh>) -> Vec<geom::Cube> {
    let mut pillars = vec![];

    let resolution = 12.;
    let clearance = 1.0;
    let cell_width = resolution - clearance;

    let (mind, maxd) = &mesh.bounds;

    let mut futures = vec![];
    let threadpool = executor::ThreadPoolBuilder::new()
        .create()
        .expect("Unable to create thread pool for pillar generation!");

    for x in FloatRange::from_step_size(
        mind.x + clearance,
        maxd.x - cell_width - clearance,
        resolution,
    ) {
        for y in FloatRange::from_step_size(
            mind.y + clearance,
            maxd.y - cell_width - clearance,
            resolution,
        ) {
            let mesh = Arc::clone(&mesh);
            let pillar_future = threadpool.spawn_with_handle(async move {
                eprint!("Generating pillar {} x {}              \r", x, y);
                std::io::stderr().flush().unwrap();
                generate_pillar(x, y, cell_width, &mesh, clearance)
            });
            if let Ok(pillar_future) = pillar_future {
                futures.push(pillar_future);
            }
        }
    }

    for pillar_future in futures {
        if let Some(pillar) = futures::executor::block_on(pillar_future) {
            pillars.push(pillar);
        }
    }

    println!("\nGenerated {} pillars.", pillars.len());
    pillars
}

fn generate_pillar(
    base_x: f64,
    base_y: f64,
    side: f64,
    mesh: &Mesh,
    clearance: f64,
) -> Option<geom::Cube> {
    let ray_spacing = 1.0;
    let (mind, _maxd) = &mesh.bounds;
    let base_z = mind.z + clearance;

    let bottom_face = Polygon::new(vec![
        Pt3::new(base_x, base_y, base_z),
        Pt3::new(base_x + side, base_y, base_z),
        Pt3::new(base_x + side, base_y + side, base_z),
        Pt3::new(base_x, base_y + side, base_z),
    ]);

    let mut height: Option<f64> = None;
    for pt in bottom_face.points(ray_spacing) {
        let hit = mesh.raycast(&Ray3::new(pt, Vec3::up()));
        if let Some(hit) = hit {
            if height.is_none() || height.unwrap() > hit.distance {
                height = Some(hit.distance);
            }
        }
    }

    if let Some(height) = height {
        let height = height - clearance;
        if height < clearance {
            // NB: We use the clearance as the minimum valid height, which seems reasonable
            // but may not be obvious.
            return None;
        }

        let cube = Cube::new(
            Pt3::new(base_x, base_y, base_z),
            Vec3::new(side, side, height),
            false,
        );

        let mut lowest_tangent_collision: Option<f64> = None;
        for face in cube.faces() {
            for point in face.points(ray_spacing) {
                let hit = mesh.raycast(&Ray3::new(point.clone(), face.normal()));
                let hit_height = if let Some(hit) = hit {
                    if hit.distance >= clearance {
                        continue; // This is fine.
                    }
                    hit.point.z - base_z
                } else {
                    // If we didn't hit anything, we're outside the mesh! This is just as bad as
                    // hitting something too close.
                    point.z - base_z
                };
                if lowest_tangent_collision.is_none()
                    || lowest_tangent_collision.unwrap() > hit_height
                {
                    lowest_tangent_collision = Some(hit_height);
                }
            }
        }
        if let Some(lowest_tangent_collision) = lowest_tangent_collision {
            let new_height = lowest_tangent_collision - clearance;
            if new_height < clearance {
                return None;
            }
            return Some(Cube::new(
                Pt3::new(base_x, base_y, base_z),
                Vec3::new(side, side, new_height),
                false,
            ));
        }

        return Some(cube);
    }
    None
}

fn csg_test() {
    println!("Running CSG test.");

    let cube = geom::Cube::new(threed::Pt3::zero(), threed::Vec3::new(10., 20., 30.), false);
    let sphere = geom::Sphere::new(threed::Pt3::new(5., 10., 15.), 5.);

    let csg = cube.to_csg().union(&sphere.to_csg());
    match csg.render_stl("test2.stl") {
        Ok(_output) => {}
        Err(e) => {
            eprintln!("Error: {:#?}", e);
            std::process::exit(1);
        }
    }
}

fn get_path<'a>(path: &'a str) -> std::io::Result<String> {
    let canonical_path = fs::canonicalize(path)?;
    let str_path = &canonical_path.to_str();
    match str_path {
        None => Err(Error::new(
            ErrorKind::InvalidData,
            "Unable to convert path to string.",
        )),
        Some(s) => Ok(s.to_string()),
    }
}
