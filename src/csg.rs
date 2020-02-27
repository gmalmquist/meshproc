use std::env;
use std::env::var;
use std::fs;
use std::io;
use std::io::Write;
use std::ops::Add;
use std::process;

use futures::io::ErrorKind;
use tempfile;
use uuid;

use crate::geom;
use crate::threed::{Pt3, ThreeD, Vec3};

pub trait CsgObj {
    type Type: CsgObj;

    fn render_stl(&self, stl_path: &str) -> io::Result<()>;

    fn union(&self, obj: &Self::Type) -> Self::Type;

    fn difference(&self, obj: &Self::Type) -> Self::Type;

    fn intersection(&self, obj: &Self::Type) -> Self::Type;
}

pub trait ToCsg<C: CsgObj> {
    fn to_csg(&self) -> C;
}

impl ToCsg<BlenderCsgObj> for geom::Cube {
    fn to_csg(&self) -> BlenderCsgObj {
        BlenderCsgObj::cube(&self)
    }
}

impl ToCsg<BlenderCsgObj> for geom::Sphere {
    fn to_csg(&self) -> BlenderCsgObj {
        BlenderCsgObj::sphere(&self)
    }
}

impl ToCsg<BlenderCsgObj> for geom::Mesh {
    fn to_csg(&self) -> BlenderCsgObj {
        BlenderCsgObj::mesh(&self)
    }
}

pub struct BlenderCsgObj {
    create_python_code: Box<dyn Fn(&str) -> String>,
}

impl BlenderCsgObj {
    pub fn sphere(sphere: &geom::Sphere) -> Self {
        let radius = sphere.radius;
        let center = sphere.center;
        return BlenderCsgObj {
            create_python_code: Box::new(move |variable_name| {
                let fn_name = random_id();
                [
                    &format!("def {}():", fn_name),
                    &format!("  mesh = bpy.data.meshes.new(r'{}')", variable_name),
                    &format!("  obj = bpy.data.objects.new(r'{}', mesh)", variable_name),
                    "  bpy.context.scene.collection.objects.link(obj)",
                    "  bm = bmesh.new()",
                    // NB: Blender's diameter property here is actually the radius.
                    &format!("  bmesh.ops.create_uvsphere(bm, u_segments=64, v_segments=32, diameter={})", radius),
                    "  bm.to_mesh(mesh)",
                    "  bm.free()",
                    &format!("  obj.location = {}", blender_vec(&center.to_tuple())),
                    "  return obj",
                    &format!("{} = {}()", variable_name, fn_name),
                ].join("\n")
            }),
        };
    }

    pub fn cube(cube: &geom::Cube) -> Self {
        let center = cube.center;
        let dimensions = cube.dimensions;
        return BlenderCsgObj {
            create_python_code: Box::new(move |variable_name| {
                let fn_name = random_id();
                let cube_size = 1.0 / Vec3::new(1., 1., 1.).mag();
                [
                    &format!("def {}():", fn_name),
                    // The "size" of the cube we pass in here is the distance from the center
                    // to one side. We actually want to make sure the *corners* of the cube
                    // are contained within the radius (not the center of the faces), so we have
                    // to scale the radius down by the magnitude of the <1, 1, 1> vector.
                    &format!(
                        "  bpy.ops.mesh.primitive_cube_add(location={}, size={})",
                        blender_vec(&center.to_tuple()),
                        cube_size
                    ),
                    // Cube is automatically selected after creation.
                    "  cube = bpy.context.view_layer.objects.active",
                    &format!("  cube.name = '{}'", variable_name),
                    &format!("  bpy.context.view_layer.objects.active.scale = ({}, {}, {})",
                             dimensions.x, dimensions.y, dimensions.z),
                    "  return cube",
                    &format!("{} = {}()", variable_name, fn_name),
                ]
                    .join("\n")
            }),
        };
    }

    pub fn mesh(mesh: &geom::Mesh) -> Self {
        Self::stl(&mesh.file)
    }

    pub fn stl(path: &str) -> Self {
        let path = String::from(path);
        BlenderCsgObj {
            create_python_code: Box::new(move |variable_name| {
                [
                    format!("bpy.ops.import_mesh.stl(filepath=r'{}')", path),
                    format!("{} = bpy.context.object", variable_name),
                ].join("\n")
            })
        }
    }

    fn get_code(&self, variable_name: &str) -> String {
        return (self.create_python_code)(variable_name);
    }

    fn boolean_modifier(&self, obj: &BlenderCsgObj, operation: &str) -> BlenderCsgObj {
        let fn_name = random_id();
        let a_name = random_id();
        let b_name = random_id();
        let a_code = self.get_code(&a_name);
        let b_code = obj.get_code(&b_name);
        let operation = operation.to_string();
        let create_python_code = move |variable_name: &str| -> String {
            [
                format!("def {}():", fn_name),
                indent(&a_code, 2),
                indent(&b_code, 2),
                format!("  bpy.context.view_layer.objects.active = {}", a_name),
                format!("  {}.select_set(True)", a_name),
                format!(
                    "  mod = {}.modifiers.new(type='BOOLEAN', name='{}-mod')",
                    a_name, a_name
                ),
                format!("  mod.object = {}", b_name),
                format!("  mod.operation = '{}'", operation),
                format!("  bpy.ops.object.modifier_apply(modifier=mod.name)"),
                format!("  bpy.context.scene.collection.objects.unlink({})", b_name),
                format!("  return {}", a_name),
                format!("{} = {}()", variable_name, fn_name),
            ]
                .join("\n")
        };
        return BlenderCsgObj {
            create_python_code: Box::new(create_python_code),
        };
    }
}

impl CsgObj for BlenderCsgObj {
    type Type = BlenderCsgObj;

    fn render_stl(&self, stl_path: &str) -> io::Result<()> {
        let executable: String = if let Ok(env_var) = env::var("BLENDER") {
            env_var
        } else if cfg!(target_os = "windows") {
            String::from("blender.exe")
        } else {
            String::from("blender")
        };

        let tempdir = tempfile::tempdir()?;
        let script_path = tempdir.path().join("script.py");
        let mut script_file = fs::File::create(&script_path)?;

        // Import utilities.
        script_file.write(
            [
                "import bmesh",
                "import bpy",
                "import mathutils",
                "import os",
                "import random",
                "import sys",
                "import time",
            ]
                .join("\n")
                .as_bytes(),
        );
        script_file.write("\n".as_bytes());

        // Load an empty scene.
        script_file.write("bpy.ops.wm.read_homefile(use_empty=True)\n".as_bytes());

        // Generate this object.
        script_file.write_all((self.create_python_code)("main_csg_object").as_bytes())?;
        script_file.write("\n".as_bytes());

        if let Ok(_) = env::var("DEBUG") {
            // Save a .blend file for debugging purposes.
            script_file.write(
                format!(
                    "bpy.ops.wm.save_as_mainfile(filepath=r'{}.blend')\n",
                    &stl_path
                )
                    .as_bytes(),
            );
        }

        // Export the scene as an stl.
        script_file
            .write(format!("bpy.ops.export_mesh.stl(filepath=r'{}')\n", &stl_path).as_bytes());

        println!(
            "Wrote code to scad file at {}",
            &script_path.to_str().unwrap()
        );

        let mut command = process::Command::new(executable);
        let process = command.args(&["--background", "--python", script_path.to_str().unwrap()]);
        match process.output() {
            Ok(output) => {
                let stdout = String::from_utf8(output.stdout).unwrap_or(String::from("n/a"));
                let stderr = String::from_utf8(output.stderr).unwrap_or(String::from("n/a"));
                if output.status.success() {
                    println!("Blender exited with status {}.", output.status.code().unwrap_or(-1));
                    println!("Blender stdout: \n{}", indent(&stdout, 2));
                    println!("Blender stderr: \n{}", indent(&stderr, 2));
                    if stderr.trim().len() > 0 {
                        println!("Since stderr isn't empty, assuming script was incorrect.");
                        if let Ok(script) = std::fs::read(script_path) {
                            if let Ok(_) = std::fs::write("debug.py", script) {
                                println!("Dumped blender script to debug.py for investigation.");
                            }
                        }
                        Err(io::Error::new(
                            ErrorKind::InvalidInput,
                            stderr,
                        ))
                    } else {
                        Ok(())
                    }
                } else {
                    Err(io::Error::new(
                        ErrorKind::Other,
                        format!(
                            "=== stdout ===\n{}\n\n=== stderr ===\n{}",
                            stdout,
                            stderr),
                    ))
                }
            }
            Err(e) => Err(e),
        }
    }

    fn union(&self, obj: &Self::Type) -> Self::Type {
        self.boolean_modifier(obj, "UNION")
    }

    fn difference(&self, obj: &Self::Type) -> Self::Type {
        self.boolean_modifier(obj, "DIFFERENCE")
    }

    fn intersection(&self, obj: &Self::Type) -> Self::Type {
        self.boolean_modifier(obj, "INTERSECT")
    }
}

fn indent(text: &str, amount: u8) -> String {
    let indentation = (0..amount).map(|_| " ").collect::<Vec<&str>>().join("");
    let lines = text.split("\n");

    lines
        .map(|l| format!("{}{}\n", indentation, l))
        .collect::<Vec<String>>()
        .join("")
}

fn random_id() -> String {
    return format!("v{}", random_uuid());
}

fn random_uuid() -> String {
    let mut uuid_buffer = uuid::Uuid::encode_buffer();
    uuid::Uuid::new_v4()
        .to_simple()
        .encode_upper(&mut uuid_buffer);
    let mut length = 0;
    for i in 0..uuid_buffer.len() {
        length = i;
        if uuid_buffer[i] == 0 {
            break;
        }
    }
    String::from_utf8_lossy(&uuid_buffer[0..length]).to_string()
}

fn blender_vec(tup: &(f64, f64, f64)) -> String {
    format!("mathutils.Vector(({}, {}, {}))", tup.0, tup.1, tup.2)
}
