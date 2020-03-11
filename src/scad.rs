use crate::geom;
use crate::threed::{Pt3, Vec3};
use std::collections::HashMap;
use std::env;
use std::ffi::OsStr;

use std::fs;
use std::io::{Write};
use std::ops::Add;
use std::process::Command;
use tempfile;
use uuid;

pub trait ToScad {
    fn to_scad(&self) -> ScadCode;
}

impl ToScad for geom::Sphere {
    fn to_scad(&self) -> ScadCode {
        scad(&format!("sphere(r={:#?});", &self.radius)).translate(self.center - Pt3::zero())
    }
}

impl ToScad for geom::Cube {
    fn to_scad(&self) -> ScadCode {
        scad(&format!(
            "cube([{:#?}, {:#?}, {:#?}], center = true);",
            self.dimensions.x, self.dimensions.y, self.dimensions.z
        ))
        .translate(self.center - Pt3::zero())
    }
}

pub struct StlImport {
    path: String,
}

impl StlImport {
    pub fn new(path: &str) -> StlImport {
        StlImport {
            path: path.to_string(),
        }
    }
}

impl ToScad for StlImport {
    fn to_scad(&self) -> ScadCode {
        let mut imported_files = HashMap::new();

        // We map all import paths to UUIDs so we can copy the imported files into the same tempdir
        // that the generated scade code lives in. We do this because OpenScad can't handle
        // absolute file paths for some reason.
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
        let uuid = String::from_utf8_lossy(&uuid_buffer[0..length]);

        let extension = std::path::Path::new(&self.path)
            .extension()
            .and_then(OsStr::to_str)
            .unwrap_or("dat");

        let key = format!("{}.{}", uuid, extension);
        imported_files.insert(key.clone(), self.path.clone());
        ScadCode {
            script: vec![format!("import(\"{}\");", key)],
            imported_files,
        }
    }
}

pub struct ScadCode {
    script: Vec<String>,
    imported_files: HashMap<String, String>,
}

impl ScadCode {
    pub fn render(self, output_filepath: &str) -> std::io::Result<std::process::Output> {
        let executable: String = if let Ok(env_var) = env::var("OPENSCAD") {
            env_var
        } else if cfg!(target_os = "windows") {
            String::from("OpenScad.exe")
        } else {
            String::from("openscad")
        };

        let tempdir = tempfile::tempdir()?;
        let scad_path = tempdir.path().join("code.scad");
        let mut scad_file = fs::File::create(&scad_path)?;
        scad_file.write_all(self.to_string().as_bytes())?;

        // Copy all imported meshes into the tempdir with their appropriate short names.
        for (key, path) in self.imported_files.iter() {
            fs::copy(path, tempdir.path().join(key)).unwrap();
        }

        println!(
            "Wrote code to scad file at {}",
            &scad_path.to_str().unwrap()
        );
        print!("> ");
        std::io::stdout().flush().unwrap();
        let mut readline = String::new();
        std::io::stdin().read_line(&mut readline).unwrap();
        println!(" Ok");

        Command::new(executable)
            .args(&["-o", output_filepath, scad_path.to_str().unwrap()])
            .output()
    }

    pub fn wrap(self, code: ScadCode) -> ScadCode {
        self + scad(" { ") + code + scad(" } ")
    }

    pub fn hull(self) -> ScadCode {
        scad("hull()").wrap(self)
    }

    pub fn translate(self, translation: Vec3) -> ScadCode {
        scad(&format!(
            "translate([{:#?}, {:#?}, {:#?}])",
            translation.x, translation.y, translation.z
        ))
        .wrap(self)
    }

    pub fn rotate(self, rotation: Vec3) -> ScadCode {
        scad(&format!(
            "rotate([{:#?}, {:#?}, {:#?}])",
            rotation.x, rotation.y, rotation.z
        ))
        .wrap(self)
    }

    pub fn scale(self, scale: Vec3) -> ScadCode {
        scad(&format!(
            "scale([{:#?}, {:#?}, {:#?}])",
            scale.x, scale.y, scale.z
        ))
        .wrap(self)
    }

    pub fn difference(self: ScadCode, two: ScadCode) -> ScadCode {
        scad("difference()").wrap(self + two)
    }

    pub fn union(self: ScadCode, two: ScadCode) -> ScadCode {
        scad("union()").wrap(self + two)
    }
}

impl ToString for ScadCode {
    fn to_string(&self) -> String {
        (&self.script).join("")
    }
}

impl Add<ScadCode> for ScadCode {
    type Output = ScadCode;

    fn add(self, rhs: ScadCode) -> Self::Output {
        let mut script = self.script;
        script.extend(rhs.script);

        let mut imported_files = self.imported_files;
        imported_files.extend(rhs.imported_files);
        ScadCode {
            script,
            imported_files,
        }
    }
}

fn scad(text: &str) -> ScadCode {
    ScadCode {
        script: vec![text.to_string()],
        imported_files: HashMap::new(),
    }
}
