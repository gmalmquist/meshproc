use crate::geom;
use crate::threed::{Pt3, Vec3};
use std::env;
use std::fmt::Display;
use std::fs;
use std::intrinsics::transmute;
use std::io::Write;
use std::ops::Add;
use std::process::Command;
use tempfile;

pub trait ToScad {
    fn to_scad(&self) -> ScadCode;
}

impl ToScad for geom::Sphere {
    fn to_scad(&self) -> ScadCode {
        translate(&self.center).wrap(scad(&format!("sphere(r={:#?});", &self.radius)))
    }
}

impl ToScad for geom::Cube {
    fn to_scad(&self) -> ScadCode {
        translate(&self.center).wrap(scad(&format!(
            "cube([{:#?}, {:#?}, {:#?}], center = true);",
            self.dimensions.x, self.dimensions.y, self.dimensions.z
        )))
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
        scad(&format!("import(\"{}\");", self.path))
    }
}

pub struct ScadCode {
    script: Vec<String>,
}

impl ScadCode {
    pub fn wrap(self, code: ScadCode) -> ScadCode {
        self + scad(" { ") + code + scad(" } ")
    }

    pub fn write_to(self, output_filepath: &str) -> std::io::Result<std::process::Output> {
        let executable: String = if let Ok(env_var) = env::var("OPENSCAD") {
            env_var
        } else if cfg!(target_os = "windows") {
            String::from("OpenScad.exe")
        } else {
            String::from("openscad")
        };

        let tempdir = tempfile::tempdir()?;
        let mut scad_file = fs::File::create(tempdir.path().join("code.scad"))?;
        scad_file.write_all(self.to_string().as_bytes())?;

        Command::new(executable)
            .args(&["-o", output_filepath])
            .output()
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
        ScadCode { script }
    }
}

impl Add<&ScadCode> for ScadCode {
    type Output = ScadCode;

    fn add(self, rhs: &ScadCode) -> Self::Output {
        let mut script = self.script;
        rhs.script.iter().for_each(|s| {
            script.push(s.clone());
        });
        ScadCode { script }
    }
}

fn scad(text: &str) -> ScadCode {
    ScadCode {
        script: vec![text.to_string()],
    }
}

fn translate(point: &Pt3) -> ScadCode {
    scad(&format!("[{:#?}, {:#?}, {:#?}]", point.x, point.y, point.z))
}
