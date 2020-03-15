use std::cmp::max;
use std::f64::INFINITY;
use std::io;
use std::io::Write;

use byteorder::{LittleEndian, WriteBytesExt};

use crate::geom;
use crate::geom::HasVertices;
use crate::scalar::FloatRange;
use crate::threed::{Basis3, Frame3, Pt3, Ray3, Vec3};

pub struct Mesh {
    pub vertices: Vec<Pt3>,
    pub face_loops: Vec<Vec<usize>>,
    pub source_file: Option<String>,
    pub bounds: (Pt3, Pt3),
    // implied geometry
    face_normals: Vec<Vec3>,
    face_centroids: Vec<Pt3>,
    vertex_normals: Vec<Vec3>,
    // adjacency info
    corners: Vec<RawCorner>,
    face_corners: Vec<Vec<usize>>,
    left_corners: Vec<usize>,
    right_corners: Vec<usize>,
}

impl Mesh {
    pub fn new(vertices: Vec<Pt3>, face_loops: Vec<Vec<usize>>, source_file: Option<String>) -> Self {
        let mut new_loops = vec![];
        for (loop_index, lp) in face_loops.into_iter().enumerate() {
            if lp.len() < 3 {
                eprintln!("Dropping degenerate face loop {} (|v| = {})",
                          loop_index, lp.len());
                continue;
            }
            let mut valid_vertices = true;
            for v in &lp {
                if *v >= vertices.len() {
                    valid_vertices = false;
                    eprintln!("Face loop {} has invalid vertex index {}!",
                              loop_index, v);
                }
            }
            if !valid_vertices {
                continue;
            }

            new_loops.push(lp);
        }
        let mut mesh = Self {
            vertices,
            face_loops: new_loops,
            source_file,
            bounds: (Pt3::zero(), Pt3::zero()),
            face_normals: vec![],
            face_centroids: vec![],
            vertex_normals: vec![],
            corners: vec![],
            face_corners: vec![],
            left_corners: vec![],
            right_corners: vec![],
        };
        // NB: Maybe it makes more sense to do this lazily? But I think almost anything we might
        // want to do with a mesh will care about this data.
        mesh.recalculate_adjacency();
        mesh.recalculate_geometry();
        mesh
    }

    pub fn vertex(&self, index: usize) -> Option<&Pt3> {
        self.vertices.get(index)
    }

    pub fn face(&self, index: usize) -> Option<MeshFace> {
        if index >= self.face_loops.len() {
            return None;
        }
        Some(MeshFace {
            mesh: self,
            index,
        })
    }

    pub fn face_count(&self) -> usize {
        self.face_loops.len()
    }

    pub fn faces(&self) -> MeshFaceIter {
        MeshFaceIter { mesh: self, index: 0 }
    }

    pub fn face_normal(&self, index: usize) -> Option<&Vec3> {
        self.face_normals.get(index)
    }

    pub fn vertex_normal(&self, index: usize) -> Option<&Vec3> {
        self.vertex_normals.get(index)
    }

    pub fn face_centroid(&self, index: usize) -> Option<&Pt3> {
        self.face_centroids.get(index)
    }

    pub fn corner(&self, face_index: usize, face_corner: usize) -> Corner {
        let face = &self.face_corners[face_index];
        Corner::new(self, face[face_corner % face.len()])
    }

    pub fn recalculate_normals(&mut self) {
        self.face_normals.clear();
        for (_index, lp) in self.face_loops.iter().enumerate() {
            let edge_one = &self.vertices[lp[1]] - &self.vertices[lp[0]];
            let edge_two = &self.vertices[lp[2]] - &self.vertices[lp[1]];
            self.face_normals.push(edge_two.cross(&edge_one).normalized());
        }
    }

    pub fn recalculate_bounds(&mut self) {
        let mut first = true;
        for pt in &self.vertices {
            if first {
                self.bounds.0.set(pt);
                self.bounds.1.set(pt);
                first = false;
                continue;
            }

            self.bounds.0.set_min(pt);
            self.bounds.1.set_max(pt);
        }
    }

    pub fn recalculate_centroids(&mut self) {
        self.face_centroids.clear();
        for (_index, lp) in self.face_loops.iter().enumerate() {
            let mut centroid = Pt3::zero();
            for vertex_index in lp {
                let v = self.vertices[*vertex_index];
                centroid.x += v.x;
                centroid.y += v.y;
                centroid.z += v.z;
            }
            centroid.x /= lp.len() as f64;
            centroid.y /= lp.len() as f64;
            centroid.z /= lp.len() as f64;
            self.face_centroids.push(centroid);
        }
    }

    pub fn recalculate_vertex_normals(&mut self) {
        self.vertex_normals.clear();

        for _i in 0..self.vertices.len() {
            self.vertex_normals.push(Vec3::zero());
        }

        for (loop_index, lp) in self.face_loops.iter().enumerate() {
            for vertex_index in lp {
                // TODO: Might be good to overload +=, /=, etc on points and vectors to dry this up?
                self.vertex_normals[*vertex_index].x += self.face_normals[loop_index].x;
                self.vertex_normals[*vertex_index].y += self.face_normals[loop_index].y;
                self.vertex_normals[*vertex_index].z += self.face_normals[loop_index].z;
            }
        }

        for v in &mut self.vertex_normals {
            v.normalize();
        }
    }

    pub fn recalculate_adjacency(&mut self) {
        let mut vertices_to_faces = vec![];

        for i in 0..self.vertices.len() {
            vertices_to_faces.push(vec![]);
        }

        self.corners.clear();
        self.face_corners.clear();
        for (i, face_loop) in self.face_loops.iter().enumerate() {
            self.face_corners.push(vec![]);
            for v in face_loop {
                vertices_to_faces[*v].push(i);
                self.face_corners[i].push(self.corners.len());
                self.corners.push(RawCorner {
                    face: i,
                    vert: *v,
                })
            }
        }

        self.left_corners.clear();
        self.right_corners.clear();

        for (corner_index, c) in self.corners.iter().enumerate() {
            let face = &self.face_loops[c.face];
            let vertex_index = face[c.vert];
            let prev_vertex_index = face[(c.vert + face.len() - 1) % face.len()];
            let mut left_corner = corner_index;
            for adj_face_index in &vertices_to_faces[vertex_index] {
                if *adj_face_index == c.face {
                    continue;
                }
                let adj_face = &self.face_loops[*adj_face_index];
                let mut adj_face_vi = 0;
                for i in 1..adj_face.len() {
                    if adj_face[i] == vertex_index {
                        adj_face_vi = i;
                        break;
                    }
                }
                let adj_next = adj_face[(adj_face_vi + 1) % adj_face.len()];
                if adj_next == prev_vertex_index {
                    // We found our left-adjacent corner!
                    left_corner = self.face_corners[*adj_face_index][adj_face_vi];
                }
            }
            self.left_corners.push(left_corner);

            // This is just for initialization, not the final value.
            self.right_corners.push(corner_index);
        }

        for (corner_index, c) in self.corners.iter().enumerate() {
            if self.right_corners[corner_index] != corner_index {
                continue; // Was already initialized.
            }
            let mut curr = corner_index;
            loop {
                let prev = curr;
                curr = self.left_corners[curr];
                self.right_corners[curr] = prev;
                if curr == corner_index {
                    break;
                }
            }
        }
    }

    pub fn recalculate_geometry(&mut self) {
        self.recalculate_bounds();
        self.recalculate_centroids();
        self.recalculate_normals();
        self.recalculate_vertex_normals();
    }

    pub fn write_stl(&self, writer: &mut dyn io::Write) -> io::Result<()> {
        // https://www.fabbers.com/tech/STL_Format
        // UINT8[80] – Header
        // UINT32 – Number of triangles
        //
        //
        // foreach triangle
        // REAL32[3] – Normal vector
        // REAL32[3] – Vertex 1
        // REAL32[3] – Vertex 2
        // REAL32[3] – Vertex 3
        // UINT16 – Attribute byte count
        // end
        //
        // Note that all triangle vertices must be in the positive octant.

        // Offset to shift all vertices into the positive octant.
        let offset = Pt3::zero() - self.bounds.0;

        let mut triangle_count = 0;
        for face in &self.face_loops {
            if face.len() < 3 {
                panic!("Mesh should never contain faces with fewer than 3 vertices.");
            }
            if face.len() == 3 {
                // 3 vertices make a single triangle.
                triangle_count += 1;
                continue;
            }
            if face.len() == 4 {
                // turn quads into two triangles.
                triangle_count += 2;
                continue;
            }
            // Create a pinwheel of triangles equal to the number of vertices.
            triangle_count += face.len();
        }

        let header: [u8; 80] = [0; 80];
        writer.write_all(&header);
        writer.write_u32::<LittleEndian>(triangle_count as u32);

        let write_triangle = |writer: &mut dyn io::Write, normal: &Vec3, vertices: &Vec<&Pt3>| {
            writer.write_f32::<LittleEndian>(normal.x as f32);
            writer.write_f32::<LittleEndian>(normal.y as f32);
            writer.write_f32::<LittleEndian>(normal.z as f32);

            for point in vertices {
                // Max with 0 is required, because with floats even after adding the offset there's
                // a chance we'll get something silly like -0.0001 or -0.0.
                writer.write_f32::<LittleEndian>((0.0_f64).max(offset.x + point.x) as f32);
                writer.write_f32::<LittleEndian>((0.0_f64).max(offset.y + point.y) as f32);
                writer.write_f32::<LittleEndian>((0.0_f64).max(offset.z + point.z) as f32);
            }

            writer.write_u16::<LittleEndian>(0); // Attribute count, which most tools expect to be 0.
        };

        for (face_index, face) in self.face_loops.iter().enumerate() {
            let normal = self.face_normal(face_index).expect("Normals are required.");

            if face.len() == 3 {
                // Simple triangle.
                write_triangle(writer, normal, &face.iter()
                    .map(|&i| &self.vertices[i])
                    .collect());
                continue;
            }

            if face.len() == 4 {
                // Make two triangles from the quad.
                write_triangle(writer, normal, &face[0..3].iter()
                    .map(|&i| &self.vertices[i])
                    .collect());
                write_triangle(writer, normal, &[face[3], face[0], face[2]].iter()
                    .map(|&i| &self.vertices[i])
                    .collect());
                continue;
            }

            // Triangulate with a pinwheel.
            let pts: Vec<&Pt3> = face.iter().map(|&i| &self.vertices[i]).collect();
            let centroid = Pt3::centroid(&pts);

            for i in 0..pts.len() {
                let a = pts[i];
                let b = pts[(i + 1) % pts.len()];
                let c = &centroid;
                write_triangle(writer, normal, &vec![a, b, c]);
            }
        }

        Ok(())
    }
}

impl geom::Shape for Mesh {
    fn raycast(&self, ray: &Ray3) -> Option<geom::RaycastHit> {
        let mut closest_hit: Option<geom::RaycastHit> = None;
        for poly in self.faces() {
            let poly_hit = poly.raycast(ray);
            if let Some(poly_hit) = poly_hit {
                closest_hit = match closest_hit {
                    None => Some(poly_hit),
                    Some(closest_hit) => {
                        if closest_hit.distance < poly_hit.distance {
                            Some(closest_hit)
                        } else {
                            Some(poly_hit)
                        }
                    }
                };
            }
        }
        closest_hit
    }

    fn signed_distance(&self, pt: Pt3) -> f64 {
        let mut distance = INFINITY;
        for poly in self.faces() {
            let dist = poly.signed_distance(pt);
            if dist < distance {
                distance = dist;
            }
        }
        distance
    }
}

// Used in the internal Mesh representation, but never exposed outside of it (instead, outside code
// uses Corner).
struct RawCorner {
    face: usize,
    vert: usize,
}

pub struct Corner<'m> {
    mesh: &'m Mesh,
    corner_index: usize,
    raw_corner: &'m RawCorner,
}

impl<'m> Corner<'m> {
    fn new(mesh: &'m Mesh, corner_index: usize) -> Self {
        assert!(corner_index < mesh.corners.len());
        Self {
            mesh,
            corner_index,
            raw_corner: &mesh.corners[corner_index],
        }
    }

    /// The vertex index matching this corner.
    pub fn vertex_index(&self) -> usize {
        self.mesh.face_loops[self.raw_corner.face][self.raw_corner.vert]
    }

    /// The actual geometry of the vertex.
    pub fn vertex(&self) -> &'m Pt3 {
        self.mesh.vertex(self.vertex_index()).unwrap()
    }

    /// Get the next corner moving counter-clockwise around the same face.
    pub fn next(&self) -> Self {
        self.mesh.corner(self.raw_corner.face, (self.raw_corner.vert + 1) % self.loop_size())
    }

    /// Get the previous corner (i.e. the next corner moving clockwise around the same face).
    pub fn prev(&self) -> Self {
        let size = self.loop_size();
        self.mesh.corner(self.raw_corner.face, (self.raw_corner.vert + size - 1) % size)
    }

    /// Get the next corner rotating counter-clockwise around this vertex.
    pub fn left(&self) -> Self {
        Self::new(self.mesh, self.mesh.left_corners[self.corner_index])
    }

    /// Get the next corner rotating clockwise around this vertex.
    pub fn right(&self) -> Self {
        Self::new(self.mesh, self.mesh.right_corners[self.corner_index])
    }

    fn loop_size(&self) -> usize {
        self.mesh.face_loops[self.raw_corner.face].len()
    }
}

pub struct MeshFace<'m> {
    mesh: &'m Mesh,
    index: usize,
}

impl<'a> MeshFace<'a> {
    pub fn plane(&self) -> geom::Plane {
        geom::Plane::new(self.centroid(), self.normal())
    }

    pub fn corner(&self, index: usize) -> Corner {
        self.mesh.corner(self.index, index)
    }

    pub fn contains(&self, pt: Pt3) -> bool {
        let normal = self.normal();
        let centroid = self.centroid();
        for edge in self.edges() {
            let v = edge.vector();
            let edge_normal = v ^ normal;
            let pt_side = edge_normal * (pt - edge.src) >= 0.0;
            let centroid_side = edge_normal * (centroid - edge.src) >= 0.0;
            if pt_side != centroid_side {
                return false;
            }
        }
        true
    }

    pub fn points(&self, resolution: f64) -> FacePointIter {
        let basis = Basis3::from_normal(self.normal());
        let frame = Frame3::new(self.centroid(), basis);

        let mut min = (None, None);
        let mut max = (None, None);
        for i in 0..self.vertex_count() {
            let pt = self.vertex(i);
            let local = frame.project(pt);
            if min.0.is_none() || min.0.unwrap() < local.i {
                min.0 = Some(local.i);
            }
            if max.0.is_none() || max.0.unwrap() > local.i {
                max.0 = Some(local.i);
            }
            if min.1.is_none() || min.1.unwrap() < local.j {
                min.1 = Some(local.j);
            }
            if max.1.is_none() || max.1.unwrap() > local.j {
                max.1 = Some(local.j);
            }
        }

        let min = (min.0.unwrap(), min.1.unwrap());
        let max = (max.0.unwrap(), max.1.unwrap());

        let i_values = FloatRange::from_step_size(min.0, max.0, resolution).collect();
        let j_values = FloatRange::from_step_size(min.1, max.1, resolution).collect();

        FacePointIter {
            face: self,
            frame,
            i_values,
            j_values,
            index: 0,
        }
    }
}

impl<'a> geom::HasVertices for MeshFace<'a> {
    fn vertex(&self, index: usize) -> &Pt3 {
        &self.mesh.vertices[self.mesh.face_loops[self.index][index]]
    }

    fn vertex_count(&self) -> usize {
        self.mesh.face_loops[self.index].len()
    }

    fn edges(&self) -> geom::FaceEdgeIter {
        geom::FaceEdgeIter::new(self)
    }

    fn normal(&self) -> Vec3 {
        self.mesh.face_normal(self.index).expect("Face normal wasn't calculated").clone()
    }

    fn centroid(&self) -> Pt3 {
        self.mesh.face_centroid(self.index).expect("Face centroid wasn't calculated").clone()
    }
}

impl<'a> geom::Shape for MeshFace<'a> {
    fn raycast(&self, ray: &Ray3) -> Option<geom::RaycastHit> {
        let hit_on_plane = self.plane().raycast(ray);
        return match hit_on_plane {
            Some(hit) => {
                if self.contains(hit.point) {
                    Some(hit)
                } else {
                    None
                }
            }
            None => None,
        };
    }

    fn signed_distance(&self, pt: Pt3) -> f64 {
        let plane = self.plane();
        let sign = if plane.normal * (pt - plane.origin) >= 0.0 {
            1.0
        } else {
            -1.0
        };

        // If the projection of the point onto the plane of this polygon is contained within the
        // boundaries of this polygon, that projection will be the closest point.
        let projection = self.plane().project(pt);
        if self.contains(projection) {
            return sign * (projection - pt).mag();
        }

        // If we're outside the bounds of the polygon, our distance to it is the closest distance
        // to any of its edges.
        let mut distance = 0.0;
        for edge in self.edges() {
            let edge_distance = edge.distance(pt);
            if edge_distance < distance {
                distance = edge_distance;
            }
        }

        distance * sign
    }
}


pub struct MeshFaceIter<'a> {
    mesh: &'a Mesh,
    index: usize,
}

impl<'a> Iterator for MeshFaceIter<'a> {
    type Item = MeshFace<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let next = self.mesh.face(self.index);
        self.index += 1;
        next
    }
}

pub struct FacePointIter<'a> {
    face: &'a MeshFace<'a>,
    frame: Frame3,
    i_values: Vec<f64>,
    j_values: Vec<f64>,
    index: usize,
}

impl<'a> Iterator for FacePointIter<'a> {
    type Item = Pt3;

    fn next(&mut self) -> Option<Self::Item> {
        let i_index = self.index / self.i_values.len();
        let j_index = self.index % self.i_values.len();
        self.index += 1;
        if i_index >= self.i_values.len() || j_index >= self.j_values.len() {
            return None;
        }
        let local = self
            .frame
            .local(self.i_values[i_index], self.j_values[j_index], 0.);
        Some(self.frame.unproject(&local))
    }
}

#[derive(Clone)]
pub struct MeshBuilder {
    vertices: Vec<Pt3>,
    face_loops: Vec<Vec<usize>>,
    source_file: Option<String>,
    dedup_dist: f64,
}

impl MeshBuilder {
    pub fn new() -> Self {
        Self {
            vertices: vec![],
            face_loops: vec![],
            source_file: None,
            dedup_dist: std::f32::MIN_POSITIVE as f64,
        }
    }

    pub fn dedup_threshold(&mut self, threshold: f64) {
        self.dedup_dist = threshold;
    }

    pub fn source_file(&mut self, path: String) {
        self.source_file = Some(path);
    }

    pub fn add_vertex(&mut self, pt: Pt3) -> usize {
        let index = self.vertices.len();
        self.vertices.push(pt);
        index
    }

    pub fn add_vertex_dedup(&mut self, pt: Pt3) -> usize {
        let mut delta = Pt3::zero();
        for (i, v) in self.vertices.iter().enumerate() {
            delta.set(&pt);
            delta.x -= v.x;
            delta.y -= v.y;
            delta.z -= v.z;

            if delta.x.abs() < self.dedup_dist
                && delta.y.abs() < self.dedup_dist
                && delta.z.abs() < self.dedup_dist {
                return i;
            }
        }
        self.add_vertex(pt)
    }

    pub fn add_face(&mut self, vertices: Vec<usize>) {
        self.face_loops.push(vertices);
    }

    pub fn build(self) -> Mesh {
        Mesh::new(self.vertices, self.face_loops, self.source_file)
    }
}
