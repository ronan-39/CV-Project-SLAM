use na::{SMatrix, SVector, Vector2, Matrix2, Vector3, Matrix3, Matrix2x3, Transform2, IsometryMatrix2};

use crate::occupancy_map_visualizer;
use bresenham::Bresenham;

pub struct OcMap {
    pub tile_states: Vec<TileState>,
    pub updated_indices: Vec<usize>,
    pub has_update: bool,
    pub dim_x: u32,
    pub dim_y: u32,
    pub scale: f32,
}

impl OcMap {
    pub fn new(dim_x: u32, dim_y: u32) -> Self {
        OcMap { 
            tile_states: vec![TileState::Unknown; (dim_x*dim_y) as usize],
            updated_indices: Vec::new(),
            has_update: false,
            dim_x,
            dim_y,
            scale: 5.0,
        }
    }

    fn get_origin(&self) -> Vector2<u32> {
        Vector2::new(self.dim_x/2 as u32, self.dim_y/2 as u32)
    }

    fn get_origin_float(&self) -> Vector2<f32> {
        Vector2::new(self.dim_x as f32 / 2.0, self.dim_y as f32 / 2.0)
    }

    // get the robot pose in map coordinates
    fn get_map_coords_pose(&self, agent_pose_world: Vector3<f32>) -> Vector3<f32> {
        let translation = self.get_origin_float();
        // let scale: f32 = 5.0;
        let scale = self.scale;
        let rotation = Matrix2::<f32>::identity() * scale;

        let map_t_world = Matrix3::new(rotation[(0,0)], rotation[(0,1)], translation[0] as f32,
                                       rotation[(1,0)], rotation[(1,1)], translation[1] as f32,
                                                   0.0,             0.0,            1.0);


        let map_coords: Vector2<f32> = Vector2::new(agent_pose_world[0], agent_pose_world[1]) * scale + translation;

        Vector3::new(map_coords[0], map_coords[1], agent_pose_world[2])
    }

    pub fn update_by_scan(&mut self, scan_endpoints: Vec<Vector2<f32>>, robot_pose_world: Vector3<f32>) {
        // let scale: f32 = 5.0; // keep this consistent with above. move scale to be a property of the ocMap struct
        let scale = self.scale;

        // pose of the robot in map coordinates
        let map_pose = self.get_map_coords_pose(robot_pose_world);

        // world_t_robot
        // a homogenous transform matrix that will transform any point in robot space to world space
        // let r = IsometryMatrix2::rotation(map_pose[2]);
        let r = r(map_pose[2]) * scale;
        let pose_transform_pre = Matrix3::new(r[(0,0)], r[(0,1)], map_pose[0],
                                              r[(1,0)], r[(1,1)], map_pose[1],
                                                   0.0,      0.0,         1.0);
        // im slightly doubtful this worked, it was almost too easy of a bandaid fix
        //yes. it doesnt apply the translation to vectors. do transform_point(&vec2.into()) to get the full transformation
        let pose_transform = Transform2::from_matrix_unchecked(pose_transform_pre);

        // i know that the lidar scans are coming from the origin of the robot, so im cheating here (should multiply robot laser origin by pose transform)
        let scan_origin_f = Vector2::new(map_pose[0], map_pose[1]);

        let scan_origin_i = Vector2::<u32>::new((map_pose[0] + 0.5) as u32, (map_pose[1] + 0.5) as u32);

        for scan in scan_endpoints {
            let scan_end_map = pose_transform.transform_point(&scan.into());

            let scan_end_map_i = Vector2::<u32>::new((scan_end_map[0] + 0.5) as u32, (scan_end_map[1] + 0.5) as u32);

            // rename the variables so this looks like it makes sense
            self.update_line_bresenhams(&scan_origin_i, &scan_end_map_i);
        }
        
        self.has_update = true;
    }

    fn update_line_bresenhams(&mut self, start_map: &Vector2<u32>, end_map: &Vector2<u32>) {
        // let max_len: u32 = 500;

        let start_tuple = (start_map[0] as isize, start_map[1] as isize);
        let end_tuple = (end_map[0] as isize, end_map[1] as isize);

        let mut last_index: Option<usize> = None;

        for (x,y) in Bresenham::new(start_tuple, end_tuple) {
            if x >= self.dim_x as isize || y < 0 {
                continue;
            }

            if y >= self.dim_y as isize || y < 0 {
                continue;
            }

            let index = (y as u32 *self.dim_x + x as u32) as usize;
            last_index = Some(index);
            if index > self.tile_states.len() - 1 {
                last_index = None;
                continue
            }

            // todo combine these into a match statement
            if let TileState::Occupied = self.tile_states[index] {
                return;
            }

            if let TileState::Free = self.tile_states[index] {
                continue;
            } else {
                self.tile_states[index] = TileState::Free;
                self.updated_indices.push(index);
            }
        }

        if let Some(idx) = last_index {
            self.tile_states[idx] = TileState::Occupied;
        }

        self.has_update = true;
    }

    pub fn clear_map(&mut self) {
        self.updated_indices = Vec::new();
        // for state in &mut self.tile_states {
        //     *state = TileState::Unknown;
        // }
        self.has_update = true;
        for i in 0..self.tile_states.len() {
            self.updated_indices.push(i); // this is bad and dumb but uhh ill fix it later
            self.tile_states[i] = TileState::Unknown;
        }
    }

}


fn r(theta: f32) -> Matrix2<f32> {
    Matrix2::new(theta.cos(), -theta.sin(),
                 theta.sin(),  theta.cos())
}

#[derive(Clone, Copy)]
pub enum TileState {
    Unknown,
    Occupied,
    Free
}

// cargo test -- --nocapture to run and see prints
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_map_coords_pose() {
        let mut oc_map = OcMap::new(64, 64);
        oc_map.scale = 0.5;

        let world_pose_a = Vector3::new(0., 0., 0.);
        let world_pose_b = Vector3::new(10., 10., std::f32::consts::PI);
        let world_pose_c = Vector3::new(-5., -15., std::f32::consts::PI / 2.0);

        let map_pose_a = Vector3::new(32., 32., 0.);
        let map_pose_b = Vector3::new(37.0, 37.0, std::f32::consts::PI);
        let map_pose_c = Vector3::new(29.5, 24.5, std::f32::consts::PI / 2.0);

        // println!("{:?}", oc_map.get_map_coords_pose(world_pose_a));
        assert_eq!(oc_map.get_map_coords_pose(world_pose_a), map_pose_a);
        assert_eq!(oc_map.get_map_coords_pose(world_pose_b), map_pose_b);
    }

    #[test]
    fn full_test() {
        let grid_size: usize = 9;
        let mut oc_map = OcMap::new(grid_size as u32,grid_size as u32);

        let robot_pose = Vector3::<f32>::zeros();
        let scan_endpoints = vec![Vector2::new(7., 0.)];

        oc_map.update_by_scan(scan_endpoints, robot_pose);

        // for (i, state) in oc_map.tile_states.iter().enumerate() {
        //     if i%grid_size == 0 && i != 0  {
        //         println!("");
        //     }
        //     match state {
        //         TileState::Unknown => print!("X"),
        //         TileState::Free => print!("0"),
        //         TileState::Occupied => print!("1"),
        //     };
        // }
        // println!("");
    }

}