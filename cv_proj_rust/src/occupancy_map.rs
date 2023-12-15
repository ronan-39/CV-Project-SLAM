use na::{SMatrix, SVector, Vector2, Matrix2, Vector3, Matrix3, Matrix2x3, Transform2, IsometryMatrix2, Vector4};
use bresenham::Bresenham;
use moka::sync::Cache;

use crate::occupancy_map_visualizer;

pub struct OcMap {
    pub tile_states: Vec<TileState>,
    pub updated_indices: Vec<usize>,
    pub has_update: bool,
    pub dim_x: u32,
    pub dim_y: u32,
    pub scale: f32,
    intensities: Vector4<f32>,
    cache: Cache<u32, f32>,
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
            intensities: Vector4::zeros(),
            cache: Cache::new(10_000),
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

    // take input in grid space
    pub fn set_agent_location(&mut self, pos: Vector2<f32>) {
        let map_coords = self.get_map_coords_pose(Vector3::new(pos.x, pos.y, 0.));
        let index = (map_coords.y as u32 * self.dim_x + map_coords.x as u32) as usize;
        if index as u32 >= self.dim_x * self.dim_y {
            return;
        }
        self.tile_states[index] = TileState::AgentPos;
        self.updated_indices.push(index);
    }

    fn update_line_bresenhams(&mut self, start_map: &Vector2<u32>, end_map: &Vector2<u32>) {
        // let max_len: u32 = 500;

        let start_tuple = (start_map[0] as isize, start_map[1] as isize);
        let end_tuple = (end_map[0] as isize, end_map[1] as isize);

        let mut last_index: Option<usize> = None;

        let line_coords: Vec<(isize, isize)> = Bresenham::new(start_tuple, end_tuple).collect();
        if line_coords.len() == 0 {
            return;
        }
        let last_xy = line_coords.last().unwrap();
        let last_index_known = (last_xy.1 as u32 *self.dim_x + last_xy.0 as u32) as usize;
        let len = line_coords.len();

        // for (x,y) in Bresenham::new(start_tuple, end_tuple) {
        for (i, (x,y)) in line_coords.iter().enumerate() {
            if x >= &(self.dim_x as isize) || y < &0 {
                continue;
            }

            if y >= &(self.dim_y as isize) || y < &0 {
                continue;
            }

            let index = (*y as u32 *self.dim_x + *x as u32) as usize;
            last_index = Some(index);
            if index > self.tile_states.len() - 1 {
                last_index = None;
                continue
            }

            // if index == last_index_known {
            //     break;
            // }

            // todo combine these into a match statement
            // if let TileState::Occupied = self.tile_states[index] {
            //     return;
            //     // break;
            // }

            // // if let TileState::Free = self.tile_states[index] {
            // //     continue;
            // // } else {
            // //     self.tile_states[index] = TileState::Free;
            // //     self.updated_indices.push(index);
            // // }

            // if let TileState::Unknown = self.tile_states[index] {
            //     self.tile_states[index] = TileState::Free;
            //     self.updated_indices.push(index);
            // }

            // try two things
            // probability of a free tile starts low and increase
            // probability of a free tile starts high and decreases when a tile is found there

            let overwrite_free = false;

            match self.tile_states[index] {
                TileState::Occupied => break,
                TileState::Free(p) => {
                    if overwrite_free && i==len-1 { self.tile_states[index] = TileState::Occupied; }
                //     // if i == len-1 { // if on the last tile of the line
                //     //     // if p < 0.5 { self.tile_states[index] = TileState::Occupied; }
                //     //     self.tile_states[index] = TileState::Occupied;
                //     // } else {
                //     //     self.tile_states[index] = TileState::Free((p+0.1).clamp(0.0, 1.0));
                //     // }
                //     continue;
                },
                TileState::Unknown => {
                    if i == len-1 { // if on the last tile of the line
                        self.tile_states[index] = TileState::Occupied;
                    } else {
                        self.tile_states[index] = TileState::Free(0.1);
                    }
                    self.updated_indices.push(index);
                },
                _ => continue,
            };
        }

        // if let Some(idx) = last_index {
        //     if let TileState::Unknown = self.tile_states[idx] {
        //         self.tile_states[idx] = TileState::Occupied;
        //         self.updated_indices.push(idx);
        //     }
        // }

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

    pub fn get_complete_hessian_derivs(&mut self, pose: Vector3<f32>, scan_endpoints: Vec<Vector2<f32>>, h: &mut Matrix3<f32>, d_t_r: &mut Vector3<f32>) {
        let size = scan_endpoints.len();

        let r = r(pose[2]);
        let transform_pre = Matrix3::new(r[(0,0)], r[(0,1)], pose[0],
                                     r[(1,0)], r[(1,1)], pose[1],
                                          0.0,      0.0,     1.0);
        let transform = Transform2::from_matrix_unchecked(transform_pre);

        let sin_rot = pose[2].sin();
        let cos_rot = pose[2].cos();

        *h = Matrix3::<f32>::zeros();
        *d_t_r = Vector3::<f32>::zeros();

        for curr_point in scan_endpoints {
            // let curr_point = &scan_endpoints[i];

            let transformed_point: na::Point<f32, 2> = transform.transform_point(&curr_point.into());
            let transformed_point_as_vec: Vector2<f32> = Vector2::new(transformed_point.x, transformed_point.y);
            let transformed_scan_endpoint = self.interp_map_value_with_derivs(transformed_point_as_vec);

            let fun_val: f32 = 1.0 - transformed_scan_endpoint[0];

            d_t_r[0] += transformed_scan_endpoint[1] * fun_val;
            d_t_r[1] += transformed_scan_endpoint[2] * fun_val; // why are they using the third element in the scan endpoints??
        }
    }

    fn my_compute_hessian(&self, pose: Vector3<f32>, scan_endpoints: Vec<Vector2<f32>>) -> () {
        let map_gradient = Matrix2x3::new(
            1.0, 0.0, -pose[2].sin() - pose[2].cos(),
            0.0, 1.0,  pose[2].cos() - pose[2].sin()
        );

        // let occ_value_gradient = interp_map_value_with_derivs()
        // let intermediate = 
    }

    fn interp_map_value_with_derivs(&mut self, coords: Vector2<f32>) -> Vector3<f32> {
        // bottom left corner of the index of the coordinate
        let scaled_coords = coords * 1.0;//self.scale;
        let ind_min = Vector2::new(scaled_coords[0] as u32, scaled_coords[1] as u32); 

        let factors: Vector2<f32> = scaled_coords - Vector2::new(ind_min[0] as f32, ind_min[1] as f32);

        let mut index = ind_min[1] * self.dim_x + ind_min[0];

        // get grid values for the 4 grid points surrounding the current coordinates
        // check the cache first
        // filter grid_point with gaussian and store in the cache (whatever that means)
        if self.cache.contains_key(&index) {
            self.intensities[0] = self.cache.get(&index).unwrap();
        } else {
            self.intensities[0] = self.get_grid_possibility_map(index);
            self.cache.insert(index, self.intensities[0]);
        }

        index+=1;

        if self.cache.contains_key(&index) {
            self.intensities[1] = self.cache.get(&index).unwrap();
        } else {
            self.intensities[1] = self.get_grid_possibility_map(index);
            self.cache.insert(index, self.intensities[1]);
        }

        index+=self.dim_x-1;

        if self.cache.contains_key(&index) {
            self.intensities[2] = self.cache.get(&index).unwrap();
        } else {
            self.intensities[2] = self.get_grid_possibility_map(index);
            self.cache.insert(index, self.intensities[2]);
        }

        index+=1;

        if self.cache.contains_key(&index) {
            self.intensities[3] = self.cache.get(&index).unwrap();
        } else {
            self.intensities[3] = self.get_grid_possibility_map(index);
            self.cache.insert(index, self.intensities[3]);
        }

        let dx1: f32 = self.intensities[0] - self.intensities[1];
        let dx2: f32 = self.intensities[2] - self.intensities[3];

        let dy1: f32 = self.intensities[0] - self.intensities[2];
        let dy2: f32 = self.intensities[1] - self.intensities[3];

        let x_fac_inv = 1.0 - factors[0];
        let y_fac_inv = 1.0 - factors[1];

        Vector3::<f32>::new(
            ((self.intensities[0] * x_fac_inv + self.intensities[1] * factors[0]) * y_fac_inv) + ((self.intensities[2] * x_fac_inv + self.intensities[3] * factors[0]) * factors[1]),
            -((dx1 * x_fac_inv) + (dx2 * factors[0])),
            -((dy1 * y_fac_inv) + (dy2 * factors[1]))
        )
    }

    fn get_unfiltered_grid_point(&self, coords: Vector2<u32>) -> f32 {
        return self.get_grid_possibility_map(coords.x + coords.y * self.dim_x);
    }

    fn get_grid_possibility_map(&self, index: u32) -> f32 {
        match self.tile_states[index as usize] {
            TileState::Unknown => 0.5,
            TileState::Occupied => 1.0,
            TileState::Free(_) => 0.0,
            TileState::Possible(x) => x,
            TileState::AgentPos => 0.0
        }
    }

    pub fn get_pc(&self) -> Vec<Vector2<f32>> {
        let pc: Vec<Vector2<f32>> = self.tile_states
            .iter()
            .enumerate()
            .filter(|(i,state)| matches!(state, TileState::Occupied))
            .map(|(i,state)|
            {
                let x: usize = i%self.dim_x as usize;
                let y: usize = i/self.dim_x as usize;
                Vector2::new(x as f32 - 50., y as f32 - 50.) / self.scale
            }
            ).collect();

            return pc;
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
    Free(f32),
    Possible(f32),
    AgentPos,
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