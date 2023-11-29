use godot::prelude::*;

extern crate nalgebra as na;
use na::{SMatrix, SVector};

mod icp;
use icp::*;

mod occupancy_map;
mod occupancy_map_visualizer;
use occupancy_map_visualizer::*;

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension{}

#[derive(GodotClass)]
#[class(base=Node)]
struct Icp {
    #[base]
    base: Base<Node>
}

#[godot_api]
impl INode for Icp {
    fn init(base: Base<Node>) -> Self {
        Icp{base}
    }
}


#[godot_api]
impl Icp {
    #[func]
    fn foo(&mut self) -> u32 {
        godot_print!("from rust!");
        1
    }

    #[func]
    fn icp_svd(pc_1: Array<Vector2>, pc_2: Array<Vector2>, n: u32) -> Transform2D {

        let (center_of_source, source_centered) = center_data(&pc_1);
        let (center_of_target, target_centered) = center_data(&pc_2);

        let correspondences = get_correspondence_indices(&source_centered, &target_centered);

        let (cov, _) = cross_covariance(&source_centered, &target_centered, &correspondences);
        let svd = cov.svd(true, true); // this one requires lapack or openblas idk
        let u: na::SMatrix::<f32,2,2> = svd.u.unwrap();
        let v_t: na::SMatrix::<f32,2,2> = svd.v_t.unwrap();
        let r: SMatrix::<f32,2,2> = u * &v_t;
        
        let cos_na = SVector::<f32, 2>::from([center_of_source.x, center_of_source.y]);
        let cot_na = SVector::<f32, 2>::from([center_of_target.x, center_of_target.y]);

        let t = cos_na - r * cot_na;

        let mut transform_2d: Transform2D = Transform2D::IDENTITY;
        transform_2d.a = Vector2::new(r[(0,0)], r[(1,0)]);
        transform_2d.b = Vector2::new(r[(0,1)], r[(1,1)]);
        transform_2d.origin = Vector2::new(t[0], t[1]);
        
        transform_2d
    }

    #[func]
    fn icp_svd_iter(source_godot: Array<Vector2>, target_godot: Array<Vector2>, n: u32) -> Transform2D {
        let mut source = source_godot.iter_shared()
                                     .map(|v| SVector::<f32, 2>::new(v.x, v.y))
                                     .collect::<Vec<SVector::<f32,2>>>();

        let mut target = target_godot.iter_shared()
                                     .map(|v| SVector::<f32, 2>::new(v.x, v.y))
                                     .collect::<Vec<SVector::<f32,2>>>();

        // Q is true - pc_1/source
        // P is moved - pc_2/target

        let mut all_correspondences: Vec<Vec<usize>> = Vec::new();
        let mut norm_values: Vec<f32> = Vec::new();
        let mut target_results: Vec<SVector::<f32,2>> = Vec::new();
        let mut latest_target: Vec::<SVector::<f32,2>> = target;

        let mut accumulated_transformation = SMatrix::<f32,3,3>::identity();

        let (center_of_source, source_centered) = center_data_na(&source);

        for i in 0..n {
            let (center_of_target, target_centered) = center_data_na(&latest_target);

            let correspondences = get_correspondence_indices_na(&target_centered, &source_centered);
            all_correspondences.push(correspondences);

            let (cov, _) = cross_covariance_na(&source_centered, &target_centered, all_correspondences.last().unwrap());
            let svd = cov.svd(true, true); // this one requires lapack or openblas idk
            let u: SMatrix::<f32,2,2> = svd.u.unwrap();
            let v_t: SMatrix::<f32,2,2> = svd.v_t.unwrap();
            let r: SMatrix::<f32,2,2> = u * &v_t;
            let t: SMatrix::<f32,2,1> = center_of_source - r * center_of_target;

            // having so much difficulty just making a transformation matrix... fix this later please
            let mut transformation_int = SMatrix::<f32,2,3>::zeros();
            transformation_int.set_column(0, &r.column(0));
            transformation_int.set_column(1, &r.column(1));
            transformation_int.set_column(2, &t);

            let mut transformation = SMatrix::<f32,3,3>::identity();
            transformation.set_row(0, &transformation_int.row(0));
            transformation.set_row(1, &transformation_int.row(1));

            // accumulated_transformation = transformation * accumulated_transformation;
            accumulated_transformation *= transformation;

            // todo theres a more rust-y way to do this but i got compiler errors hahaaaaaa
            for i in 0..latest_target.len() {
                latest_target[i] = r * latest_target[i] + t;
            }

            let error: f32 = target_centered.iter()
                                            .zip(&source_centered)
                                            .map(|(i,j)| (i - j).magnitude()).sum::<f32>() / target_centered.len() as f32;

            godot_print!("iteration {:?} with error {:?}", i, error);

        }

        let mut transform_2d: Transform2D = Transform2D::IDENTITY;
        transform_2d.a = Vector2::new(accumulated_transformation[(0,0)], accumulated_transformation[(1,0)]);
        transform_2d.b = Vector2::new(accumulated_transformation[(0,1)], accumulated_transformation[(1,1)]);
        transform_2d.origin = Vector2::new(accumulated_transformation[(0,2)], accumulated_transformation[(1,2)]);
        
        transform_2d
    }

    #[func]
    fn icp_point_to_point_least_squares(source_godot: Array<Vector2>, target_godot: Array<Vector2>, n: u32) -> Array<Vector2> {
        let mut source = source_godot.iter_shared()
                                     .map(|v| SVector::<f32, 2>::new(v.x, v.y))
                                     .collect::<Vec<SVector::<f32,2>>>();

        let mut target = target_godot.iter_shared()
                                     .map(|v| SVector::<f32, 2>::new(v.x, v.y))
                                     .collect::<Vec<SVector::<f32,2>>>();

        let (target_values, chi_values, corresp_values) = icp::icp_point_to_point_least_squares(&source, &target, n as usize);
        let result: Array<Vector2> = target_values.last().unwrap().iter().map(|v| Vector2::new(v[0], v[1])).collect();

        result
    }

    #[func]
    fn icp_point_to_plane(source_godot: Array<Vector2>, target_godot: Array<Vector2>, n: u32) -> Array<Vector2> {
        let mut source = source_godot.iter_shared()
        .map(|v| SVector::<f32, 2>::new(v.x, v.y))
        .collect::<Vec<SVector::<f32,2>>>();

        let mut target = target_godot.iter_shared()
                .map(|v| SVector::<f32, 2>::new(v.x, v.y))
                .collect::<Vec<SVector::<f32,2>>>();

        let (target_values, chi_values, corresp_values) = icp::icp_point_to_plane(&source, &target, n as usize);
        let result: Array<Vector2> = target_values.last().unwrap().iter().map(|v| Vector2::new(v[0], v[1])).collect();

        result
    }

    #[func]
    fn get_normals_to_draw(arr: Array<Vector2>, step: u32) -> Array<Vector2> {
        let points = arr.iter_shared()
                        .map(|v| SVector::<f32, 2>::new(v.x, v.y))
                        .collect::<Vec<SVector::<f32,2>>>();

        let (_, normals_at_points) = compute_normals(&points, None);
        let godot_normals: Array<Vector2> = normals_at_points.iter().map(|v| Vector2::new(v[0], v[1])).collect();

        godot_normals
    }

    #[func]
    fn test(arr: Array<Vector2>) {
        godot_print!("print from rust!");
    }
}

fn get_correspondence_indices(pc_1: &Array<Vector2>, pc_2: &Array<Vector2>) -> Array<u32> {
    let mut closest_point_idx: Array<u32> = Array::new();
    let mut distance_buffer: Array<f32> = Array::new();

    for p2 in pc_2.iter_shared() {
        distance_buffer.clear();
        for p1 in pc_1.iter_shared() {
            distance_buffer.push((p2-p1).length())
        }
        closest_point_idx.push(min_index(&distance_buffer));
    }

    closest_point_idx
}

fn get_correspondence_indices_na(pc_1: &Vec<SVector<f32,2>>, pc_2: &Vec<SVector<f32,2>>) -> Vec<usize> {
    let mut closest_point_idx: Vec<usize> = Vec::new();
    let mut distance_buffer: Vec<f32> = Vec::new();

    for p2 in pc_2 {
        distance_buffer.clear();
        for p1 in pc_1 {
            distance_buffer.push((p2-p1).magnitude())
        }
        closest_point_idx.push(min_index_na(&distance_buffer));
    }

    closest_point_idx
}

fn min_index(arr: &Array<f32>) -> u32 {
    let mut max = f32::INFINITY;
    let mut idx: usize = 0;
    for (i, val) in arr.iter_shared().enumerate() {
        if val < max {
            max = val;
            idx = i;
        }
    }
    idx as u32
}

// not na but im using it with the functions fixed to use na
fn min_index_na(arr: &Vec<f32>) -> usize {
    let mut max: f32 = f32::INFINITY;
    let mut idx: usize = 0;
    for (i, val) in arr.iter().enumerate() {
        if val < &max {
            max = *val;
            idx = i;
        }
    }
    idx
}

fn center_data(pc: &Array<Vector2>) -> (Vector2, Array<Vector2>) {
    let mut x_sum: f32 = 0.;
    let mut y_sum: f32 = 0.;

    for vec in pc.iter_shared() {
        x_sum += vec.x;
        y_sum += vec.y;
    }

    let len = pc.len();
    let center = Vector2::new(x_sum, y_sum) / len as f32;

    let center_data = pc.clone().iter_shared().map(|v| v-center).collect();

    (center, center_data)
}

fn center_data_na(pc: &Vec<SVector<f32,2>>) -> (SVector<f32,2>, Vec<SVector<f32,2>>) {
    let mut x_sum: f32 = 0.;
    let mut y_sum: f32 = 0.;

    for vec in pc.iter() {
        x_sum += vec.x;
        y_sum += vec.y;
    }

    let len = pc.len();
    let center = na::Vector2::new(x_sum, y_sum) / len as f32;

    let center_data = pc.clone().iter().map(|v| v-center).collect();

    (center, center_data)
}

fn cross_covariance(pc_1: &Array<Vector2>, pc_2: &Array<Vector2>, correspondences: &Array<u32>) -> (SMatrix::<f32,2,2>, Vec<usize>) {
    // let mut cov = NdArray2::<f32>::zeros((2,2).f());
    let mut cov = na::SMatrix::<f32,2,2>::zeros();
    let mut excluded_idx: Vec<usize> = Vec::new();

    for (i, vec) in pc_1.iter_shared().enumerate() {
        cov[(0,0)] += vec.x * pc_2.get(correspondences.get(i) as usize).x;
        cov[(0,1)] += vec.x * pc_2.get(correspondences.get(i) as usize).y;
        cov[(1,0)] += vec.y * pc_2.get(correspondences.get(i) as usize).x;
        cov[(1,1)] += vec.y * pc_2.get(correspondences.get(i) as usize).y;
    }

    (cov, excluded_idx)
}

fn cross_covariance_na(pc_1: &Vec<SVector<f32,2>>, pc_2: &Vec<SVector<f32,2>>, correspondences: &Vec<usize>) -> (SMatrix::<f32,2,2>, Vec<usize>) {
    let mut cov = na::SMatrix::<f32,2,2>::zeros();
    let mut excluded_idx: Vec<usize> = Vec::new();

    for (i, vec) in pc_1.iter().enumerate() {
        cov[(0,0)] += vec[0] * pc_2[correspondences[i]][0];
        cov[(0,1)] += vec[0] * pc_2[correspondences[i]][1];
        cov[(1,0)] += vec[1] * pc_2[correspondences[i]][0];
        cov[(1,1)] += vec[1] * pc_2[correspondences[i]][1];
    }

    (cov, excluded_idx)
}



// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn it_works() {

//         assert_eq!(1,1);
//     }
// }
