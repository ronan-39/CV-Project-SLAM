extern crate nalgebra as na;
use na::{SMatrix, SVector, Vector2, Matrix2, Vector3, Matrix3, Matrix2x3};
// use std::f32::{sin, cos};
use lstsq::lstsq;

pub fn icp_point_to_point_least_squares(source: &Vec<Vector2<f32>>, target: &Vec<Vector2<f32>>, n: usize) -> (Vec<Vec<Vector2<f32>>>, Vec<f32>, Vec<Vec<usize>>) {
    let mut pose = Vector3::<f32>::zeros();
    let mut chi_values = Vec::<f32>::new();
    let mut pose_values = vec![pose.clone()];
    let mut target_values = vec![target.clone()];
    let mut corresp_values = Vec::<Vec<usize>>::new();
    let mut target_copy: Vec<Vector2<f32>> = target.clone();

    for i in 0..n {
        let mut rot = r(pose[2]);
        let mut t: Vector2<f32> = pose.fixed_rows::<2>(0).into();
        let correspondences = get_correspondence_indices(&target_copy, source);
        corresp_values.push(correspondences);
        let (h, g, chi) = prepare_system(pose, target, source, corresp_values.last().unwrap());
        let dx: Vector3<f32> = lstsq(&h, &(-1.0 * g), 1e-14).unwrap().solution;
        pose += dx;
        pose[2] = pose[2].sin().atan2(pose[2].cos());
        chi_values.push(chi[(0,0)]);
        pose_values.push(pose.clone());
        
        rot = r(pose[2]);
        t = pose.fixed_rows::<2>(0).into();
        target_copy = target.clone().iter().map(|point| rot * point + t).collect();
        target_values.push(target_copy.clone()); // could remove the clone if i replace target_copy on line 17 with target_values.last().unwrap()
    }
    corresp_values.push(corresp_values.last().unwrap().clone());

    (target_values, chi_values, corresp_values)
}

pub fn icp_point_to_plane(source: &Vec<Vector2<f32>>, target: &Vec<Vector2<f32>>, n: usize
    ) -> (Vec<Vec<Vector2<f32>>>, Vec<f32>, Vec<Vec<usize>>) {
    let mut pose = Vector3::<f32>::zeros();
    let mut chi_values = Vec::<f32>::new();
    let mut pose_values = vec![pose.clone()];
    let mut target_values = vec![target.clone()];
    let mut target_copy: Vec<Vector2<f32>> = target.clone();
    let mut corresp_values = Vec::<Vec<usize>>::new();

    let source_normals = compute_normals(source, None).0;

    for i in 0..n {
        let mut rot = r(pose[2]);
        let mut t: Vector2<f32> = pose.fixed_rows::<2>(0).into();
        let correspondences = get_correspondence_indices(&target_copy, source);
        corresp_values.push(correspondences);
        let (h, g, chi) = prepare_system_normals(pose, target, source, corresp_values.last().unwrap(), &source_normals);
        let dx: Vector3<f32> = lstsq(&h, &(-1.0 * g), 1e-14).unwrap().solution;
        pose += dx;
        pose[2] = pose[2].sin().atan2(pose[2].cos());
        chi_values.push(chi);
        pose_values.push(pose.clone());

        rot = r(pose[2]);
        t = pose.fixed_rows::<2>(0).into();
        target_copy = target.clone().iter().map(|point| rot * point + t).collect();
        target_values.push(target_copy.clone()); // could remove the clone if i replace target_copy on line xx with target_values.last().unwrap()
    }
    corresp_values.push(corresp_values.last().unwrap().clone());

    (target_values, chi_values, corresp_values)
}

fn prepare_system_normals(pose: Vector3<f32>, target: &Vec<Vector2<f32>>, source: &Vec<Vector2<f32>>, correspondences: &Vec<usize>, source_normals: &Vec<Vector2<f32>>
    ) -> (Matrix3<f32>, Vector3<f32>, f32) {
    let mut h = Matrix3::<f32>::zeros();
    let mut g = Vector3::<f32>::zeros();
    // let mut chi = Matrix2::<f32>::zeros();
    let mut chi: f32 = 0.0;

    // println!("source_normals: {:?}", source_normals);

    for (i,j) in correspondences.iter().enumerate() {
        let p_point = target[i];
        let q_point = source[*j];
        let normal = source_normals[*j];
        let e: f32 = normal.dot(&error(pose, p_point, q_point));
        // let e = error(pose, p_point, q_point);
        let jcb = normal.transpose() * jacobian(pose, p_point); // dimensions dont match
        // let jcb: Vector3<f32> = jacobian(pose, p_point);
        h += jcb.transpose() * jcb;
        g += jcb.transpose() * e;
        // println!("{:?}th e: {:?}", i, e);
        // println!("{:?}th normal: {:?}", i, normal);
        // println!("{:?}th j: {:?}", i, j);
        chi += e * e;
    }
    (h, g, chi)
}

fn prepare_system(pose: Vector3<f32>, target: &Vec<Vector2<f32>>, source: &Vec<Vector2<f32>>, correspondences: &Vec<usize>) -> (Matrix3<f32>, Vector3<f32>, Matrix2<f32>) {
    let mut h = Matrix3::<f32>::zeros();
    let mut g = Vector3::<f32>::zeros();
    let mut chi = Matrix2::<f32>::zeros();

    for (i,j) in correspondences.iter().enumerate() {
        let p_point = target[i];
        let q_point = source[*j];
        let e = error(pose, p_point, q_point);
        let weight = 1.0;
        let jcb = jacobian(pose, p_point);
        h += jcb.transpose() * jcb;
        g += jcb.transpose() * e;
        chi += e * e.transpose();
    }
    (h, g, chi)
}

// before merging, note the change in order and variables in the function name
fn get_correspondence_indices(target: &Vec<Vector2<f32>>, source: &Vec<Vector2<f32>>) -> Vec<usize> {
    let mut closest_point_idx: Vec<usize> = Vec::new();
    let mut distance_buffer: Vec<f32> = Vec::new();

    for p2 in target {
        distance_buffer.clear();
        for p1 in source {
            distance_buffer.push((p2-p1).magnitude())
        }
        closest_point_idx.push(min_index(&distance_buffer));
    }

    closest_point_idx
}

fn min_index(arr: &Vec<f32>) -> usize {
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

fn r(theta: f32) -> Matrix2<f32> {
    Matrix2::new(theta.cos(), -theta.sin(),
                 theta.sin(),  theta.cos())
}

fn d_r(theta: f32) -> Matrix2<f32> {
    Matrix2::new(-theta.sin(), -theta.cos(),
                  theta.cos(), -theta.sin())
}

fn jacobian(pose: Vector3<f32>, point: Vector2<f32>) -> Matrix2x3<f32> {
    let theta = pose[2];
    let mut j = SMatrix::<f32,2,3>::zeros();
    
    j[(0,0)] = 1.0;
    j[(1,1)] = 1.0;
    j.set_column(2, &(d_r(theta) * point));

    j.into()
}

fn error(pose: Vector3<f32>, target_point: Vector2<f32>, source_point: Vector2<f32>) -> Vector2<f32> {
    let rotation = r(pose[2]);
    let translation: Vector2<f32> = pose.fixed_rows::<2>(0).into();
    let prediction = rotation * target_point + translation;

    prediction - source_point
}

pub fn compute_normals(points: &Vec<Vector2<f32>>, step: Option<usize>) -> (Vec<Vector2<f32>>, Vec<Vector2<f32>>) {
    let step_ = match step {
        Some(u) => u,
        None => 1
    };
    let mut normals: Vec<Vector2::<f32>> = vec![Vector2::new(0., 0.,)];
    let mut normals_at_points = Vec::<Vector2<f32>>::new();

    for i in step_..points.len()-step_ {
        let prev_point = points[i-step_];
        let next_point = points[i+step_];
        let curr_point = points[i];
        let dx = next_point[0] - prev_point[0];
        let dy = next_point[1] - prev_point[1];
        let mut normal = Matrix2::new(0., 0., -dy, dx);
        normal /= normal.norm();
        normals.push(normal.fixed_rows::<1>(1).transpose());
        normals_at_points.push(normals.last().unwrap() + curr_point);
    }
    normals.push(normals[0].clone());

    (normals, normals_at_points)
}


// cargo test -- --nocapture to run and see prints
#[cfg(test)]
mod tests {
    use super::*;

    // #[test]
    fn it_works() {

        let source = vec![Vector2::new(0., 0.), Vector2::new(0., 1.), Vector2::new(0., 2.)];
        let target = vec![Vector2::new(0., 0.), Vector2::new(1., 0.), Vector2::new(2., 0.)];

        let (target_values, chi_values, corresp_values) = icp_point_to_point_least_squares(&source, &target, 1);
        // println!("chi values: {:?}", chi_values);
        // println!("target values: {:?}", target_values.last().unwrap());
        // println!("corresp values: {:?}", corresp_values.last().unwrap());
    }

    // #[test]
    fn test_error() {
        let pose = Vector3::<f32>::zeros();
        let source = Vector2::<f32>::new(1.,2.);
        let target = Vector2::<f32>::new(3.,4.);

        let error = error(pose, source, target);

        assert_eq!(error, Vector2::<f32>::new(-2., -2.));
    }

    // #[test]
    fn test_prepare() {
        let pose = Vector3::<f32>::zeros();
        let source = vec![Vector2::new(0., 0.), Vector2::new(0., 1.), Vector2::new(0., 2.)];
        let target = vec![Vector2::new(0., 0.), Vector2::new(1., 0.), Vector2::new(2., 0.)];

        let correspondences = get_correspondence_indices(&target, &source);

        // swap target and source
        let (h, g, chi) = prepare_system(pose, &source, &target, &correspondences);
        // println!("h: {:?}", h);
        // println!("g: {:?}", g);
        // println!("chi: {:?}", chi);
    }

    // #[test]
    fn test_r() {
        let r1 = r(0.0);
        let r2 = r(std::f32::consts::PI);
        let r3 = d_r(0.0);
        let r4 = d_r(std::f32::consts::PI);

        for (x,y) in r1.iter().zip(&Matrix2::<f32>::new(1., -0., 0., 1.)) {
            assert!((x-y).abs() < 1e-6);
        }
        
        for (x,y) in r2.iter().zip(&Matrix2::<f32>::new(-1., 0., 0., -1.)) {
            assert!((x-y).abs() < 1e-6);
        }

        for (x,y) in r3.iter().zip(&Matrix2::<f32>::new(0., -1., 1., 0.)) {
            assert!((x-y).abs() < 1e-6);
        }

        for (x,y) in r4.iter().zip(&Matrix2::<f32>::new(0., 1., -1., 0.)) {
            assert!((x-y).abs() < 1e-6);
        }
    }


    // #[test]
    fn test_point_to_point_least_squares() {
        let angle = std::f32::consts::PI / 4.0;
        let r_true = r(angle);
        let t_true = Vector2::<f32>::new(-2., 5.);

        let num_points = 30;
        let true_data: Vec<Vector2<f32>> = (0..num_points).map(|i| Vector2::<f32>::new(i as f32, 0.2*i as f32*(0.5*i as f32).sin())).collect();
        let moved_data: Vec<Vector2<f32>> = true_data.clone().iter().map(|v| r_true * v + t_true).collect();

        // manually confirmed p and q are correct
        let q = true_data;
        let p = moved_data;
        let x = Vector3::new(0., 0., 0.);

        let correspondences = get_correspondence_indices(&p, &q);
        
        let (p_values, chi_values, corresp_values) = icp_point_to_point_least_squares(&q, &p, 1);

        // correct
        // println!("chi values: {:?}", chi_values);
        
        //correct
        // println!("p values: {:?}", p_values.last().unwrap());
        
        // correct
        // println!("corresp values: {:?}", corresp_values.last().unwrap());
    }

    #[test]
    fn test_point_to_plane() {
        let angle = std::f32::consts::PI / 4.0;
        let r_true = r(angle);
        let t_true = Vector2::<f32>::new(-2., 5.);

        let num_points = 30;
        let true_data: Vec<Vector2<f32>> = (0..num_points).map(|i| Vector2::<f32>::new(i as f32, 0.2*i as f32*(0.5*i as f32).sin())).collect();
        let moved_data: Vec<Vector2<f32>> = true_data.clone().iter().map(|v| r_true * v + t_true).collect();

        // manually confirmed p and q are correct
        let q = true_data;
        let p = moved_data;
        let x = Vector3::new(0., 0., 0.);

        let correspondences = get_correspondence_indices(&p, &q);
        
        // confirmed compute_normals works
        let normals = compute_normals(&q, None).0;
        // println!("normals: {:?}", normals);

        let (h, g, chi) = prepare_system_normals(x, &p, &q, &correspondences, &normals);
        // println!("h: {:?}", h);

        let (p_values, chi_values, corresp_values) = icp_point_to_plane(&q, &p, 30);
        // correct
        // println!("chi values: {:?}", chi_values);

        // //
        // println!("p values: {:?}", p_values.last().unwrap());
        
        // //
        // println!("corresp values: {:?}", corresp_values.last().unwrap());
    }

}