use godot::prelude::*;
use godot::engine::{MultiMeshInstance2D, QuadMesh, MultiMesh};

use crate::occupancy_map::*;

#[derive(GodotClass)]
#[class(base=Node2D)]
struct OccupancyMapVisualizer {
    oc_map: Option<OcMap>,
    tile_size: f32,
    mmi: Option<Gd<MultiMeshInstance2D>>,

    #[export]
    dim_x: u32,

    #[export]
    dim_y: u32,

    #[base]
    base: Base<Node2D>
}

#[godot_api] // needed to export variables
impl OccupancyMapVisualizer {
    pub fn update_map_states() {
        todo!();
    }
}

#[godot_api]
impl INode2D for OccupancyMapVisualizer {
    fn init(base: Base<Node2D>) -> Self {
        OccupancyMapVisualizer{
            oc_map: None,
            tile_size: 3.0,
            mmi: None,
            dim_x: 65,
            dim_y: 65,
            base
        }
    }

    fn ready(&mut self) {
        godot_print!("creating the occupancy map visualization");
        godot_print!("{:?}", self.base.get_children());

        self.oc_map = Some(OcMap::new(self.dim_x, self.dim_y));

        let mut mmi: Gd<MultiMeshInstance2D> = MultiMeshInstance2D::new_alloc();

        let mut quad: Gd<QuadMesh> = QuadMesh::new();
        (*quad).set_size(Vector2::new(self.tile_size as f32, self.tile_size as f32));

        let mut multimesh: Gd<MultiMesh> = MultiMesh::new();
        (*multimesh).set_instance_count(0); // attempted debug
        (*multimesh).set_mesh(quad.upcast());
        (*multimesh).set_use_colors(true);

        let dim_x = self.dim_x as i32;
        let dim_y = self.dim_y as i32;
        let tile_count = (dim_x * dim_y) as i32;
        (*multimesh).set_instance_count(tile_count);
        (*multimesh).set_visible_instance_count(tile_count);

        for i in 0..tile_count {
            let x: f32 = (i%dim_x) as f32 * self.tile_size + self.tile_size / 2.0;
            let y: f32 = f32::floor((i/dim_x) as f32) * self.tile_size + self.tile_size / 2.0;
            let tf = Transform2D::from_angle_origin(std::f32::consts::PI, Vector2::new(x,y));
            (*multimesh).set_instance_transform_2d(i, tf);
            (*multimesh).set_instance_color(i, match self.oc_map.as_ref().unwrap().tile_states[i as usize] {
                TileState::Unknown => Color::from_rgba(0.6, 0.6, 0.6, 1.),
                TileState::Occupied => Color::from_rgba(0.05, 0.05, 0.05, 1.),
                TileState::Free => Color::from_rgba(0.95, 0.95, 0.95, 1.)
            });

            // if i%3 == 0 {
            //     (*multimesh).set_instance_color(i, Color::from_rgba(1., 0., 0., 1.));
            // } else {
            //     (*multimesh).set_instance_color(i, Color::from_rgba(0., 1., 0., 1.));
            // }
        }

        (*mmi).set_multimesh(multimesh);
        self.mmi = Some(mmi.clone());
        self.base.add_child(mmi.clone().upcast());
        
        mmi.set_owner(self.base.clone().upcast());
        godot_print!("done");
    }

}

