
pub struct OcMap {
    pub tile_states: Vec<TileState>,
    pub dim_x: u32,
    pub dim_y: u32,
}

impl OcMap {
    pub fn new() -> Self {
        OcMap { 
            tile_states: vec![TileState::Unknown; 16*16],
            dim_x: 32,
            dim_y: 32,
        }
    }
}

#[derive(Clone, Copy)]
pub enum TileState {
    Unknown,
    Occupied,
    Free
}