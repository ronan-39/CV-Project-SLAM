shader_type canvas_item;

global uniform vec4 color_unknown;
global uniform vec4 color_free;
global uniform vec4 color_occupied;

varying vec4 tile_color;

void vertex() {
	// Called for every vertex the material is visible on.
	if (INSTANCE_CUSTOM.xy == vec2(0,0)) {
		tile_color = color_unknown;
	}
	
	if (INSTANCE_CUSTOM.xy == vec2(0,1)) {
		tile_color = color_free;
	}
	
	if (INSTANCE_CUSTOM.xy == vec2(1,0)) {
		tile_color = color_occupied;
	}
	
}

void fragment() {
	//COLOR = tile_color;
	COLOR = vec4(1,0,0,1);
}

