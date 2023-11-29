@tool
extends Node2D

@onready var mm = $grid_map_test
@onready var background = $background
@onready var tile_material = preload("res://assets/grid_tile_material.tres")

@export var tile_x: int = 5
@export var tile_y: int = 5
@export var tile_size: int = 60

func _ready():
	var quad = QuadMesh.new()
	quad.size = Vector2(tile_size, tile_size)
	#quad.material = tile_material
	
	mm.multimesh.instance_count = 0
	mm.multimesh.mesh = quad
	mm.multimesh.use_colors = true
	mm.multimesh.instance_count = tile_x * tile_y
	
	for i in mm.multimesh.instance_count:
		var x = (i%tile_x)*tile_size + tile_size / 2
		var y = floor(i/tile_y)*tile_size + tile_size / 2
		mm.multimesh.set_instance_transform_2d(i, Transform2D(PI, Vector2(x, y)))
		mm.multimesh.set_instance_color(i, Color(0,1,0,1))
		
	mm.multimesh.set_instance_color(10, Color(1,0,0,1))
