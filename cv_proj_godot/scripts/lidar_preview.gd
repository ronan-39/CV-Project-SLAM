extends Node2D

@onready var color_rect = $ColorRect
@export var zoom: float = 15.0

var to_draw = []
var icp_1 = []
var icp_2 = []
var normals = []

func _draw():
	for point in to_draw:
		draw_circle(point, 3.0, Color.DARK_RED)
		
	for p in icp_1:
		draw_circle(p, 3.0, Color.DARK_BLUE)
	
	for p in icp_2:
		draw_circle(p, 3.0, Color.DARK_GREEN)
		
	for n in normals:
		draw_line(n[0], n[1], Color.AQUA)
	
	draw_circle(color_rect.size / 2.0, 2.0, Color.BLACK)

func draw_points(snapshot):
	clear_buffers()
	
	var to_origin = color_rect.size / 2.0

	for point in snapshot:
		var fixed_point = point
		to_draw.append(fixed_point * zoom + to_origin)
		
	queue_redraw()

func draw_icp(snapshot_1, snapshot_2):
	clear_buffers()
	
	var to_origin = color_rect.size / 2.0
	
	for point in snapshot_1:
		var fixed_point = point
		icp_1.append(fixed_point * zoom + to_origin)
	
	for point in snapshot_2:
		var fixed_point = point
		icp_2.append(fixed_point * zoom + to_origin)
		
	queue_redraw()

func draw_normals(snapshot, normal_endpoints):
	var to_origin = color_rect.size / 2.0
	var len = 1.0

	for i in range(snapshot.size()-2):
		var normal = normal_endpoints[i] / len
		var line = [snapshot[i+1] * zoom + to_origin, normal * zoom + to_origin]
		normals.append(line)
		
	queue_redraw()
	
	
func clear_buffers():
	to_draw = []
	icp_1 = []
	icp_2 = []
	normals = []
