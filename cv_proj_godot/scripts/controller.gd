extends Node

@onready var environment = $'../simulation/test_env_1'
@onready var agent = environment.agent

@onready var lidar_preview = $'../lidar_preview/lidar_preview'

@onready var logbox = $TextEdit
@onready var icp_gds = $icp_gds
@onready var icp = $Icp

var snapshots: Array = []

func _ready():
	print(environment)
	print(agent)

func _take_snapshot():
	var agent_global_transform = agent.transform.origin
	var agent_rotation = agent.rotation.y
	var snapshot: Array[Vector2] = []
	var log = "Snapshot " + str(len(snapshots)) + ": "
	for c in agent.lidar.get_children():
		snapshot.append((xz(c.get_collision_point()) - xz(agent_global_transform)).rotated(agent_rotation - PI/2))
		log += str(snapshot[-1])
		log += " "
	snapshots.append(snapshot)
	logbox.insert_line_at(0, log)
	
	lidar_preview.draw_points(snapshot)

func _iterate_icp():
	if len(snapshots) < 2:
		logbox.insert_line_at(0, "Cannot compute the transform with less than two snapshots")
		return
	
	# uncomment to manually check icp running for some points
	var s2: Array[Vector2] = [Vector2(0., 0.), Vector2(1.,0.), Vector2(2.,0.)]
	var s1: Array[Vector2] = [Vector2(0., 0.), Vector2(0.,1.), Vector2(0.,2.)]
	#snapshots[-2] = s2
	#snapshots[-1] = s1
	
	#print(snapshots)
		
	#icp_gds.icp(snapshots[-2], snapshots[-1], 1)
	icp.foo()
	#var transform = icp.icp_svd(snapshots[-2], snapshots[-1], 1)
	#var transform = icp.icp_svd_iter(snapshots[-2], snapshots[-1], 1)
	var corrected_points_two = icp.icp_point_to_plane(snapshots[-2], snapshots[-1], 10)
	#print(transform)
	var corrected_points: Array[Vector2] = []
	#for vec in snapshots[-1]:
		#corrected_points.append(transform * vec)
		
	snapshots[-1] = corrected_points_two # edit points in place, good to see the iteration happen
	lidar_preview.draw_icp(snapshots[-2], corrected_points_two)

	logbox.insert_line_at(0, "Computing the agent's transform between the last two LIDAR snapshots")

func _compare_consecutive_snapshots():
	if len(snapshots) < 2:
		logbox.insert_line_at(0, "Cannot compute the transform with less than two snapshots")
		return
	lidar_preview.draw_icp(snapshots[-2], snapshots[-1])

func _show_normals():
	var normal_endpoints = icp.get_normals_to_draw(snapshots[-1], 1)
	lidar_preview.draw_normals(snapshots[-1], normal_endpoints)
	

func xz(vector):
	return Vector2(vector.x, vector.z)
