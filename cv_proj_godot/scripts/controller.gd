extends Node

@onready var environment = $'../simulation/test_env_1'
@onready var agent = environment.agent

@onready var lidar_preview = $'../lidar_preview/lidar_preview'
@onready var omv = $'../OccupancyMapVisualizer'
@onready var snapshot_omv = $'../SnapshotOMV'

@onready var logbox = $TextEdit
@onready var icp_gds = $icp_gds
@onready var icp = $'../Icp'

var snapshots: Array = []
var agent_pose := Vector3(0., 0., 0.,)
var previous_pose := Vector3(0., 0., 0.,)

func _ready():
	print(environment)
	print(agent)

func _take_snapshot():
	var agent_global_transform = agent.transform.origin
	var agent_rotation = agent.rotation.y
	var snapshot: Array[Vector2] = []
	var log = "Snapshot " + str(len(snapshots)) + ": "
	for c in agent.lidar.get_children():
		# weird bug where it collides at the origin. ignore these points
		if c.get_collision_point() == Vector3.ZERO:
			continue
		snapshot.append((xz(c.get_collision_point()) - xz(agent_global_transform)).rotated(agent_rotation - PI/2))
		#var scan_endpoint = xz(c.get_collision_point()) - xz(agent_global_transform)
		#snapshot.append(scan_endpoint)
		log += str(snapshot[-1])
		log += " "
	snapshots.append(snapshot)
	logbox.insert_line_at(0, log)
	
	lidar_preview.draw_points(snapshot)
	
	# todo why do i have to make the x negative and rotation negative?
	var robot_pose = Vector3(agent.transform.origin.z, -agent.transform.origin.x, -agent.rotation.y)
	
	#var robot_pose = Vector3(agent.transform.origin.z, agent.transform.origin.x, agent.rotation.y)
	omv.update_map(snapshot, robot_pose)
	#lidar_preview.draw_points(omv.get_pc_gd())
	#var pose_estimate = xz(agent.transform.origin).rotated(agent_rotation - PI/2)
	#var snapshot_at_best_guess = []
	#for p in snapshot:
		#snapshot_at_best_guess.append(p + pose_estimate)
	#lidar_preview.draw_icp(omv.get_pc_gd(), snapshot_at_best_guess)
	
	#snapshot_omv.clear_map()
	#snapshot_omv.update_map(snapshot, Vector3.ZERO)
	#snapshot_omv.match_to_map(omv)
	
	# debug variables to keep track of how the agent moves from scan to scan
	previous_pose = agent_pose
	agent_pose = robot_pose

func _on_match_scan_to_map_pressed():
	print("matching scan to map")
	var snapshot = snapshots[-1]
	var agent_rotation = agent.rotation.y
	
	#var pose_estimate = xz(agent.transform.origin).rotated(agent_rotation - PI/2)
	var random_offset = Vector2(randf_range(0, 0.2), randf_range(0, 0.2))
	var pose_estimate = agent_pose
	print("pose estimate: ", pose_estimate)
	# here, pose estimate is in grid space
	
	var snapshot_at_best_guess: Array[Vector2] = []
	for p in snapshot:
		snapshot_at_best_guess.append(xy(pose_estimate) + (p).rotated(-agent_rotation))
		#snapshot_at_best_guess.append(p.rotated(-agent_rotation) + Vector2(pose_estimate.x, pose_estimate.y))
	
	var oc_map_pc: Array[Vector2] = omv.get_pc_gd()
	
	#var corrected_points = icp.icp_point_to_point_least_squares(oc_map_pc, snapshot_at_best_guess, 10)
	var corrected_points = icp.icp_point_to_plane(oc_map_pc, snapshot_at_best_guess, 10)
	var transform = icp.icp_point_to_plane_transform(oc_map_pc, snapshot_at_best_guess, 7)
	lidar_preview.draw_icp(oc_map_pc, corrected_points)
	#lidar_preview.draw_icp(oc_map_pc, snapshot_at_best_guess)
	#lidar_preview.draw_icp(oc_map_pc, snapshots[-1])
	print("transform: ", transform)
	#print(transform + pose_estimate)
	#print("pose_estimate from match scan to map: ", pose_estimate)
	var new_pose_estimate = transform + pose_estimate

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
	var corrected_points_two = icp.icp_point_to_plane(snapshots[-2], snapshots[-1], 7)
	var transform = icp.icp_point_to_plane_transform(snapshots[-2], snapshots[-1], 7)
	#print(transform)
	var corrected_points: Array[Vector2] = []
	#for vec in snapshots[-1]:
		#corrected_points.append(transform * vec)
		
	snapshots[-1] = corrected_points_two # edit points in place, good to see the iteration happen
	lidar_preview.draw_icp(snapshots[-2], corrected_points_two)

	logbox.insert_line_at(0, "Computing the agent's transform between the last two LIDAR snapshots")
	
	var real_transformation = agent_pose - previous_pose
	var error = xy(real_transformation) - xy(transform)
	logbox.insert_line_at(0, "Actual translation between previous two scans: " + str(xy(real_transformation)))
	logbox.insert_line_at(0, "Estimated translation between previous two scans: " + str(xy(transform)))
	logbox.insert_line_at(0, "Translation error: " + str(error.length() / xy(real_transformation).length()))
	logbox.insert_line_at(0, "Angle error: " + str(rad_to_deg(real_transformation[2] - transform[2])) + " degrees")

func _compare_consecutive_snapshots():
	if len(snapshots) < 2:
		logbox.insert_line_at(0, "Cannot compute the transform with less than two snapshots")
		return
	lidar_preview.draw_icp(snapshots[-2], snapshots[-1])

func _show_normals():
	var normal_endpoints = icp.get_normals_to_draw(snapshots[-1], 1)
	lidar_preview.draw_normals(snapshots[-1], normal_endpoints)
	
func _clear_map():
	omv.clear_map()

func xz(vector):
	return Vector2(vector.x, vector.z)

func xy(vector):
	return Vector2(vector.x, vector.y)

func _on_slam_gd_console_log(str):
	logbox.insert_line_at(0, str)


func _on_set_agent_pos_pressed():
	var new_pose = Vector3(3.0, 0.0, 0.0)
	logbox.insert_line_at(0, "Setting agents position to " + str(Vector2(new_pose.x, new_pose.y)))
	agent.transform.origin.x = new_pose.x
	agent.transform.origin.y = new_pose.y
	agent.rotation.y = new_pose.z
	#_take_snapshot()

func grid_to_world(vec) -> Vector3:
	return Vector3(-vec.y, vec.x, -vec.z)

