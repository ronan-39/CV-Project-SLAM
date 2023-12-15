extends Node

signal console_log(str)

@onready var environment = $"../simulation/test_env_1"
@onready var agent = environment.agent

@onready var icp = $"../Icp"
@onready var omv = $'../OccupancyMapVisualizer'
@onready var lidar_preview = $'../lidar_preview/lidar_preview'
@onready var lidar_preview_2 = $'../lidar_preview_2/lidar_preview'

var stepped_scans: bool = true
var scan_interval: int = 5 # frames between scans
var interval_tracker: int = 0

var snapshots_max_size = 10
var snapshots = []
var this_is_the_first_scan = true

var agent_pose = Vector3(0., 0., 0.,)
var previous_pose = Vector3(0., 0., 0.,)
var true_agent_pose = Vector3(0., 0., 0.,)

func _ready():
	pass

func _physics_process(delta):
	true_agent_pose = Vector3(agent.transform.origin.x, agent.transform.origin.z, -agent.rotation.y)
	
	if interval_tracker == scan_interval:
		interval_tracker = 0
		scan()
		#lidar_preview.draw_points(snapshots[-1])
		
		# if it IS the first scan, then the pose is 0,0,0, which is already the default
		previous_pose = agent_pose
		
		if !this_is_the_first_scan:
			#update_agent_pose()
			#update_agent_pose_cheating()
			update_agent_pose_from_map()
			omv.update_map(snapshots[-1], agent_pose)
		else:
			omv.update_map(snapshots[-1], Vector3(0., 0., 0.,))
			#omv.update_map(snapshots[-1], true_agent_pose)
			#var a_point: Array[Vector2] = [Vector2(3., 0), Vector2(0., 6.0)]
			#omv.update_map(a_point, agent_pose)
		
		this_is_the_first_scan = false
	
	if !stepped_scans:
		interval_tracker+=1
		
	#omv.show_agent_location(world_to_grid(xy(true_agent_pose)))
	omv.show_agent_location(xy(agent_pose))

func step():
	# make a scan be taken next time physics_process() runs
	interval_tracker = scan_interval
		
func scan():
	var agent_global_transform = agent.transform.origin
	var agent_rotation = agent.rotation.y
	var snapshot: Array[Vector2] = []
	var log = "Snapshot " + str(len(snapshots)) + ": "
	for c in agent.lidar.get_children():
		if c.get_collision_point() == Vector3.ZERO:
			continue # avoid bug where it collides with nothing at the origin
		snapshot.append((xz(c.get_collision_point()) - xz(agent_global_transform)).rotated(agent_rotation - PI/2))
		#snapshot.append((xz(c.get_collision_point()) - xz(agent_global_transform)))
		log += str(snapshot[-1])
		log += " "
	snapshots.append(snapshot)
	console_log.emit(log)
	
	# this should be made more efficient with some kind of cyclic buffer
	if snapshots.size() > snapshots_max_size:
		snapshots.pop_front()

func update_agent_pose():
	# cheating while testing
	#agent_pose = Vector3(agent.transform.origin.z, -agent.transform.origin.x, -agent.rotation.y)
	
	# use icp to find the transfrom from the previous two scans, then apply that transform to the stored pose
	var past_scans: Array[Vector2] = []
	for i in min(snapshots.size()-1, 5):
		print(-i-2)
		past_scans.append_array(snapshots[-i-2])
	
	var transform = icp.icp_point_to_plane_transform(past_scans, snapshots[-1], 10)
	#var transform = icp.icp_point_to_plane_transform(omv.get_pc_gd(), snapshots[-1], 7)
	agent_pose += transform
	
	print("what i think transform should be:")
	print(transform)
	
	print("what it is")
	print(Vector3(agent.transform.origin.z, -agent.transform.origin.x, -agent.rotation.y) - previous_pose)

func update_agent_pose_from_map():
	#console_log.emit("Updating agent pose from map:")
	var snapshot = snapshots[-1]
	var agent_rotation = agent.rotation.y
	var pose_estimate = agent_pose

	var snapshot_at_best_guess: Array[Vector2] = []
	for p in snapshot:
		snapshot_at_best_guess.append(xy(pose_estimate) + (p).rotated(-agent_rotation))

	var oc_map_pc: Array[Vector2] = omv.get_pc_gd()
	
	var use_point_to_plane: bool = false
	var corrected_points: Array[Vector2] = []
	var transform: Vector3 = Vector3.ZERO
	
	if use_point_to_plane:
		corrected_points = icp.icp_point_to_plane(oc_map_pc, snapshot_at_best_guess, 7)
		transform = icp.icp_point_to_plane_transform(oc_map_pc, snapshot_at_best_guess, 7)
	else:
		corrected_points = icp.icp_point_to_point_least_squares(oc_map_pc, snapshot_at_best_guess, 20)
		transform = icp.icp_point_to_point_transform(oc_map_pc, snapshot_at_best_guess, 20)
		
	lidar_preview.draw_icp(oc_map_pc, snapshot_at_best_guess)
	lidar_preview_2.draw_icp(oc_map_pc, corrected_points)
	#lidar_preview.draw_points(snapshots[-1])
	var new_pose_estimate_grid = transform + pose_estimate
	
	console_log.emit("Agent pose ground truth: " + str(Vector3(agent.transform.origin.z, -agent.transform.origin.x, -agent.rotation.y)))
	agent_pose = new_pose_estimate_grid
	
	# essentially, use a compass
	agent_pose.z = -agent.rotation.y
	#omv.update_map(corrected_points, Vector3(0., 0., 0.))

# for debug purposes, this function sets the pose to the ground truth value
func update_agent_pose_cheating():
	#agent_pose = Vector3(agent.transform.origin.x, agent.transform.origin.z, agent.rotation.y)
	agent_pose = Vector3(agent.transform.origin.z, -agent.transform.origin.x, -agent.rotation.y)
	console_log.emit("Setting the agent pose to the ground truth: " + str(agent_pose))

func xz(vector):
	return Vector2(vector.x, vector.z)

func xy(vector):
	return Vector2(vector.x, vector.y)

func world_pose_to_grid_pose(vec):
	return Vector3(vec.y, -vec.x, -vec.z)
	
func grid_to_world(vec) -> Vector3:
	return Vector3(-vec.y, vec.x, -vec.z)

func _on_stepped_scans_toggled(toggled_on):
	if toggled_on:
		interval_tracker = 0
		stepped_scans = true
	else:
		stepped_scans = false

func world_to_grid(vec):
	return Vector2(vec.y, -vec.x)

func _on_step_scan_pressed():
	step()
