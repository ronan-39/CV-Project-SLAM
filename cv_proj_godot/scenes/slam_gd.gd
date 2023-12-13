extends Node

signal console_log(str)

@onready var environment = $"../simulation/test_env_1"
@onready var agent = environment.agent

@onready var icp = $"../Icp"
@onready var omv = $'../OccupancyMapVisualizer'
@onready var lidar_preview = $'../lidar_preview/lidar_preview'

var stepped_scans: bool = true
var scan_interval: int = 5 # frames between scans
var interval_tracker: int = 0

var snapshots_max_size = 10
var snapshots = []
var this_is_the_first_scan = true

var agent_pose = Vector3(0., 0., 0.,)
var previous_pose = Vector3(0., 0., 0.,)

func _ready():
	pass

func _physics_process(delta):
	if interval_tracker == scan_interval:
		interval_tracker = 0
		scan()
		lidar_preview.draw_points(snapshots[-1])
		
		# if it IS the first scan, then the pose is 0,0,0, which is already the default
		if !this_is_the_first_scan:
			#update_agent_pose()
			update_agent_pose_from_map()
		
		this_is_the_first_scan = false
			
		omv.update_map(snapshots[-1], agent_pose)
		previous_pose = Vector3(agent.transform.origin.z, -agent.transform.origin.x, -agent.rotation.y)
	
	if !stepped_scans:
		interval_tracker+=1
		
	omv.show_agent_location(world_to_grid(xz(agent.transform.origin)))
	#print(world_to_grid(Vector2(5., 0)))
	#omv.show_agent_location(world_to_grid(Vector2(5., 0.)))
	#omv.show_agent_location(Vector2(-5.0, 0.0))

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
		#snapshot.append((xz(c.get_collision_point()) - xz(agent_global_transform)).rotated(agent_rotation - PI/2))
		snapshot.append((xz(c.get_collision_point()) - xz(agent_global_transform)))
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
	var pose_estimate = previous_pose
	var map_pc = omv.get_pc_gd()
	var scan_pc: Array[Vector2] = []
	for s in snapshots[-1]:
		scan_pc.append(s + xy(pose_estimate))
	
	var transform = icp.icp_point_to_plane_transform(map_pc, scan_pc, 10)
	var corrected_points = icp.icp_point_to_plane(map_pc, scan_pc, 10)
	lidar_preview.draw_icp(map_pc, corrected_points)
	var tf = Vector2(-transform.y, transform.x)
	print("transform: ", tf)
	
	#agent_pose += Vector3(tf.x, tf.y, 0.)
	agent_pose += transform
	print("agent pose: ", agent_pose)


func xz(vector):
	return Vector2(vector.x, vector.z)

func xy(vector):
	return Vector2(vector.x, vector.y)

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
