extends Node3D

@onready var turtle_bot: Node3D = get_parent()

@export var enabled: bool = true
@export var speed: float = 0.5
@export var rotation_speed: float = 2

func _physics_process(delta):
	var input = Input.get_vector("a", "d", "s", "w")
	
	var velocity = input.y
	
	var rot = input.x
	
	# rotation takes predecence over translation
	if rot != 0:
		velocity = 0
		
	turtle_bot.rotation.y -= rot * rotation_speed * delta
	turtle_bot.transform.origin += Vector3(velocity * speed * delta, 0, 0).rotated(Vector3.UP, turtle_bot.rotation.y)
	
	#print(turle_bot.transform.origin)
