extends Label

@onready var env = $'../../simulation/test_env_1'
@onready var agent = env.agent

func _physics_process(delta):
	self.text = str(xz(agent.transform.origin)) + "\n" + str(agent.rotation.y)

func xz(vec):
	return Vector2(vec.x, vec.z)
