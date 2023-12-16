extends Button

@onready var player = $AnimationPlayer
@onready var stepped_scans = $'../controller/stepped_scans'
@onready var slam_gd = $'../SLAM_GD'

func _ready():
	self.pressed.connect(self._on_button_pressed)

func _on_button_pressed():
	print("running recording")
	stepped_scans.button_pressed = false
	slam_gd.store_values = true
	player.play("move")
	pass
	
func _press_w(s: bool):
	if s:
		Input.action_press("w")
	else:
		Input.action_release("w")

func _press_a(s: bool):
	if s:
		Input.action_press("a")
	else:
		Input.action_release("a")
		
func _press_s(s: bool):
	if s:
		Input.action_press("s")
	else:
		Input.action_release("s")
		
func _press_d(s: bool):
	if s:
		Input.action_press("d")
	else:
		Input.action_release("d")

func _close_game():
	#get_tree().quit()
	pass
