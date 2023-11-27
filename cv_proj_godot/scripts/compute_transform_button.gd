extends Button

@onready var parent = get_parent()

func _ready():
	self.pressed.connect(self._button_pressed)
	self.pressed.connect(parent._iterate_icp)
	
func _button_pressed():
	print("compute transform")
