extends Button

@onready var parent = get_parent()

func _ready():
	self.pressed.connect(self._button_pressed)
	self.pressed.connect(parent._take_snapshot)
	
func _button_pressed():
	print("take snapshot")
