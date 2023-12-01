extends Button

@onready var parent = get_parent()

func _ready():
	self.pressed.connect(self._button_pressed)
	self.pressed.connect(parent._clear_map)
	
func _button_pressed():
	pass
