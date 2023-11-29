extends Button

@onready var parent = get_parent()

func _ready():
	self.pressed.connect(self._button_pressed)
	self.pressed.connect(parent._compare_consecutive_snapshots)
	
func _button_pressed():
	pass
