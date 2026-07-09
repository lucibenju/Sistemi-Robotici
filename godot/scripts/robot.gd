extends Node3D

var theRobot

# Object management
var objects_collected = 0
var object_node = null

func _ready():
	theRobot = $Body
	
	# Start position
	var start_marker = $"../ObjectManager/START"
	theRobot.global_position = start_marker.global_position
	theRobot.global_rotation.y = start_marker.global_rotation.y
	
	# Subscribe to pose updates from Python
	DDS.subscribe("X")
	DDS.subscribe("Y")
	DDS.subscribe("Theta")
	
	# Subscribe to pick/drop commands from Python
	DDS.publish("Pick", DDS.DDS_TYPE_FLOAT, 0.0)
	DDS.publish("Drop", DDS.DDS_TYPE_FLOAT, 0.0)

# _physics_process runs at a FIXED rate (Project Settings → Physics → Ticks Per Second)
# This decouples simulation from render framerate, keeping dt stable on the Python side.
func _physics_process(_delta):
	# Publish tick as a binary trigger (1.0 = "step ready").
	# Python reads this to know Godot has advanced one physics step.
	# Python uses its own fixed DT = 1/physics_ticks_per_second, NOT this _delta.
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, 1.0)
	
	# 1. Update pose from Python
	var x     = DDS.read("X")
	var y     = DDS.read("Y")
	var theta = DDS.read("Theta")
	
	if Engine.get_physics_frames() % 60 == 0:
		print("DDS values - X: ", x, " Y: ", y, " Theta: ", theta)
		print("Robot pos: ", theRobot.global_position)
	
	if (x != null) and (y != null) and (theta != null):
		# Map Python (x, y) to Godot (x, -z)
		theRobot.global_position.x = x
		theRobot.global_position.z = -y
		theRobot.global_rotation.y = theta
		
	# 2. Check for Pick command
	var pick_cmd = DDS.read("Pick")
	if pick_cmd != null and pick_cmd == 1.0:
		DDS.subscribed_vars["Pick"] = 0.0 # clear locally
		DDS.publish("Pick", DDS.DDS_TYPE_FLOAT, 0.0)  # tell Python
		_pick_nearest()
		
	# 3. Check for Drop command
	var drop_cmd = DDS.read("Drop")
	if drop_cmd != null and drop_cmd == 1.0:
		DDS.publish("Drop", DDS.DDS_TYPE_FLOAT, 0.0)
		
func _pick_nearest():
	var objects = get_tree().get_nodes_in_group("Targets")
	# Detection: check if robot is near any visible object
	for obj in objects:
		if not is_instance_valid(obj):
			continue
		if not obj.visible:
			continue
		var dx = theRobot.global_position.x - obj.global_position.x
		var dz = theRobot.global_position.z - obj.global_position.z
		var dist = sqrt(dx * dx + dz * dz)
		print("\ndist:", dist)
		if dist < 0.5:
			obj.visible = false
			obj.remove_from_group("Targets")
			print("Picked object at ", obj.global_position)
			break
