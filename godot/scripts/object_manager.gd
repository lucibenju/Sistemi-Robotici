extends Node3D

@onready var targets_parent = $Targets
@onready var robot_body = $"../Robot/Body"

var target_positions: Array[Vector3] = []
var objects = []


func _ready():
	update_target_positions()
	randomize()
	target_positions.shuffle()
	
	DDS.subscribe("Pick")
	DDS.subscribe("Drop")

	# Spawn 4 objects at random target positions
	for i in range(4):
		if i >= target_positions.size():
			break

		var mesh_instance = MeshInstance3D.new()
		var mesh = BoxMesh.new()
		mesh.size = Vector3(0.2, 0.2, 0.2)
		mesh_instance.mesh = mesh

		var material = StandardMaterial3D.new()
		material.albedo_color = Color(0.815, 0.094, 0.0, 1.0)
		mesh_instance.material_override = material

		mesh_instance.add_to_group("Targets")
		add_child(mesh_instance)
		mesh_instance.global_position = target_positions[i]
		objects.append(mesh_instance)

	print("Spawned ", objects.size(), " objects.")
	print(objects)
	for i in range(objects.size()):
		print(" Índice ", i, ": Nodo ", objects[i].name, " en la posición ", objects[i].global_position)


func update_target_positions():
	target_positions.clear()
	if targets_parent:
		for child in targets_parent.get_children():
			if child is Marker3D:
				target_positions.append(child.global_position)

func _physics_process(_delta):
	# Detection: check if robot is near any visible object
	var detected = 0.0
	for obj in objects:
		if not is_instance_valid(obj):
			continue
		if not obj.visible:
			continue
		var dx = robot_body.global_position.x - obj.global_position.x
		var dz = robot_body.global_position.z - obj.global_position.z
		var dist = sqrt(dx * dx + dz * dz)
		print("\ndist:", dist)
		if dist < 0.5:
			detected = 1.0
			break
			
	DDS.publish("ObjectDetected", DDS.DDS_TYPE_FLOAT, detected)
