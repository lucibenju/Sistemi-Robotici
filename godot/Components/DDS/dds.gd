extends Node

const COMMAND_KEEP_ALIVE = 0x80
const COMMAND_SUBSCRIBE = 0x81
const COMMAND_PUBLISH = 0x82

const TIME_TO_LIVE = 2

var server_port = 4444
var udp_server : UDPServer
var udp_peers = []

var subscribers : Dictionary
var variables : Dictionary
var subscribed_vars : Dictionary

const DDS_TYPE_UNKNOWN = 0
const DDS_TYPE_INT = 1
const DDS_TYPE_FLOAT = 2

class DDSVariable:
	var name : String
	var type : int
	var value
	var peers
	var packet : PackedByteArray
	var value_offset : int

	func init(n : String):
		self.peers = []
		self.name = n
		self.type = DDS_TYPE_UNKNOWN
		self.value = 0
		self.value_offset = 0

	func add_peer(p : PacketPeerUDP):
		# Avoid adding the same peer twice
		if p not in self.peers:
			self.peers.append(p)

	func set_value(t : int, value):
		if t != self.type:
			self.type = t
			self.prepare_packet()
		self.value = value
		match self.type:
			DDS_TYPE_INT:
				self.packet.encode_s32(self.value_offset, int(value))
			DDS_TYPE_FLOAT:
				self.packet.encode_float(self.value_offset, float(value))

	func prepare_packet():
		self.packet = PackedByteArray()
		self.packet.resize(len(self.name) + 3 + 4)
		self.packet.encode_u8(0, COMMAND_PUBLISH)
		self.packet.encode_u8(1, self.type)
		self.packet.encode_u8(2, len(self.name))
		var i = 3
		for c in self.name.to_ascii_buffer():
			self.packet.encode_u8(i, c)
			i += 1
		self.value_offset = i

	func publish():
		for p : PacketPeerUDP in self.peers:
			p.put_packet(self.packet)

class SubscribedVarCollection:
	var var_list : Dictionary
	var ttl : float
	var peer : PacketPeerUDP

	func set_var_list(v : Dictionary):
		self.var_list = v

	func init(peer : PacketPeerUDP):
		self.var_list = {}
		self.ttl = 0
		self.peer = peer

	func process(delta : float):
		self.ttl += delta
		return self.ttl > TIME_TO_LIVE

	func keep_alive():
		self.ttl = 0


func _ready() -> void:
	udp_server = UDPServer.new()
	udp_server.listen(server_port)
	subscribers = {}
	variables = {}
	subscribed_vars = {}
	print("DDS ready, listening on port ", server_port)


func _process(delta: float) -> void:
	for k in subscribers.keys():
		if subscribers[k].process(delta):
			subscribers.erase(k)
	udp_server.poll()
	if udp_server.is_connection_available():
		var peer : PacketPeerUDP = udp_server.take_connection()
		udp_peers.append(peer)

	for peer in udp_peers:
		while peer.get_available_packet_count() > 0:
			var var_collection : SubscribedVarCollection
			if subscribers.get(peer) == null:
				var_collection = SubscribedVarCollection.new()
				var_collection.init(peer)
				subscribers[peer] = var_collection
			else:
				var_collection = subscribers[peer]
				
			var packet = peer.get_packet()
			var command = packet.decode_u8(0)
			match command:
				COMMAND_KEEP_ALIVE:
					var_collection.keep_alive()
				COMMAND_SUBSCRIBE:
					subscribe_from_remote(peer, packet)
				COMMAND_PUBLISH:
					publish_from_remote(peer, packet)


func subscribe_from_remote(peer : PacketPeerUDP, packet : PackedByteArray):
	var number_of_vars = packet.decode_u8(1)
	var index = 2
	var peer_var = {}
	for i in range(number_of_vars):
		var slen = packet.decode_u8(index)
		var subpacket = packet.slice(index + 1, index + slen + 1)
		var name : String = subpacket.get_string_from_utf8()
		var variable
		if variables.get(name) == null:
			variable = DDSVariable.new()
			variable.init(name)
			variables[name] = variable
		else:
			variable = variables[name]
		variable.add_peer(peer)
		peer_var[name] = variable
		# Send current value immediately to new subscriber
		if variable.type != DDS_TYPE_UNKNOWN:
			peer.put_packet(variable.packet)
		index += (slen + 1)
	subscribers[peer].set_var_list(peer_var)


func publish_from_remote(peer : PacketPeerUDP, packet : PackedByteArray):
	var typ = packet.decode_u8(1)
	var slen = packet.decode_u8(2)
	var subpacket = packet.slice(3, slen + 3)
	var name : String = subpacket.get_string_from_utf8()
	var value = 0.0
	match typ:
		DDS_TYPE_INT:
			value = packet.decode_s32(slen + 3)
		DDS_TYPE_FLOAT:
			value = packet.decode_float(slen + 3)
	subscribed_vars[name] = value


func publish(name : String, type : int, value):
	var item
	if variables.get(name) == null:
		# Always create the variable, even without subscribers yet
		item = DDSVariable.new()
		item.init(name)
		variables[name] = item
	else:
		item = variables[name]
	item.set_value(type, value)
	item.publish()


func subscribe(name : String):
	if not subscribed_vars.has(name):
		subscribed_vars[name] = null


func read(name : String):
	return subscribed_vars.get(name, null)
