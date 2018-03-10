import socket
import json

def connectToHost():
	# serverAddress = '00:1f:e1:dd:08:3d' # Mac Address of Bluetooth Device
	serverAddress = 'localhost'
	port = 1
	# s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((serverAddress, port))


def toSend(x, y, d):
	jsonToSend = json.dumps({'x':x, 'y':y, 'd':d}, sort_keys=True, indent=4, separators=(',',':'))
	s.send(bytes(jsonToSend, 'UTF-8'))

toSend(1, 3, 3)