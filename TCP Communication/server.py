import socket

# hostAddress = '' # Mac Address of Bluetooth Device
hostAddress = 'localhost'
port = 1
backlog = 1
size = 1024
# s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((hostAddress, port))
s.listen(backlog)

try:
	client, address = s.accept()
	while 1:
		data = client.recv(size)
		if data:
			print(data)
			client.send(data)

except:
	print("Closing Socket")
	client.close()
	s.close()