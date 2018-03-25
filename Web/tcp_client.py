import socket
import random
import time
import json

def location_generator():
    types = ["self_location", "point"]
    num = 20
    counter = 0
    while True:
        yield (types[counter%1], "(%d, %d)" %(num+10, num+20))
        num = num + 10
        counter = counter + 1

def launch_TCP_client():
    ip = "127.0.0.1"
    port = 5555
    size =1024
    location = location_generator()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip, port))
    while True:
        time.sleep(1)
        data = next(location)
        data = json.dumps({"type": data[0], "location":data[1]})
        s.send(data.encode())
    s.close()

if __name__ == '__main__':
    launch_TCP_client()
