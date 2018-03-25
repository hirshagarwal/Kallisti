from flask import Flask
import json
import threading
import socket

points = []
lock = threading.Lock()

ip = "127.0.0.1"
port = 5555
size = 1024




def launch_TCP_server():
    global points
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((ip, port))
    s.listen(1)
    conn, addr = s.accept()
    print("Connection address:", addr)
    while True:
        data = conn.recv(size).decode('utf-8')
        print(data)
        data = json.loads(data)
        lock.acquire()
        try:
            points.append(data)
            if len(points)>10:
                points=[]
        finally:
            lock.release()
    s.close()





# Flask 
app = Flask(__name__)

@app.route('/index.html')
def index():
    return app.send_static_file("index.html")

@app.route('/myscript.js')
def vmyscript():
    return app.send_static_file("myscript.js")

@app.route('/control-panel.html')
def control_panel():
    return app.send_static_file("control-panel.html")

@app.route('/drawing.js')
def drawing_js():
    return app.send_static_file("drawing.js")

@app.route('/newpoints')
def newpoints():
    global points
    lock.acquire()
    try:
        if len(points) != 0:
            local_newpoint = points
            points=[]
            return json.dumps(local_newpoint)
        else:
            return json.dumps({})
    finally:
        lock.release()

if __name__ =="__main__":
    try:
        tcp_server = threading.Thread(target = launch_TCP_server)
        tcp_server.start()
        app.run()
    finally:
        exit()