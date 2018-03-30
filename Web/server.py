from flask import Flask, request, send_from_directory, make_response
import json
import threading
import socket
import paramiko

points = []
lock = threading.Lock()

address = "28:C2:DD:44:20:C8"
port = 8888
size = 1024
ssh = None




def launch_TCP_server():
    global points
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.bind((address, port))
    s.listen(1)
    conn, addr = s.accept()
    print("Connection address:", addr)
    while True:
        data = conn.recv(size).decode('utf-8')
        print("receive: ", data)
        data = json.loads(data)
        lock.acquire()
        try:
            points.append(data)
        finally:
            lock.release()
    s.close()





# Flask 
app = Flask(__name__)

@app.route('/connect.html')
def connect_page():
    return app.send_static_file("connect.html")

@app.route('/blueprint.html')
def blueprint_page():
    return app.send_static_file("blueprint.html")

# Return Js files
@app.route('/script/<path:path>')
def js_file(path):
    return send_from_directory("static/script", path)

# Return Css files
@app.route('/style/<path:path>')
def style_file(path):
    return send_from_directory('static/style', path)

#Return Images
@app.route('/image/<path:path>')
def image_file(path):
    return send_from_directory('static/image', path)


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

@app.route('/ssh-connection')
def ssh_connection():
    global ip, ssh
    ip = request.args["ip_address"]
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(ip, username='robot', password='maker')
        print("ssh no error")
        resp = make_response("<html></html>", 200)
        return resp 
    except Exception as e:
        print(e)
        resp = make_response("<html></html>", 400)
        return resp 


@app.route('/start-robot')
def start_robot():
    stdin, stdout, stderr = ssh.exec_command("echo maker | sudo -S python3 robot_path.py start")
    # Wait until command has been executed.
    while True:
        if stdout.channel.exit_status_ready() and stdout.channel.recv_ready():
            break

    # Receive Standard ouput stream
    output=stdout.channel.recv(1024).decode("utf-8")
    
    # Examine output to check whether command get executed successfully.
    if "OK" in output:
        resp = make_response("<html></html>", 200)
        return resp
    else:
        resp = make_response("<html></html>", 400)
        return resp 

@app.route('/stop-robot')
def stop_robot():
    stdin, stdout, stderr = ssh.exec_command("echo maker | sudo -S python3 robot_path.py stop")
    # Wait until command has been executed.
    while True:
        if stdout.channel.exit_status_ready() and stdout.channel.recv_ready():
            break

    # Receive Standard ouput stream
    output=stdout.channel.recv(1024).decode("utf-8")
    
    # Examine output to check whether command get executed successfully.
    if "OK" in output:
        resp = make_response("<html></html>", 200)
        return resp
    else:
        resp = make_response("<html></html>", 400)
        return resp 




if __name__ =="__main__":
    try:
        tcp_server = threading.Thread(target = launch_TCP_server)
        tcp_server.start()
        app.run()
    finally:
        exit()