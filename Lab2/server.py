# date_time_server.py

import socket
from datetime import datetime

ip = "192.168.50.58"
port = 8002

server = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
server.bind((ip, port))
server.listen()

send_html_response = True

print("Server started on addresss:", ip, "port:", port)
while True:
    conn, addr = server.accept()
    print("replying to", addr)
    if send_html_response:
        response = str('HTTP/1.1 200 OK\nContent-Type: text/html\n\n' + str(datetime.now())).encode()
    else:
        response = str(datetime.now()).encode()

    conn.send(response)
    conn.close()
