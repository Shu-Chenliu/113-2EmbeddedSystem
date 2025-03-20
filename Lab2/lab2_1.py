import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# TCP Server Configuration
HOST = '172.20.10.13'  # Listen on all available interfaces
PORT = 8002       # Port to listen on
BUFFER_SIZE = 1024 # Max amount of data received at once

# Data storage (keeping last 100 values)
history_length = 100
data_x = deque(maxlen=history_length)
data_y = deque(maxlen=history_length)
data_z = deque(maxlen=history_length)

# Set up socket server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print(f"Listening on {HOST}:{PORT}...")
client_socket, client_address = server_socket.accept()
print(f"Connection from {client_address}")

def update_plot(frame):
    global client_socket
    try:
        data = client_socket.recv(BUFFER_SIZE).decode('utf-8').strip()
        print(data)
        if data:
            values = data.split(' ')
            values = [v.split(':')[1] for v in values]  # Extracts the values [-1, 4, 979]
            print(values)

            if len(values) == 3:  # Expecting "x,y,z"
                x, y, z = map(float, values)
                print(x,y,z)
                data_x.append(x)
                data_y.append(y)
                data_z.append(z)
                
                ax1.clear()
                ax1.plot(data_x, label='X-axis')
                ax1.plot(data_y, label='Y-axis')
                ax1.plot(data_z, label='Z-axis')
                ax1.legend()
                ax1.set_ylim([-1000, 1000])  # Adjust range as needed
    except Exception as e:
        print(f"Error: {e}")

try:

    fig, ax1 = plt.subplots()
    ani = animation.FuncAnimation(fig, update_plot, interval=100, save_count=200)
    plt.show()
except KeyboardInterrupt:
    print("Exiting gracefully...")
finally:
    print("Closing sockets...")
    client_socket.close()
    server_socket.close()
