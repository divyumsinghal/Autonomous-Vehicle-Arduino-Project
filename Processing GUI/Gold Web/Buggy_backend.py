from flask import Flask, request
import socket
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

app = Flask(__name__)
server_address = (os.getenv('SERVER_ADDRESS'), 5200)  # Address of your Arduino server
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(server_address)

@app.route('/')
def index():
    return app.send_static_file('index.html')


@app.route('/command', methods=['POST'])
def send_command():
    command = request.form['command']
    client_socket.sendall(command.encode())
    print(command.encode())
    return 'Command sent successfully'


@app.route('/status')
def get_status():

    try:

        # Set up a socket server to listen for incoming connections
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:

            # Bind the server socket to the server address
            server_socket.bind(server_address)

            # Listen for incoming connections
            server_socket.listen(1)
            
            print("Waiting for connection from the buggy...")

            # Accept a connection
            connection, client_address = server_socket.accept()
            
            print("Connection established with the buggy:", client_address)

            # Receive the message from the buggy
            message = connection.recv(1024).decode()  # Receive string message
            
            if message:

                values = message.split(',')

                if len(values) == 6:
                    distance_travelled = values[0]
                    speed_travelled = values[1]
                    obstacle_distance = values[2]
                    speed_pwm_percent = values[3]
                    mode_read_from_server = values[4]
                    tag_read_from_server = values[5]

                    mode_name = ""

                    if mode_read_from_server == '0':
                        mode_name = "Tag Controlled Speed"
                    elif mode_read_from_server == '1':
                        mode_name = "Object Following"
                    elif mode_read_from_server == '2':
                        mode_name = "GUI Controlled Target Speed"

                    print(f"Distance travelled: {distance_travelled} meters<br>" \
                           f"Obstacle distance: {'No Object within 50 cm' if obstacle_distance[0] == '5' else obstacle_distance} cm<br>" \
                           f"Speed of Car: {speed_travelled} cm/s<br>" \
                           f"Mode: {mode_name}<br>" \
                           f"Tag: {tag_read_from_server}<br>" \
                           f"Speed percentage of PWM Signal: {speed_pwm_percent}%")

                    # Return status message
                    return f"Distance travelled: {distance_travelled} meters<br>" \
                           f"Obstacle distance: {'No Object within 50 cm' if obstacle_distance[0] == '5' else obstacle_distance} cm<br>" \
                           f"Speed of Car: {speed_travelled} cm/s<br>" \
                           f"Mode: {mode_name}<br>" \
                           f"Tag: {tag_read_from_server}<br>" \
                           f"Speed percentage of PWM Signal: {speed_pwm_percent}%"

    except Exception as e:
        print("Error:", e)

    return 'Error fetching status'


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
