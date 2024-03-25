from flask import Flask, render_template, request
import socket

app = Flask(__name__)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.0.24', 5200))  # Connect to your Arduino server

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/command', methods=['POST'])
def send_command():
    command = request.form['command']
    client_socket.sendall(command.encode())
    return 'Command sent successfully'

if __name__ == '__main__':
    app.run(debug=True)
