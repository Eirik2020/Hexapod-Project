import socket

# Set up the client
HOST = '192.168.10.117'  # Replace with the Raspberry Pi's IP address
PORT = 65432            # Same port as the server

# Function to capture WASD input and send to server
def send_wasd_input(sock):
    print("Control with WASD keys (press 'q' to quit):")
    while True:
        key = input()  # Capture user input (e.g., 'W', 'A', 'S', 'D', or 'q')
        
        # Convert input to lowercase to handle both uppercase and lowercase
        key = key.lower()

        # Send only valid WASD keys or exit on 'q'
        if key in ['w', 'a', 's', 'd']:
            sock.sendall(key.encode('utf-8'))  # Send the keystroke to the server
        elif key == 'q':  # Quit if 'q' is pressed
            print("Exiting...")
            break
        else:
            print("Invalid input, use WASD keys to control or 'q' to quit.")

# Set up the client connection
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))  # Connect to the Raspberry Pi server
    send_wasd_input(s)       # Capture and send WASD input

