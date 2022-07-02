import socket
import struct
import time

HOST = "0.0.0.0"
PORT = 8888

def generate_command(command_type):
    if command_type == "getLidarData":
        return struct.pack("!HH", 0, 0)
    elif command_type == "takeoffAsync":
        return struct.pack("!H", 1)
    elif command_type == "hoverAsync":
        return struct.pack("!H", 2)
    elif command_type == "moveToPositionAsync":
        return struct.pack("!H3fI", 3,  0, 0, -9, 5)


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))

        command = generate_command("takeoffAsync")
        s.sendall(command)
        data = s.recv(1024)
        time.sleep(3)

        command = generate_command("moveToPositionAsync")
        s.sendall(command)
        data = s.recv(1024)
        time.sleep(3)
        
        command = generate_command("hoverAsync")
        s.sendall(command)
        data = s.recv(1024)
        time.sleep(3)
        

    
if __name__ == "__main__":
    main()