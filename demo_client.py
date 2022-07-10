import socket
import struct
import time

HOST = "0.0.0.0"
PORT = 8888

def generate_command(command_type):
    if command_type == "getLidarData":
        return struct.pack("<HH", 0, 0)
    elif command_type == "takeoffAsync":
        return struct.pack("<H", 1)
    elif command_type == "hoverAsync":
        return struct.pack("<H", 2)
    elif command_type == "moveToPositionAsync":
        return struct.pack("<H3fI", 3,  0, 0, -9, 5)
    elif command_type == "moveByVelocityBodyFrameAsync":
        return struct.pack("<H4f", 4, 0,0,-5,0.5)
    elif command_type == "moveByRollPitchYawThrottleAsync":
        return struct.pack("<H5f", 5, 0,0, 40, 1, 0.5)


def parse_lidar_data(data):
    q0, q1, q2, q3 = struct.unpack("!4f", data[:16])
    print(q0, q1, q2, q3)
    x, y, z = struct.unpack("!3f", data[16:16 + 12])
    print(x, y, z)

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))

        
        command = generate_command("takeoffAsync")
        s.sendall(command)
        data = s.recv(1024)
        time.sleep(1)

        command = generate_command("moveToPositionAsync")
        s.sendall(command)
        data = s.recv(1024)
        time.sleep(1)
        
        command = generate_command("hoverAsync")
        s.sendall(command)
        data = s.recv(1024)
        time.sleep(1)
        
         
        command = generate_command("moveByVelocityBodyFrameAsync")
        print(len(command))
        s.sendall(command)
        data = s.recv(1024)
        time.sleep(1)
        

        command = generate_command("moveByRollPitchYawThrottleAsync")
        print(len(command))
        s.sendall(command)
        data = s.recv(1024)
        time.sleep(1)
        
    
if __name__ == "__main__":
    main()