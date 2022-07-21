import socket
import struct
import time

HOST = "10.0.0.11"
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
    elif command_type == "getLidarDataPosition":
        return struct.pack("<HH", 8, 0)
    elif command_type == "getDistanceSensorData":
        return struct.pack("<H", 7)
    elif command_type == "moveByRollPitchYawZAsync":
        return struct.pack("<H5f", 6, 0, 0, 0, 11, 0.5)
    elif command_type == "moveByVelocityZAsync":
        return struct.pack("<H4f", 9, 1, 2, 2, 11)
    


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))

        command = generate_command("getDistanceSensorData")
        s.sendall(command)
        data = s.recv(1024)
        print(struct.unpack("<f", data))
        time.sleep(1)

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
        
         
        command = generate_command("moveByRollPitchYawZAsync")
        print(len(command))
        s.sendall(command)
        data = s.recv(1024)
        time.sleep(1)
        

        # command = generate_command("moveByRollPitchYawThrottleAsync")
        # print(len(command))
        # s.sendall(command)
        # data = s.recv(1024)
        # time.sleep(1)
        # TODO getDistanceSensorData, moveByVelocityZAsync
        
    
if __name__ == "__main__":
    main()