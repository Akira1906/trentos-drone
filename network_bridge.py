import socket
import airsim
import struct

HOST = "127.0.0.1"
PORT = 8888

LIDARS = ["LidarSensorHor", "LidarSensorVer1", "LidarSensorVer2"]
VEHICLE = "Drone1"

class AirSimClient:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        #self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        print('Connected!\n')

    def execute_command(self, command):
        if command["type"] == "getLidarData":
            data = self.client.getLidarData(lidar_name = LIDARS[command["lidar"]], vehicle_name = VEHICLE)
            print(data)
            return {"type": "lidar_data", "data": data}
        elif command["type"] == "takeoffAsync":
            self.client.takeoffAsync().join()
            return {"type": "executed_command"}
        elif command["type"] == "hoverAsync":
            self.client.hoverAsync().join()
            return {"type": "executed_command"}
        elif command["type"] == "moveToPositionAsync":
            self.client.moveToPositionAsync(command["x"], command["y"], command["z"], command["velocity"]).join()
            return {"type": "executed_command"}
        elif command["type"] == "moveByVelocityBodyFrameAsync":
            self.client.moveByVelocityBodyFrameAsync(command["vx"], command["vy"], command["vz"], \
                                                    command["duration"]).join()
            return {"type": "executed_command"}
        elif command["type"] == "moveByRollPitchYawThrottleAsync":
            self.client.moveByRollPitchYawThrottleAsync(command["roll"], command["pitch"], command["yaw"], \
                                                        command["throttle"], command["duration"], VEHICLE).join()
            return {"type": "executed_command"}
        

def parse_command(data):
    command_byte = struct.unpack("!H", data[:2])[0]
    if command_byte == 0:
        #getLidarData
        lidar = struct.unpack("!H", data[2:])[0]
        return {"type": "getLidarData", "lidar": lidar}
    elif command_byte == 1:
        #takeoffAsync
        return {"type": "takeoffAsync"}
    elif command_byte == 2:
        #hoverAsync
        return {"type": "hoverAsync"}
    elif command_byte == 3:
        #moveToPositionAsync
        x, y, z, velocity = struct.unpack("!3fI", data[2:])
        return {"type": "moveToPositionAsync", "x": x, "y":y, "z":z, "velocity": velocity}
    elif command_byte == 4:
        #moveByVelocityBodyFrameAsync
        vx, vy, vz, duration = struct.unpack("!4f", data[2:])
        return {"type": "moveByVelocityBodyFrameAsync",  \
                "vx":vx, "vy":vy, "vz": vz, "duration": duration}
    elif command_byte == 5:
        #moveByRollPitchYawThrottleAsync
        roll, pitch, yaw, throttle, duration = struct.unpack("!5f", data[2:])
        return {"type": "moveByRollPitchYawThrottleAsync",  \
                "roll": roll, "pitch": pitch, "yaw": yaw, "throttle": throttle, "duration": duration}
    

def serialize_result(result):
    resp = b""
    if result["type"] == "lidar_data":
        lidar_data = result["data"]
        orientation = lidar_data.pose.orientation
        q0, q1, q2, q3 = orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val
        resp += struct.pack("!4f", q0, q1, q2, q3)

        position = lidar_data.pose.position
        x, y, z = position.x_val, position.y_val, position.z_val
        resp += struct.pack("!3f", x, y, z)

        n_point_clouds = len(lidar_data.point_cloud)

        resp += struct.pack("!I", n_point_clouds)
        
        for x in lidar_data.point_cloud:
            resp += struct.pack("!f", x)
    elif result["type"] == "executed_command":
        resp += struct.pack("!H", 1)

    print(len(resp))
    return resp


def main():
    airsim_client = AirSimClient()
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                
                try:
                    command = parse_command(data)
                except Exception as e:
                    print(e)
                    continue
                print(command)
                result = airsim_client.execute_command(command)
                resp = serialize_result(result)
                conn.sendall(resp)


if __name__ == "__main__":
    main()