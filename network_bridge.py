import socket
import airsim
import struct
import numpy as np

HOST = "0.0.0.0"
PORT = 8888

#SHOULD BE COMPATIBLE WITH SETTINGS.JSON
LIDARS = ["LidarSensorHor", "LidarSensorVer1", "LidarSensorVer2"]
VEHICLE = "Drone1"
DISTANCE = "Distance"

class AirSimClient:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        #self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        print('Connected!\n')


    def process_lidar_data(self, lidar_data):
        data = []
        orientation = lidar_data.pose.orientation
        q0, q1, q2, q3 = orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val
        rotation_matrix = np.array(([1-2*(q2*q2+q3*q3),2*(q1*q2-q3*q0),2*(q1*q3+q2*q0)],
                                        [2*(q1*q2+q3*q0),1-2*(q1*q1+q3*q3),2*(q2*q3-q1*q0)],
                                        [2*(q1*q3-q2*q0),2*(q2*q3+q1*q0),1-2*(q1*q1+q2*q2)]))

        position = lidar_data.pose.position

        for i in range(0, len(lidar_data.point_cloud), 3):
            xyz = lidar_data.point_cloud[i:i+3]

            corrected_x, corrected_y, corrected_z = np.matmul(rotation_matrix, np.asarray(xyz))
            final_x = corrected_x + position.x_val
            final_y = corrected_y + position.y_val
            final_z = corrected_z + position.z_val
            data += [final_x, final_y, final_z]

        return data
        
    def execute_command(self, command):
        if command["type"] == "getLidarData":
            lidar_data = self.client.getLidarData(lidar_name = LIDARS[command["lidar"]], vehicle_name = VEHICLE)
            data = self.process_lidar_data(lidar_data)
            return {"type": "lidar_data", "data": data}
        elif command["type"] == "getLidarDataPosition":
            lidar_data = self.client.getLidarData(lidar_name = LIDARS[command["lidar"]], vehicle_name = VEHICLE)
            position = lidar_data.pose.position
            return {"type": "lidar_data_position", "data": [position.x_val, position.y_val, position.z_val]}
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
        elif command["type"] == "moveByRollPitchYawZAsync":
            self.client.moveByRollPitchYawZAsync(command["roll"], command["pitch"], command["yaw"], \
                                                command["z"], command["duration"], VEHICLE).join()
            return {"type": "executed_command"}
        elif command["type"] == "getDistanceSensorData":
            distance_data = self.client.getDistanceSensorData(DISTANCE, VEHICLE)
            return {"type": "distance_data", "data":  distance_data.distance}
        elif command["type"] == "moveByVelocityZAsync":
            self.client.moveByVelocityZAsync(command["vx"], command["vy"], command["z"],  command["duration"],\
                                            airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 90)).join()
            return {"type": "executed_command"}
            

def parse_command(data):
    command_byte = struct.unpack("<H", data[:2])[0]
    
    if command_byte == 0:
        #getLidarData
        lidar = struct.unpack("<H", data[2:])[0]
        return {"type": "getLidarData", "lidar": lidar}
    elif command_byte == 1:
        #takeoffAsync
        return {"type": "takeoffAsync"}
    elif command_byte == 2:
        #hoverAsync
        return {"type": "hoverAsync"}
    elif command_byte == 3:
        #moveToPositionAsync
        x, y, z, velocity = struct.unpack("<3fI", data[2:])
        return {"type": "moveToPositionAsync", "x": x, "y":y, "z":z, "velocity": velocity}
    elif command_byte == 4:
        #moveByVelocityBodyFrameAsync
        vx, vy, vz, duration = struct.unpack("<4f", data[2:])
        return {"type": "moveByVelocityBodyFrameAsync",  \
                "vx":vx, "vy":vy, "vz": vz, "duration": duration}
    elif command_byte == 5:
        #moveByRollPitchYawThrottleAsync
        roll, pitch, yaw, throttle, duration = struct.unpack("<5f", data[2:])
        return {"type": "moveByRollPitchYawThrottleAsync",  \
                "roll": roll, "pitch": pitch, "yaw": yaw, "throttle": throttle, "duration": duration}
    elif command_byte == 6:
        #moveByRollPitchYawZAsync
        roll, pitch, yaw, z, duration = struct.unpack("<5f", data[2:])
        return {"type": "moveByRollPitchYawZAsync", \
                "roll": roll, "pitch": pitch, "yaw": yaw, "z": z, "duration": duration}
    elif command_byte == 7:
        #getDistanceSensorData
        return {"type": "getDistanceSensorData"}
    elif command_byte == 8:
        #getLidarDataPosition
        lidar = struct.unpack("<H", data[2:])[0]
        return {"type": "getLidarDataPosition", "lidar": lidar}
    elif command_byte == 9:
        vx, vy, z, duration = struct.unpack("<4f", data[2:])
        return {"type": "moveByVelocityZAsync", "vx": vx, "vy": vy, "z": z, "duration": duration}


def serialize_result(result):
    resp = b""
    if result["type"] == "lidar_data":
        lidar_data = result["data"]

        n_point_clouds = len(lidar_data)
        resp += struct.pack("<I", n_point_clouds)
        for x in lidar_data:
            resp += struct.pack("<f", x)
    elif result["type"] == "lidar_data_position":
        resp += struct.pack("<3f", *result["data"])
    elif result["type"] == "distance_data":
        resp += struct.pack("<f", result["data"])
    elif result["type"] == "executed_command":
        resp += struct.pack("<H", 1)

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
                print(data)
                if not data:
                    break
                try:
                    command = parse_command(data)
                except Exception as e:
                    print(e)
                    continue
                print(command)
                result = airsim_client.execute_command(command)
                print(result)
                resp = serialize_result(result)
                print(resp)
                conn.sendall(resp)


if __name__ == "__main__":
    main()