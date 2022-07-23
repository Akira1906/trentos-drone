#ifndef COMMUNICATION_H 
#define COMMUNICATION_H

void getLidarData(OS_Socket_Handle_t socket, char * buffer, uint16_t lidar);
void getData(OS_Socket_Handle_t socket, char * request, uint16_t len_request, char *  buffer);
void getLidarPosition(OS_Socket_Handle_t socket, char * buffer, uint16_t lidar);
void getDistance(OS_Socket_Handle_t socket, char * buffer);
void sendTakOffCommand(OS_Socket_Handle_t socket, char * buffer);
void sendHoverCommand(OS_Socket_Handle_t socket, char * buffer);
void sendMoveByRollPitchYawZAsyncCommand(OS_Socket_Handle_t socket, char *buffer, float roll, float pitch, float yaw, float z, float duration);
void sendMoveByVelocityBodyFrameCommand(OS_Socket_Handle_t socket, char *buffer, float vx, float vy, float vz, float duration);
void sendMoveByVelocityZCommand(OS_Socket_Handle_t socket, char *buffer, float vx, float vy, float z, float duration);

#endif