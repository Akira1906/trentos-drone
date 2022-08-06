#ifndef COMMUNICATION_H 
#define COMMUNICATION_H

/* 
    Sends request to get Lidar data
    Receives and copies data into buffer
*/
void getLidarData(OS_Socket_Handle_t socket, char * buffer, uint16_t lidar);

void getData(OS_Socket_Handle_t socket, char * request, uint16_t len_request, char *  buffer);
/*
    Sends request to get lidar position
    Received and puts data into buffer
*/

void getLidarPosition(OS_Socket_Handle_t socket, char * buffer, uint16_t lidar);
/*
    Sends request to get distance from distance vector
    Received and puts data into buffer
*/
void getDistance(OS_Socket_Handle_t socket, char * buffer);

/* All the rest send commands and their respective arguments, wait for response from the simulator that command was executed */

void sendTakOffCommand(OS_Socket_Handle_t socket, char * buffer);
void sendHoverCommand(OS_Socket_Handle_t socket, char * buffer);
void sendMoveByRollPitchYawZAsyncCommand(OS_Socket_Handle_t socket, char *buffer, float roll, float pitch, float yaw, float z, float duration);
void sendMoveByVelocityBodyFrameCommand(OS_Socket_Handle_t socket, char *buffer, float vx, float vy, float vz, float duration);
void sendMoveByVelocityZCommand(OS_Socket_Handle_t socket, char *buffer, float vx, float vy, float z, float duration);
void sendRotateByYawRateCommand(OS_Socket_Handle_t socket, char *buffer, uint16_t yaw_rate, uint16_t duration);

#endif