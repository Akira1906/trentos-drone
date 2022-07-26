#include "OS_FileSystem.h"

#include "system_config.h"
#include "OS_Socket.h"
#include "interfaces/if_OS_Socket.h"

#include "lib_debug/Debug.h"
#include <string.h>
// #include "utils.h"
#include <camkes.h>
// #include <math.h>
#include "communication.h"

void getData(OS_Socket_Handle_t socket, char * request, uint16_t len_request, char *  buffer){
    size_t n;
    OS_Error_t ret;
    do
    {
        seL4_Yield();
        ret = OS_Socket_write(socket, request, len_request, &n);
    }
    while (ret == OS_ERROR_TRY_AGAIN);

    if (OS_SUCCESS != ret)
    {
        Debug_LOG_ERROR("OS_Socket_write() failed with error code %d", ret);
    }

    size_t actualLenRecv = 0;
    // Read initial packet size
    size_t packet_size = 0;
    do
    {
        ret = OS_Socket_read(
                socket,
                &packet_size,
                2,
                &actualLenRecv);

        switch (ret)
        {
        case OS_SUCCESS:
            // Debug_LOG_INFO(
            //     "OS_Socket_read() received %zu bytes of data",
            //     actualLenRecv);
            continue;

        case OS_ERROR_TRY_AGAIN:
            Debug_LOG_TRACE(
                "OS_Socket_read() reported try again");
            continue;
        
        case OS_ERROR_CONNECTION_CLOSED:
            Debug_LOG_INFO(
                "OS_Socket_read() reported connection closed");
            break;
        
        case OS_ERROR_NETWORK_CONN_SHUTDOWN:
            Debug_LOG_DEBUG(
            "OS_Socket_read() reported connection closed");
            break;
        
        default:
            Debug_LOG_ERROR(
                "OS_Socket_read() failed, error %d", ret);
            break;
        }
    }
    while (ret != OS_SUCCESS);

    // Debug_LOG_INFO("Packet size %d\n", packet_size);

    size_t data_read = 0;
    char* position = buffer;
    
    size_t read = 0;

    while (packet_size - data_read > 0)
    {
        ret = OS_Socket_read(socket, position, packet_size - data_read, &read);

        // Debug_LOG_INFO("OS_Socket_read() - bytes read: %d, err: %d", read, ret);

        switch (ret)
        {
        case OS_SUCCESS:
            data_read += read;
            // Debug_LOG_INFO("Total data read  %d  left %d\n", data_read, packet_size - data_read); 
            position += data_read;
            break;
        case OS_ERROR_CONNECTION_CLOSED:
            Debug_LOG_WARNING("connection closed");
            read = 0;
            break;
        case OS_ERROR_NETWORK_CONN_SHUTDOWN:
            Debug_LOG_WARNING("connection shut down");
            read = 0;
            break;
        case OS_ERROR_TRY_AGAIN:
                Debug_LOG_WARNING(
                    "OS_Socket_read() reported try again");
                seL4_Yield();
                continue;
        default:
            Debug_LOG_ERROR("OS_Socket_read() returned error code %d, bytes read %zu",
                            ret, (size_t) (position - buffer));
        }
    }

    
}


void getLidarData(OS_Socket_Handle_t socket, char * buffer, uint16_t lidar){
    uint16_t request[2] = {0, lidar};
    size_t len_request = sizeof(uint16_t) * 2;

    getData(socket, (char *)request, len_request, buffer);
}


void getLidarPosition(OS_Socket_Handle_t socket, char * buffer, uint16_t lidar){
    uint16_t request[2] = {8, lidar};
    size_t len_request = sizeof(uint16_t) * 2;

    getData(socket, (char *)request, len_request, buffer);
}

void getDistance(OS_Socket_Handle_t socket, char * buffer){
    uint16_t command = 7;
    getData(socket, (char *)&command, sizeof(uint16_t), buffer);   
}


void sendTakOffCommand(OS_Socket_Handle_t socket, char * buffer){
    uint16_t takeoff = 1;
    getData(socket, (char *)&takeoff, sizeof(uint16_t), buffer);
}

void sendHoverCommand(OS_Socket_Handle_t socket, char * buffer){
    uint16_t hover = 2;
    getData(socket, (char *)&hover, sizeof(uint16_t), buffer);
}

void sendMoveByRollPitchYawZAsyncCommand(OS_Socket_Handle_t socket, char *buffer, float roll, float pitch, float yaw, float z, float duration){
    float data[5] = {roll, pitch, yaw, z, duration};
    char * request = malloc(sizeof(uint16_t) + sizeof(float) * 5);
    uint16_t command = 6;
    memcpy(request, &command, sizeof(uint16_t));
    memcpy(request + sizeof(uint16_t), data, sizeof(float) * 5);
    getData(socket, request, sizeof(uint16_t) + sizeof(float) * 5, buffer);
    free(request);
}

void sendMoveByVelocityBodyFrameCommand(OS_Socket_Handle_t socket, char *buffer, float vx, float vy, float vz, float duration){
    float data[4] = {vx, vy, vz, duration};
    char * request = malloc(sizeof(uint16_t) + sizeof(float) * 4);
    uint16_t command = 4;
    memcpy(request, &command, sizeof(uint16_t));
    memcpy(request + sizeof(uint16_t), data, sizeof(float) * 4);
    getData(socket, request, sizeof(uint16_t) + sizeof(float) * 4, buffer);
    free(request);
}

void sendMoveByVelocityZCommand(OS_Socket_Handle_t socket, char *buffer, float vx, float vy, float z, float duration){
    float data[4] = {vx, vy, z, duration};
    char * request = malloc(sizeof(uint16_t) + sizeof(float) * 4);
    uint16_t command = 9;
    memcpy(request, &command, sizeof(uint16_t));
    memcpy(request + sizeof(uint16_t), data, sizeof(float) * 4);
    getData(socket, request, sizeof(uint16_t) + sizeof(float) * 4, buffer);
    free(request);
}

void sendRotateByYawRateCommand(OS_Socket_Handle_t socket, char *buffer, uint16_t yaw_rate, uint16_t duration){
    uint16_t request[3] = {10, yaw_rate, duration};
    size_t len_request = sizeof(uint16_t) * 3;

    getData(socket, (char *)request, len_request, buffer);
}