/*
 * Copyright (C) 2021, Hensoldt Cyber GmbH
 */

#include "OS_FileSystem.h"

#include "system_config.h"
#include "OS_Socket.h"
#include "interfaces/if_OS_Socket.h"

#include "lib_debug/Debug.h"
#include <string.h>

#include <camkes.h>

//------------------------------------------------------------------------------
// static OS_FileSystem_Config_t spiffsCfg_1 =
// {
//     .type = OS_FileSystem_Type_SPIFFS,
//     .size = OS_FileSystem_USE_STORAGE_MAX,
//     .storage = IF_OS_STORAGE_ASSIGN(
//         storage_rpc_1,
//         storage_dp_1),
// };

// static OS_FileSystem_Config_t spiffsCfg_2 =
// {
//     .type = OS_FileSystem_Type_SPIFFS,
//     .size = OS_FileSystem_USE_STORAGE_MAX,
//     .storage = IF_OS_STORAGE_ASSIGN(
//         storage_rpc_2,
//         storage_dp_2),
// };

// static char fileData[250];

//------------------------------------------------------------------------------
// static void
// test_OS_FileSystem(OS_FileSystem_Config_t* cfg)
// {
//     OS_Error_t ret;
//     OS_FileSystem_Handle_t hFs;

//     static const char* fileName = "testfile.txt";
//     static const off_t fileSize = sizeof(fileData);
//     static OS_FileSystemFile_Handle_t hFile;

//     // Init file system
//     if ((ret = OS_FileSystem_init(&hFs, cfg)) != OS_SUCCESS)
//     {
//         Debug_LOG_ERROR("OS_FileSystem_init() failed, code %d", ret);
//     }

//     // Format file system
//     if ((ret = OS_FileSystem_format(hFs)) != OS_SUCCESS)
//     {
//         Debug_LOG_ERROR("OS_FileSystem_format() failed, code %d", ret);
//     }

//     // Mount file system
//     if ((ret = OS_FileSystem_mount(hFs)) != OS_SUCCESS)
//     {
//         Debug_LOG_ERROR("OS_FileSystem_mount() failed, code %d", ret);
//     }

//     // Open file
//     if((ret = OS_FileSystemFile_open(hFs, &hFile, fileName,
//                                 OS_FileSystem_OpenMode_RDWR,
//                                 OS_FileSystem_OpenFlags_CREATE)) != OS_SUCCESS)
//     {
//         Debug_LOG_ERROR("OS_FileSystemFile_open() failed, code %d", ret);
//     }

//     // Write to the file
//     off_t to_write, written;
//     to_write = fileSize;
//     written = 0;

//     while (to_write > 0)
//     {
//         if((ret = OS_FileSystemFile_write(hFs, hFile, written, sizeof(fileData), fileData)) != OS_SUCCESS)
//         {
//             Debug_LOG_ERROR("OS_FileSystemFile_write() failed, code %d", ret);
//         }

//         written  += sizeof(fileData);
//         to_write -= sizeof(fileData);
//     }

//     // Read from the file
//     uint8_t buf[sizeof(fileData)];
//     off_t to_read, read;
//     to_read = fileSize;
//     read = 0;

//     while (to_read > 0)
//     {
//         if((ret = OS_FileSystemFile_read(hFs, hFile, read, sizeof(buf), buf)) != OS_SUCCESS)
//         {
//             Debug_LOG_ERROR("OS_FileSystemFile_read() failed, code %d", ret);
//         }

//         if(memcmp(fileData, buf, sizeof(buf)))
//             Debug_LOG_ERROR("File content read does not equal file content to be written.");

//         read    += sizeof(buf);
//         to_read -= sizeof(buf);
//     }

//     // Close file
//     if((ret = OS_FileSystemFile_close(hFs, hFile)) != OS_SUCCESS)
//     {
//         Debug_LOG_ERROR("OS_FileSystemFile_close() failed, code %d", ret);
//     }

//     // Clean up
//     if((ret = OS_FileSystem_unmount(hFs)) != OS_SUCCESS)
//     {
//         Debug_LOG_ERROR("OS_FileSystem_unmount() failed, code %d", ret);
//     }

//     if ((ret = OS_FileSystem_free(hFs)) != OS_SUCCESS)
//     {
//         Debug_LOG_ERROR("OS_FileSystem_free() failed, code %d", ret);
//     }
// }

//----------------------------------------------------------------------
// Network
//----------------------------------------------------------------------

static const if_OS_Socket_t networkStackCtx =
    IF_OS_SOCKET_ASSIGN(networkStack);

//------------------------------------------------------------------------------
static OS_Error_t
waitForNetworkStackInit(
    const if_OS_Socket_t* const ctx)
{
    OS_NetworkStack_State_t networkStackState;

    for (;;)
    {
        networkStackState = OS_Socket_getStatus(ctx);
        if (networkStackState == RUNNING)
        {
            // NetworkStack up and running.
            return OS_SUCCESS;
        }
        else if (networkStackState == FATAL_ERROR)
        {
            // NetworkStack will not come up.
            Debug_LOG_ERROR("A FATAL_ERROR occurred in the Network Stack component.");
            return OS_ERROR_ABORTED;
        }

        // Yield to wait until the stack is up and running.
        seL4_Yield();
    }
}

static OS_Error_t
waitForConnectionEstablished(
    const int handleId)
{
    OS_Error_t ret;

    // Wait for the event letting us know that the connection was successfully
    // established.
    for (;;)
    {
        ret = OS_Socket_wait(&networkStackCtx);
        if (ret != OS_SUCCESS)
        {
            Debug_LOG_ERROR("OS_Socket_wait() failed, code %d", ret);
            break;
        }

        char evtBuffer[128];
        const size_t evtBufferSize = sizeof(evtBuffer);
        int numberOfSocketsWithEvents;

        ret = OS_Socket_getPendingEvents(
                  &networkStackCtx,
                  evtBuffer,
                  evtBufferSize,
                  &numberOfSocketsWithEvents);
        if (ret != OS_SUCCESS)
        {
            Debug_LOG_ERROR("OS_Socket_getPendingEvents() failed, code %d",
                            ret);
            break;
        }

        if (numberOfSocketsWithEvents == 0)
        {
            Debug_LOG_TRACE("OS_Socket_getPendingEvents() returned "
                            "without any pending events");
            continue;
        }

        // We only opened one socket, so if we get more events, this is not ok.
        if (numberOfSocketsWithEvents != 1)
        {
            Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned with "
                            "unexpected #events: %d", numberOfSocketsWithEvents);
            ret = OS_ERROR_INVALID_STATE;
            break;
        }

        OS_Socket_Evt_t event;
        memcpy(&event, evtBuffer, sizeof(event));

        if (event.socketHandle != handleId)
        {
            Debug_LOG_ERROR("Unexpected handle received: %d, expected: %d",
                            event.socketHandle, handleId);
            ret = OS_ERROR_INVALID_HANDLE;
            break;
        }

        // Socket has been closed by NetworkStack component.
        if (event.eventMask & OS_SOCK_EV_FIN)
        {
            Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned "
                            "OS_SOCK_EV_FIN for handle: %d",
                            event.socketHandle);
            ret = OS_ERROR_NETWORK_CONN_REFUSED;
            break;
        }

        // Connection successfully established.
        if (event.eventMask & OS_SOCK_EV_CONN_EST)
        {
            Debug_LOG_DEBUG("OS_Socket_getPendingEvents() returned "
                            "connection established for handle: %d",
                            event.socketHandle);
            ret = OS_SUCCESS;
            break;
        }

        // Remote socket requested to be closed only valid for clients.
        if (event.eventMask & OS_SOCK_EV_CLOSE)
        {
            Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned "
                            "OS_SOCK_EV_CLOSE for handle: %d",
                            event.socketHandle);
            ret = OS_ERROR_CONNECTION_CLOSED;
            break;
        }

        // Error received - print error.
        if (event.eventMask & OS_SOCK_EV_ERROR)
        {
            Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned "
                            "OS_SOCK_EV_ERROR for handle: %d, code: %d",
                            event.socketHandle, event.currentError);
            ret = event.currentError;
            break;
        }
    }

    return ret;
}

static void getData(OS_Socket_Handle_t socket, char * request, uint16_t len_request, char *  buffer){
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

    do
    {
        ret = OS_Socket_read(
                socket,
                buffer,
                OS_DATAPORT_DEFAULT_SIZE,
                &actualLenRecv);

        switch (ret)
        {
        case OS_SUCCESS:
            Debug_LOG_INFO(
                "OS_Socket_read() received %zu bytes of data",
                actualLenRecv);
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
    
}

static void getLidarData(OS_Socket_Handle_t socket, char * buffer, uint16_t lidar){
    uint16_t request[2] = {0, lidar};
    size_t len_request = sizeof(uint16_t) * 2;

    getData(socket, (char *)request, len_request, buffer);
}

static void getLidarPosition(OS_Socket_Handle_t socket, char * buffer, uint16_t lidar){
    uint16_t request[2] = {8, lidar};
    size_t len_request = sizeof(uint16_t) * 2;

    getData(socket, (char *)request, len_request, buffer);
}

static void getDistance(OS_Socket_Handle_t socket, char * buffer){
    uint16_t command = 7;
    getData(socket, (char *)&command, sizeof(uint16_t), buffer);   
}


static void sendTakOffCommand(OS_Socket_Handle_t socket, char * buffer){
    uint16_t takeoff = 1;
    getData(socket, (char *)&takeoff, sizeof(uint16_t), buffer);
}

static void sendHoverCommand(OS_Socket_Handle_t socket, char * buffer){
    uint16_t hover = 2;
    getData(socket, (char *)&hover, sizeof(uint16_t), buffer);
}

static void sendMoveByRollPitchYawZCommand(OS_Socket_Handle_t socket, char *buffer, float roll, float pitch, float yaw, float z, float duration){
    float data[5] = {roll, pitch, yaw, z, duration};
    char * request = malloc(sizeof(uint16_t) + sizeof(float) * 5);
    uint16_t command = 6;
    memcpy(request, &command, sizeof(uint16_t));
    memcpy(request + sizeof(uint16_t), data, sizeof(float) * 5);
    getData(socket, request, sizeof(uint16_t) + sizeof(float) * 5, buffer);
    free(request);
}

static void sendMoveByVelocityBodyFrameCommand(OS_Socket_Handle_t socket, char *buffer, float vx, float vy, float vz, float duration){
    float data[4] = {vx, vy, vz, duration};
    char * request = malloc(sizeof(uint16_t) + sizeof(float) * 4);
    uint16_t command = 4;
    memcpy(request, &command, sizeof(uint16_t));
    memcpy(request + sizeof(uint16_t), data, sizeof(float) * 4);
    getData(socket, request, sizeof(uint16_t) + sizeof(float) * 4, buffer);
    free(request);
}

static void sendMoveByVelocityZCommand(OS_Socket_Handle_t socket, char *buffer, float vx, float vy, float z, float duration){
    float data[4] = {vx, vy, z, duration};
    char * request = malloc(sizeof(uint16_t) + sizeof(float) * 4);
    uint16_t command = 9;
    memcpy(request, &command, sizeof(uint16_t));
    memcpy(request + sizeof(uint16_t), data, sizeof(float) * 4);
    getData(socket, request, sizeof(uint16_t) + sizeof(float) * 4, buffer);
    free(request);
}

static float * parseLidarPoints(char *buffer, int * number_of_points){
    memcpy(number_of_points, buffer, sizeof(int));
    float * points = malloc(sizeof(float) * (*number_of_points));
    memcpy(points, buffer + sizeof(int), sizeof(float *) * (*number_of_points));
    return points;
}

static float * parseLidarPosition(char *buffer){
    float * points = malloc(sizeof(float) * 3);
    memcpy(points, buffer, sizeof(float *) * 3);
    return points;
}


//------------------------------------------------------------------------------
int run()
{
    Debug_LOG_INFO("Starting test_app_server...");

    // Check and wait until the NetworkStack component is up and running.
    OS_Error_t ret = waitForNetworkStackInit(&networkStackCtx);
    if (OS_SUCCESS != ret)
    {
        Debug_LOG_ERROR("waitForNetworkStackInit() failed with: %d", ret);
        return -1;
    }

    OS_Socket_Handle_t hSocket;
    ret = OS_Socket_create(
              &networkStackCtx,
              &hSocket,
              OS_AF_INET,
              OS_SOCK_STREAM);
    if (ret != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_Socket_create() failed, code %d", ret);
        return -1;
    }

    const OS_Socket_Addr_t dstAddr =
    {
        .addr = CFG_TEST_HTTP_SERVER,
        .port = EXERCISE_CLIENT_PORT
    };

    ret = OS_Socket_connect(hSocket, &dstAddr);
    if (ret != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_Socket_connect() failed, code %d", ret);
        OS_Socket_close(hSocket);
        return ret;
    }

    ret = waitForConnectionEstablished(hSocket.handleID);
    if (ret != OS_SUCCESS)
    {
        Debug_LOG_ERROR("waitForConnectionEstablished() failed, error %d", ret);
        OS_Socket_close(hSocket);
        return -1;
    }

    Debug_LOG_INFO("Send request to host...");
    static char buffer[OS_DATAPORT_DEFAULT_SIZE];

    getLidarPosition(hSocket, buffer, 0);
    float * position = parseLidarPosition(buffer);
    
    for (int i = 0; i < 3; i ++){
        Debug_LOG_INFO("position point %f\n", position[i]);
    }

    getLidarData(hSocket, buffer, 0);
  
    int number_of_points = 0;
    float * points = parseLidarPoints(buffer, &number_of_points);
    

    for (int i = 0; i < number_of_points; i ++){
        Debug_LOG_INFO("point %f\n", points[i]);
    }

    Debug_LOG_INFO("Sending takeoffCommand\n");
    sendTakOffCommand(hSocket, buffer);
    Debug_LOG_INFO("Takeoff status %d\n", * (uint16_t*) buffer);


    Debug_LOG_INFO("Sending HoverCommand\n");
    sendHoverCommand(hSocket, buffer);
    Debug_LOG_INFO("Hover status %d\n", * (uint16_t*) buffer);

    Debug_LOG_INFO("Sending sendMoveByRollPitchYawZCommand\n");
    sendMoveByRollPitchYawZCommand(hSocket, buffer, 0, 0, 0, 10, 0.5);
    Debug_LOG_INFO("sendMoveByRollPitchYawZCommand status %d\n", * (uint16_t*) buffer);

    Debug_LOG_INFO("Sending getDistance\n");
    getDistance(hSocket, buffer);
    Debug_LOG_INFO("getDistance distance %f\n", * (float*) buffer);

    Debug_LOG_INFO("Sending sendMoveByVelocityBodyFrameCommand\n");
    sendMoveByVelocityBodyFrameCommand(hSocket, buffer, 1, 0, 1, 0.5);
    Debug_LOG_INFO("sendMoveByVelocityBodyFrameCommand status %d\n", * (uint16_t*) buffer);

    Debug_LOG_INFO("Sending sendMoveByVelocityZCommand\n");
    sendMoveByVelocityZCommand(hSocket, buffer, 1, 0, 1, 1);
    Debug_LOG_INFO("sendMoveByVelocityZCommand status %d\n", * (uint16_t*) buffer);


    OS_Socket_close(hSocket);
    free(points);
    free(position);


    // ----------------------------------------------------------------------
    // Storage Test
    // ----------------------------------------------------------------------

    // Work on file system 1 (residing on partition 1)
    // test_OS_FileSystem(&spiffsCfg_1);

    // Work on file system 2 (residing on partition 2)
    // test_OS_FileSystem(&spiffsCfg_2);

    Debug_LOG_INFO("Demo completed successfully.");

    return 0;
}