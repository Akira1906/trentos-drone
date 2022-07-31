/*
 * Copyright (C) 2021, Hensoldt Cyber GmbH
 */

#include "OS_FileSystem.h"

#include "system_config.h"
#include "OS_Socket.h"
#include "interfaces/if_OS_Socket.h"

#include "lib_debug/Debug.h"
#include <string.h>
#include "utils.h"
#include "limits.h"
#include <camkes.h>
#include <math.h>
#include "communication.h"

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


static float * parseLidarPoints(char *buffer, int * number_of_points){
    memcpy(number_of_points, buffer, sizeof(int));
    float * points = malloc(sizeof(float) * (*number_of_points));
    memcpy(points, buffer + sizeof(int), sizeof(float *) * (*number_of_points));
    return points;
}

float * parseLidarPosition(char *buffer){
    float * points = malloc(sizeof(float) * 3);
    memcpy(points, buffer, sizeof(float *) * 3);
    return points;
}



//------------------------------------------------------------------------------
//Flight Sequences
//------------------------------------------------------------------------------


float * executeHorizontalScan(OS_Socket_Handle_t socket, char *buffer, int * n_lidar_points){
    int number_of_points = 0;
    float * lidar_data[100];
    int lidar_data_sizes[100];
    int pos = 0;
    do{
        getLidarData(socket, buffer, 0);
        float * points = parseLidarPoints(buffer, &number_of_points);
        lidar_data[pos] = points;
        lidar_data_sizes[pos] = number_of_points;
        pos += 1;
        sendMoveByVelocityBodyFrameCommand(socket, buffer, 0, 0, -5, 0.5);

    }while(number_of_points > 0);
    
    int total_length = 0;
    for (int i = 0; i < pos;  i++ ){
        total_length += lidar_data_sizes[i];
        Debug_LOG_INFO("point to data %d \n", lidar_data_sizes[i]);
    }
    Debug_LOG_INFO("total number of points %d \n", total_length / 3);
    memcpy(n_lidar_points, &total_length, sizeof(int));
    float * lidar_points = (float *)malloc(total_length * sizeof(float));
    int cur = 0;
    for (int i = 0; i < pos;  i++ ){
        memcpy(lidar_points + cur, lidar_data[i], lidar_data_sizes[i] * sizeof(float));
        cur += lidar_data_sizes[i];
        free(lidar_data[i]);
    }
    
    return lidar_points;
    
}


float *  evaluateLandingTarget(OS_Socket_Handle_t socket, char *buffer, float * lidar_points, int n_lidar_points){
    float lowestZ = lidar_points[n_lidar_points-1];
    Debug_LOG_INFO("Lowest Z %f\n",  lowestZ);
    int pos;
    for (int i = 0; i <= n_lidar_points - 3; i += 3){
        if (lidar_points[i + 2] == lowestZ){
            pos = i;
            break;
        }
    }
    float landing_points[n_lidar_points - pos][3];

    int max_distance = 5;

    float last_point[2] = {(lidar_points + pos)[0], (lidar_points + pos)[1]};

    float cur_sum[2] = {(lidar_points + pos)[0], (lidar_points + pos)[1]};
    int n = 1;
    int landing_point_n = 0;

    for (int i = pos + 3; i <= n_lidar_points - 3; i += 3){
        Debug_LOG_INFO("Cur lidar_point %f %f \n", lidar_points[i], lidar_points[i+1]);
        float x = lidar_points[i];
        float y = lidar_points[i+1];
        float z = lidar_points[i+2];
        if (sqrt(pow(x - last_point[0], 2) + pow(y - last_point[1], 2)) < max_distance){
            cur_sum[0] += x;
            cur_sum[1] += y;
            n += 1;
        }else{
            landing_points[landing_point_n][0] = cur_sum[0] / n;
            landing_points[landing_point_n][1] = cur_sum[1] / n;
            landing_points[landing_point_n][2] = z;
            landing_point_n += 1;
            n = 1;
            cur_sum[0] = x;
            cur_sum[1] = y;
        }

        last_point[0] = x;
        last_point[1] = y;
    }
    
    landing_points[landing_point_n][0] = cur_sum[0] / n;
    landing_points[landing_point_n][1] = cur_sum[1] / n;
    landing_points[landing_point_n][2] = lowestZ;
    landing_point_n += 1;

    for (int i = 0; i <  landing_point_n; i ++ ){
        Debug_LOG_INFO ("Detected landing point %f %f %f\n", landing_points[i][0], landing_points[i][1], landing_points[i][2]);
    }

    getLidarPosition(socket, buffer, 0);
    float * lidar_position = parseLidarPosition(buffer);
    Debug_LOG_INFO("Lidar position %f %f \n", lidar_position[0], lidar_position[0]);
    int res = 0;
    int max_dist = INT_MIN;
    for (int i = 0; i <= landing_point_n; i++){
        int cur_dist = sqrt(pow(lidar_position[0] - landing_points[i][0], 2) + pow(lidar_position[1] - landing_points[i][1], 2));
        Debug_LOG_INFO("Distance %d to landing point  %f %f %f\n", cur_dist, landing_points[i][0], landing_points[i][1], landing_points[i][2]);
        if (cur_dist > max_dist){
            max_dist = cur_dist;
            res = i;
        }
    }

    float * landing_point = malloc(sizeof(float) * 3);
    memcpy(landing_point,     &(landing_points[res][0]), sizeof(float)); 
    memcpy(landing_point + 1, &(landing_points[res][1]), sizeof(float));
    memcpy(landing_point + 2, &(landing_points[res][2]), sizeof(float));

    return landing_point;
}


/* 
        Steers the drone in the direction of the landing position until the landing platform is
        detected below the drone with the distance Sensor.
        Args:
            OS_Socket_Handle_t socket       : Network socket to communicate with the drone
            char *buf                       : Pointer to the buffer used for receiving dataframes
            uint16_t lidar                  : ID of the lidar used to get the drones position
            float* landingPosition          : Pointer to the 3 float coordinates describing the rough landing position

*/ 
void flyToLandingPosition(OS_Socket_Handle_t socket, char *buf, uint16_t lidar, float* landingPosition){
    getLidarPosition(socket, buf, lidar);
    //positon should be in buffer now TODO replace with parsePositon?
    //determines the position of the landing platform relative to the drone
    float currPosition[3];
    memcpy(currPosition, buf, sizeof(currPosition));//TODO should parsePosition be used here?
    Debug_LOG_INFO("\nflyToLandingPosition - current Position of the drone: %f, %f, %f\n", currPosition[0], currPosition[1], currPosition[2]);
    float xDifference = landingPosition[0] - currPosition[0];
    float yDifference = landingPosition[1] - currPosition[1];
    //determines the position of the landing plattform relative to the drone
    float distanceToDrone = sqrt(pow(xDifference, 2) + pow(yDifference, 2));

    //calculate the direction at which the drone has to fly to reach the landingPosition
    float yawAngle = - acos(xDifference/distanceToDrone);

    //fly in the direction of the landingPosition until the distance sensor detects the landing platform
    float flightPitch = 0.02 * M_PI;
    
    sendMoveByRollPitchYawZAsyncCommand(socket, buf, 0, 0, yawAngle, landingPosition[2]-3, 1);
    getDistance(socket, buf);
    //sleep(1)

    float distanceToGround;
    memcpy(&distanceToGround, buf, sizeof(distanceToGround));
    Debug_LOG_INFO("flyToLandingPositon - distanceToGround: %f", distanceToGround);
    while(distanceToGround > 6){
        sendMoveByRollPitchYawZAsyncCommand(socket, buf, 0, flightPitch, yawAngle, landingPosition[2]-3, 0.1);
        getDistance(socket, buf);
        memcpy(&distanceToGround, buf, sizeof(distanceToGround));
        Debug_LOG_INFO("flyToLandingPositon - distanceToGround: %f", distanceToGround);
    }
    Debug_LOG_INFO("flyToLandingPosition - Reached Destination");
    sendMoveByRollPitchYawZAsyncCommand(socket, buf, 0, -6*flightPitch, yawAngle, landingPosition[2]-3, 3);

}


/* 
        Parses the lidar data generated by the vertical lidars 
        and detects the center of the landing spot close to the drone.
        Args:
            float* buf1              : Pointer to buffer2 containing pointcloud1
            int buf1_len            : Length of buffer1 in bytes
            float* buf2              : Pointer to buffer2 containing pointcloud2
            int buf2_len            : Length of buffer1 in bytes
            uint16_t lidar1         : ID of the lidar1
            uint16_t lidar2         : ID of the lidar2 

        Returns:
            float* landing_target   : Pointer to the 3 float coordinates of the calculated landing target

*/
/*
float * detectExactLandingPoint(float* buf1, int buf1_len, float* buf2, int buf2_len, uint16_t lidar1, uint16_t lidar2){
    if(buf1_len == 0 || buf2_len == 0)
        Debug_LOG_ERROR("detectExactLandingPoint: pointcloud buffers too small");

    float* pointcloud1 = buf1;
    float* pointcloud2 = buf2;

    float* pointcloud1_end = roughlyFilterHighestPoints(pointcloud1, pointcloud1 + buf1_len/4, 0.1);
    float* pointcloud2_end = roughlyFilterHighestPoints(pointcloud2, pointcloud2 + buf2_len/4, 0.1);

    int pointcloud1_len = (pointcloud1_end - pointcloud1)/3;
    int pointcloud2_len = (pointcloud2_end - pointcloud2)/3;

    float lowest_z1 = pointcloud1[2];
    float lowest_z2 = pointcloud2[2];

    short ignore_lidar = 0;
    if(fabs(lowest_z1 - lowest_z2) > 1){
        if(lowest_z1 < lowest_z2)
            ignore_lidar = 2;
        else
            ignore_lidar = 1;
    }

    float middle_point1[3];
    float middle_point2[3];
    float currPosition[3];
    getLidarPosition(socket, buf1, lidar1);
    //positon should be in buffer now
    //determines the position of the landing platform relative to the drone
    memcpy(currPosition, buf1, sizeof(currPosition));//TODO should parsePosition be used here?


    if(ignore_lidar == 2 || ignore_lidar == 0 ){
        middle_point1 = getClosestObjectMiddlePoint(pointcloud1, pointcloud1_len, currPosition);
    }

    if(ignore_lidar == 1 || ignore_lidar == 0){
        middle_point2 = getClosestObjectMiddlePoint(pointcloud2, pointcloud2_len, currPosition);
    }

    float* landing_target = malloc(sizeof(float) * 3);

    if(ignore_lidar == 2){
        landing_target = middle_point1;
        landing_target[2] = lowest_z1;
        return landing_target;
    }

    if(ignore_lidar == 1){
        landing_target = middle_point2;
        landing_target[2] = lowest_z2;
        return landing_target;
    }
    
    landing_target[0] = (middle_point1[0] + middle_point2[0]) / 2;
    landing_target[1] = (middle_point1[1] + middle_point2[1]) / 2;
    landing_target[2] = (lowest_z1 + lowest_z2)/2;
    return landing_target;
}
*/

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
    
    int n_lidar_points = 0;
    float * lidar_points = executeHorizontalScan(hSocket, buffer, &n_lidar_points);        
    // for (int  i = 0; i <= n_lidar_points - 3; i += 3 ){
    //     Debug_LOG_INFO("Cur point %f %f %f\n", lidar_points[i], lidar_points[i + 1], lidar_points[i + 2]);
    // }
    int lidar_points_len = n_lidar_points/3;
    int object_data_len = 0;
    Debug_LOG_INFO("getSurroundingObjectsData");  
    float * object_data = getSurroundingObjectsData(lidar_points, lidar_points_len, &object_data_len);
    Debug_LOG_INFO("getSurroundingObjectsData done: %i", object_data_len); 

    for (int i = 0; i < object_data_len * 3; i += 3) {
        Debug_LOG_INFO("Object at : %f %f %f ", object_data[i], object_data[i+1], object_data[i+2]);
    }
    
    float * landingPosition = evaluateLandingTarget(hSocket, buffer, lidar_points, n_lidar_points);
    
    Debug_LOG_INFO("Landing position %f %f %f\n", landingPosition[0], landingPosition[1], landingPosition[2]);
    Debug_LOG_INFO("Test flyToLandingPosition()\n");
    Debug_LOG_INFO("Dataport size %ld\n", OS_DATAPORT_DEFAULT_SIZE);
    // float land[3] = {-68.81416794736842, 51.59855642105263, -24.090395};
    flyToLandingPosition(hSocket, buffer, 0, landingPosition);
    
    // int vertical_lidar_data_1_size = 0;
    // getLidarData(hSocket, buffer, 1);
    // float * vertical_lidar_data_1 = parseLidarPoints(buffer, &vertical_lidar_data_1_size);
    // Debug_LOG_INFO("Got 1 vertical data\n");
    // int vertical_lidar_data_2_size = 0;
    // getLidarData(hSocket, buffer, 2);
    // float * vertical_lidar_data_2 = parseLidarPoints(buffer, &vertical_lidar_data_2_size);
    // Debug_LOG_INFO("Got 2 vertical data\n");
    
    // float * landing_target = detectExactLandingPoint(vertical_lidar_data_1, vertical_lidar_data_1_size, vertical_lidar_data_2, 
    //                         vertical_lidar_data_2_size, 1, 2);


    // Debug_LOG_INFO("Landing Target %f %f %f\n", landing_target[0], landing_target[1], landing_target[2]);
    // sendRotateByYawRateCommand(hSocket, buffer, 45, 1);
    
    OS_Socket_close(hSocket);
    free(lidar_points);
    free(landingPosition);
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
