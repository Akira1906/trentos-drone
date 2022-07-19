#include "utils.h"
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define MAX_DIST 6
#define MAX_DOUBLE 100000000000000

double distance (float x, float y, float * end_point){
    x = x - end_point[0];
    y = y - end_point[1];
    double res = sqrt(pow(x, 2) + pow(y, 2)); 
    return res;
}

float * get_middle_point(float *  start, float * end){
    if (start[0] == end[0] && start[1] == end[1]){
        return start;
    }
    start[0] = (start[0] + end[0]) / 2;
    start[1] = (start[1] + end[1]) / 2;
    return start;
}

/*
   Takes a pointcloud which contains preferrably only one rotation of the lidar, so objects won't be detected twice.
        1. Determine the number of objects dislayed by the pointcloud and which points belong to them.
        2. Determines the average point in space of an object detected earlier
        Args:
            self
            points      ([float])       : List of a coordinate list
            number_of_points    int     : Length of the points list
            n_middle_points     int *   : Pointer to Integer storing number of middle points found

        Returns:
            middlePoints    ([float])   : estimated middlepoints of the objects scanned by the lidar 

*/

float * getObjectPositionsInPointcloud(float * points, int number_of_points, int * n_middle_points){
    float start[3] = {points[0], points[1], points[2]};
    float end[3] = {points[0], points[1], points[2]};
    float middle_points [number_of_points][3];
    int j = 0;

    for (int i = 3; i <= number_of_points - 3; i += 3){
        float x = points[i];
        float y = points[i + 1];
        float z = points[i + 2];

        if (distance(x, y, end) < MAX_DIST){
            end[0] = x;
            end[1] = y;
            end[2] = z;
        }else{
            float * middle_point = get_middle_point(start, end);
            middle_points[j][0] = middle_point[0];
            middle_points[j][1] = middle_point[1];
            middle_points[j][2] = middle_point[3];
            j += 1;
            start[0] = x;
            start[1] = y;
            start[2] = z;
            end[0] = x;
            end[1] = y;
            end[2] = z;
        }

    }
    
    float * middle_point = get_middle_point(start, end);
    middle_points[j][0] = middle_point[0];
    middle_points[j][1] = middle_point[1];
    middle_points[j][2] = middle_point[3];
    j += 1;
    memcpy(n_middle_points, &j, sizeof(int));
    float * res = malloc(j * sizeof(float) * 3);
    memcpy(res, middle_points, j * sizeof(float) * 3);
    return res;
    
}


/*
        Takes a pointcloud which contains only similar z values.
        1. Determine the number of objects dislayed by the pointcloud and which points belong to them.
        2. Determines the average point in space of an object detected earlier
        3. Determines the distance of the objects to the drone
        4. Chooses the closest object to the drone and returns its average point
        Args:            
            middle_points([[str]])  : Already filtered middle_points
            n_middle_points int     : number of middle points
            lidar_position [float, float, float] : Current lidar poisiton
        Returns:
            closest_middle_point ([float, float, float]) : Coordinates of the middlepoint of an detected object closest to the drone

*/

float * getClosestObjectMiddlePoint(float * middle_points, int n_middle_points, float * lidar_position){
    float * closest_middle_point = malloc(sizeof(float) * 3);
    closest_middle_point[0] = middle_points[0];
    closest_middle_point[1] = middle_points[1];
    closest_middle_point[2] = middle_points[2];
    double min_distance = MAX_DOUBLE;
    double cur_distance = 0;
    if (n_middle_points > 3){
        for(int i = 0; i < n_middle_points * 3; i += 3){
            float x = middle_points[i];
            float y = middle_points[i + 1];
            float z = middle_points[i + 2];

            cur_distance = distance(x, y, lidar_position);
            if (cur_distance < min_distance){
                min_distance = cur_distance;
                closest_middle_point[0] = x;
                closest_middle_point[1] = y;
                closest_middle_point[2] = z;
            }
        }
    }
    return closest_middle_point;
}