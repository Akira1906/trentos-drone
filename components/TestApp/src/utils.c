#include "utils.h"
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

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

    for (int i = 3; i <= (number_of_points*3) - 3; i += 3){
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


/* 
        Filters a 2-dimensional pointcloud array for their z value exactly. The array is filtered by overriding the old
        array with the filtered elements. The functions returns a pointer to one past the end of this now filtered array.
        Args:
            float *pointcloudBegin          : Pointer to the first element of the pointcloud array
            float *pointcloudEnd            : Pointer to one past the last pointcloud array element    
            float zFilter                   : Z value on that we want to filter on

        Returns:
            filteredPointcloudEnd           : Pointer to one past the last array element of the filtered array

*/
float* exactFilterPoints(float *pointcloudBegin, float *pointcloudEnd, float zFilter){
    float *pos = pointcloudBegin;
    //Filter for zFilter z value
    for(; pointcloudBegin != pointcloudEnd; pointcloudBegin+=3){
        if(pointcloudBegin[2] == zFilter){
            pos[0] = pointcloudBegin[0];
            pos[1] = pointcloudBegin[1];
            pos[2] = pointcloudBegin[2];
            pos += 3;
        }
    }
  
    return (float*)pos; //points to one past the last element of the filtered array
}


/* 
        Filters a 2-dimensional array for their roughly lowest z value with a defined accuracy.
        Args:
            float *pointcloudBegin          : Pointer to the first element of the pointcloud array
            float *pointcloudEnd            : Pointer to one past the last pointcloud array element
            float accuracy                  : filter accuracy

        Returns:
            filteredPointcloudEnd           : Pointer to one past the last array element of the filtered array

*/ 
float* roughlyFilterHighestPoints(float *pointcloudBegin, float *pointcloudEnd, float accuracy){
    float *pos = pointcloudBegin;
    float lowestZ = 0;
    //Determine lowest z value
    for(; pos != pointcloudEnd; pos += 3){
        if(pos[2]<lowestZ){
            lowestZ = pos[2];
        }
    }
    //Filter for lowestZ with accuracy
    pos = pointcloudBegin;
    for(; pointcloudBegin != pointcloudEnd; pointcloudBegin += 3){
        if(pointcloudBegin[2] <= lowestZ + accuracy){
            pos[0] = pointcloudBegin[0];
            pos[1] = pointcloudBegin[1];
            pos[2] = pointcloudBegin[2];
            pos += 3;
        }
    }

    return (float*)pos; //points to one past the last element of the filtered array
}


/*
        Filters each z-level of horizontal lidar data for objects. Combines the objects of each z-level to one object array
        which contains the objects contained in the lidar_points and their respective height.
        Args:
            float *lidar_points         : Pointer to the pointcloud array
            int lidar_points_len        : Length of the pointcloud array (number of points)
            int *object_data_len        : Pointer to return the length of object_data (number of objects)
        Returns:
            float *object_data          : Pointer to the object data array


*/
float * getSurroundingObjectsData (float * lidar_points, int lidar_points_len, int * return_len) {
    int max_distance = MAX_DIST;//TODO set max dist
    float * objects_coordinates_per_level = malloc(sizeof(float) * lidar_points_len * 3);
    float * objects_coordinates_pos = objects_coordinates_per_level;
    float * lidar_points_pos = lidar_points;

    int objects_coordinates_len = 0;
    //Debug_LOG_INFO("getSurroundingObjectsData: variable Initialization");

    //for(int i = 0; i< lidar_points_len; i++){
    //        Debug_LOG_INFO("getSurroundingObjectsData: %f %f %f", 
    //            lidar_points[3 * i],
    //            lidar_points[3 * i + 1],
    //            lidar_points[3 * i + 2]);
    //    }
    

    while(lidar_points_len > 0){
        float highestZ = lidar_points_pos[2];
        //Debug_LOG_INFO("highestZ: %f", highestZ);
        //Debug_LOG_INFO("lidar_points_len before: %i", lidar_points_len);
        for(;lidar_points_pos[2] == highestZ && lidar_points_len > 0; lidar_points_pos += 3){
            lidar_points_len--;
        }
        //Debug_LOG_INFO("lidar_points_len after: %i", lidar_points_len);
        //lidar_points_pos zeigt jetzt auf ein element nach den Punkten auf dieser Scanebene
        //lidar_points zeigt auf das erste Element dieser Scanebene

        int local_objects_coordinates_len = 0;
        float * objects_coordinates = getObjectPositionsInPointcloud(lidar_points, (lidar_points_pos - lidar_points)/3, &local_objects_coordinates_len);

        //for(int i = 0; i< local_objects_coordinates_len; i++){
        //    Debug_LOG_INFO("getSurroundingObjectsData: Objects in Layer: %f %f %f", 
        //        objects_coordinates[3 * i],
        //        objects_coordinates[3 * i + 1],
        //        objects_coordinates[3 * i + 2]);
        //}

        memcpy(objects_coordinates_pos, objects_coordinates, local_objects_coordinates_len * sizeof(float) * 3);
        //copy discovered coordinates

        objects_coordinates_len += local_objects_coordinates_len;
        objects_coordinates_pos += local_objects_coordinates_len * 3;
        free(objects_coordinates);
        lidar_points = lidar_points_pos;
        
        //Debug_LOG_INFO("getSurroundingObjectsData: new layer");
        }
        //Debug_LOG_INFO("getSurroundingObjectsData: Number of total objects detected in all scan layers: %i'", objects_coordinates_len);
        

        //now we have the number of the objects and their coordinates

        objects_coordinates_pos = objects_coordinates_per_level + (3 * (objects_coordinates_len - 1)); // points to the last element of the array
        float * objects_coordinates_level_pos = objects_coordinates_pos;

        float * object_data = (float*) calloc(objects_coordinates_len, sizeof(float) * 3);
        int object_data_len = 0;
        //float * end_of_objects_coordinates_per_level = objects_coordinates_per_level + 3 * objects_coordinates_len;// points to one element past the end

        while(objects_coordinates_level_pos >= objects_coordinates_per_level ){

            for(;objects_coordinates_pos >= objects_coordinates_per_level && objects_coordinates_pos[2] == objects_coordinates_level_pos[2]; objects_coordinates_pos -= 3){
                bool found = false;

                for(int i =0; i<object_data_len; i++){

                    if((float)distance(objects_coordinates_pos[0],objects_coordinates_pos[1], (object_data + 3 * i)) < max_distance){
                        ((object_data + 3 * i)[0]) = (objects_coordinates_pos[0] + ((object_data + 3 * i)[0]))/2;
                        ((object_data + 3 * i)[1]) = (objects_coordinates_pos[1] + ((object_data + 3 * i)[1]))/2;
                        found = true;
                        break;
                    }
                }

                if(!found){
                    //Debug_LOG_INFO("getSurroundingObjectsData: new independent point discovered: %f %f %f", objects_coordinates_pos[0], objects_coordinates_pos[1], objects_coordinates_pos[2]);
                    memcpy((object_data + (3 * object_data_len)), objects_coordinates_pos, 3 * sizeof(float));
                    object_data_len ++;
                }
            }
            //Debug_LOG_INFO("getSurroundingObjectsData: total objects for this layer: %i\n", object_data_len);
            objects_coordinates_level_pos = objects_coordinates_pos;
        }

        //now object_data should contain all the position and heights of

    

    //object_coordinates_per_level now contains all the calculated object positons separated by scanning level
    *return_len = object_data_len;
    return object_data;
}
