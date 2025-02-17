#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "utils.h"

#define MAX_DIST 12
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

    while(lidar_points_len > 0){
        float highestZ = lidar_points_pos[2];

        for(;lidar_points_pos[2] == highestZ; lidar_points_pos += 3){
            lidar_points_len--;
        }
        //lidar_points_pos zeigt jetzt auf ein element nach den Punkten auf dieser Scanebene

        int local_objects_coordinates_len = 0;
        float * objects_coordinates = getObjectPositionsInPointcloud(lidar_points, (lidar_points_pos - lidar_points)/3, &local_objects_coordinates_len);

        memcpy(objects_coordinates_pos, objects_coordinates, local_objects_coordinates_len * sizeof(float) * 3);
        //copy discovered coordinates

        objects_coordinates_len += local_objects_coordinates_len;
        objects_coordinates_pos += local_objects_coordinates_len * 3;
        free(objects_coordinates);
        lidar_points = lidar_points_pos;
        }
        
        
        //now we have the number of the objects and their coordinates

        objects_coordinates_pos = objects_coordinates_per_level + (3 * (objects_coordinates_len - 1)); // points to the last element of the array
        float * objects_coordinates_level_pos = objects_coordinates_pos;

        float * object_data = (float*) calloc(objects_coordinates_len, sizeof(float) * 3);
        int object_data_len = 0;
        float * end_of_objects_coordinates_per_level = objects_coordinates_per_level + 3 * objects_coordinates_len;// points to one element past the end

        while(objects_coordinates_level_pos >= objects_coordinates_per_level ){

            for(;objects_coordinates_pos >= objects_coordinates_per_level && objects_coordinates_pos[2] == objects_coordinates_level_pos[2]; objects_coordinates_pos -= 3){
                bool found = false;

                for(int i =0; i<object_data_len; i++){
                    //float x = objects_coordinates_pos[0] - ((object_data + 3 * i)[0]);
                    //float y = objects_coordinates_pos[1] - ((object_data + 3 * i)[1]);

                    if((float)distance(objects_coordinates_pos[0],objects_coordinates_pos[1], (object_data + 3 * i)) < max_distance){
                        ((object_data + 3 * i)[0]) = (objects_coordinates_pos[0] + ((object_data + 3 * i)[0]))/2;
                        ((object_data + 3 * i)[1]) = (objects_coordinates_pos[1] + ((object_data + 3 * i)[1]))/2;
                        found = true;
                        break;
                    }
                }

                if(!found){
                    memcpy((object_data + (3 * object_data_len)), objects_coordinates_pos, 3 * sizeof(float));
                    object_data_len ++;
                }
            }

        objects_coordinates_level_pos = objects_coordinates_pos;
        }

        //now object_data should contain all the position and heights of

    

    //object_coordinates_per_level now contains all the calculated object positons separated by scanning level
    *return_len = object_data_len;
    return object_data;
}


int main(int argc, char *argv[]) {

 float pointcloud3[9][3] = {
        {1.0, 2.0, -3.9},
        {1.1, 2.1, -3.9},
        {1.0, 2.3, -3.9},
        {1.1, 2.4, -3.9},
        {1.0, 20.0, -3.9},
        {1.1, 21.0, -3.9},
        {1.2, 2.2, -4.0},
        {1.3, 20.3, -4.0},
        {1.4, -10.0, -4.2}
    };
int * object_data_len;

float * objectData = getSurroundingObjectsData((float *)pointcloud3, 9, object_data_len);



    //--------------------------
    //USAGE/TEST exactFilterPoints
/*
    int pointcloudLen = 5;
    float zFilter = -4.0;
    float pointcloud1[5][3] = {
        {1.0, 2.0, -3.9},
        {1.1, 2.1, -4.1},
        {1.2, 2.2, -4.0},
        {1.3, 2.3, -4.0},
        {1.4, 2.4, -3.8}
    };
    float *pointcloudPointer = (float *)pointcloud1;
    float* pointcloudEnd = exactFilterPoints(pointcloudPointer, pointcloudPointer + (pointcloudLen*3), zFilter);

    int arrayLength = ((float*)pointcloudEnd - (float*)pointcloudPointer) * sizeof(*pointcloudPointer);
    
    float *filteredPointcloud1 = (float *) malloc(arrayLength);
    int filteredPointcloud1Len = arrayLength/(3 * 4);
    if(filteredPointcloud1 == NULL)
        exit(1);
    if(NULL == memcpy(filteredPointcloud1, pointcloudPointer, arrayLength))
        exit(1);
    //free old array

    //-------------------------
    //USAGE/TEST roughlyFilterHighestPoints
    
    pointcloudLen = 5;
    float accuracy = 0.1;
    float pointcloud2[5][3] = {
        {1.0, 2.0, -3.01},
        {1.1, 2.1, -4.11},
        {1.2, 2.2, -4.01},
        {1.3, 2.3, -4.0},
        {1.4, 2.4, -4.1}
    };
    float *pointcloudPointer2 = (float*) pointcloud2;
    pointcloudEnd = roughlyFilterHighestPoints(pointcloudPointer2, pointcloudPointer2 + (pointcloudLen*3), accuracy);

    int pointcloud2_len = ((float*)pointcloudEnd - (float*)pointcloudPointer2) /3;
    
    arrayLength = ((float*)pointcloudEnd - (float*)pointcloudPointer2) * sizeof(*pointcloudPointer2);
    float *filteredPointcloud2 = (float *) malloc(arrayLength);
    int filteredPointcloud2Len = arrayLength/(3 * 4);
    if(filteredPointcloud2 == NULL)
        exit(1);
    if(NULL == memcpy(filteredPointcloud2, pointcloudPointer, arrayLength))
        exit(1);
    //free old array
    accuracy = -accuracy;
    accuracy = fabs(accuracy);

    //------------------------
    free(filteredPointcloud1);
    free(filteredPointcloud2);
*/

   


}
