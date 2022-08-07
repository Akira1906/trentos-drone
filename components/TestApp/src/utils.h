#ifndef UTILS_H 
#define UTILS_H

float * getClosestObjectMiddlePoint(float * middle_points, int n_middle_points, float * lidar_position);
int roughlyFilterHighestPoints(float * points, int length, float accuracy);
float getLowestZ(float * points, int length);
float * exactFilterPoints(float  *pointcloudBegin, float *pointcloudEnd, float zFilter);
float * getSurroundingObjectsData (float * lidar_points, int lidar_points_len, int * return_len);

#endif