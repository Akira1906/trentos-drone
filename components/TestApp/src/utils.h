#ifndef UTILS_H 
#define UTILS_H

float * getObjectPositionsInPointcloud(float * points, int number_of_points, int * n_middle_points);
float * getClosestObjectMiddlePoint(float * middle_points, int n_middle_points, float * lidar_position);
float * roughlyFilterHighestPoints(float *pointcloudBegin, float *pointcloudEnd, float accuracy);
float * exactFilterPoints(float  *pointcloudBegin, float *pointcloudEnd, float zFilter);
float * getSurroundingObjectsData (float * lidar_points, int lidar_points_len, int * return_len);

#endif