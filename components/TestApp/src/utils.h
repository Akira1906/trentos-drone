#ifndef UTILS_H 
#define UTILS_H

float * getObjectPositionsInPointcloud(float * points, int number_of_points, int * n_middle_points);
float * getClosestObjectMiddlePoint(float * middle_points, int n_middle_points, float * lidar_position);

#endif