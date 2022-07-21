#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* 
        Filters a 2-dimensional pointcloud array for their z value exactly. The array is filtered by overriding the old
        array with the filtered elements. The functions returns a pointer to one past the end of this now filtered array.
        Args:
            float (*pointcloudBegin)[3]     : Pointer to the first element of the pointcloud array
            float (*pointcloudEnd)[3]       : Pointer to one past the last pointcloud array element    
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
            float (*pointcloudBegin)[3]     : Pointer to the first element of the pointcloud array
            float (*pointcloudEnd)[3]       : Pointer to one past the last pointcloud array element
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

int main(int argc, char *argv[]) {
    //--------------------------
    //USAGE/TEST exactFilterPoints

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


}
