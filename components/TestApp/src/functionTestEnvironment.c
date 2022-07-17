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
float* exactFilterPoints(float (*pointcloudBegin)[3], float (*pointcloudEnd)[3], float zFilter){
    float (*pos)[3] = pointcloudBegin;
    //Filter for zFilter z value
    for(; pointcloudBegin != pointcloudEnd; ++pointcloudBegin){
        if((*pointcloudBegin)[2] == zFilter){
            (*pos)[0] = (*pointcloudBegin)[0];
            (*pos)[1] = (*pointcloudBegin)[1];
            (*pos)[2] = (*pointcloudBegin)[2];
            pos ++;
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
float* roughlyFilterHighestPoints(float (*pointcloudBegin)[3], float (*pointcloudEnd)[3], float accuracy){
    float (*pos)[3] = pointcloudBegin;
    float lowestZ = 0;
    //Determine lowest z value
    for(; pos != pointcloudEnd; ++pos){
        if((*pos)[2]<lowestZ){
            lowestZ = (*pos)[2];
        }
    }
    //Filter for lowestZ with accuracy
    pos = pointcloudBegin;
    for(; pointcloudBegin != pointcloudEnd; ++pointcloudBegin){
        if((*pointcloudBegin)[2] <= lowestZ + accuracy){
            (*pos)[0] = (*pointcloudBegin)[0];
            (*pos)[1] = (*pointcloudBegin)[1];
            (*pos)[2] = (*pointcloudBegin)[2];
            pos ++;
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
    float (*pointcloudPointer)[3] = pointcloud1;
    float* pointcloudEnd = exactFilterPoints(pointcloudPointer, pointcloudPointer + pointcloudLen, zFilter);

    int arrayLength = ((float*)pointcloudEnd - (float*)pointcloudPointer)/3 * sizeof(*pointcloudPointer);
    
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
    pointcloudPointer = pointcloud2;
    pointcloudEnd = roughlyFilterHighestPoints(pointcloudPointer, pointcloudPointer + pointcloudLen, accuracy);

    arrayLength = ((float*)pointcloudEnd - (float*)pointcloudPointer)/3 * sizeof(*pointcloudPointer);
    
    float *filteredPointcloud2 = (float *) malloc(arrayLength);
    int filteredPointcloud2Len = arrayLength/(3 * 4);
    if(filteredPointcloud2 == NULL)
        exit(1);
    if(NULL == memcpy(filteredPointcloud2, pointcloudPointer, arrayLength))
        exit(1);
    //free old array


    //------------------------
    free(filteredPointcloud1);
    free(filteredPointcloud2);


}
