


/* 
        Filters a raw pointCloudList for their an exact z value only and casts to float.
        Args:
            self
            pointcloudList ([[str]])    : List of Points that we want to filter on
            zFilter         (str)       : Z value on that we want to filter on, should be a float as a string

        Returns:
            pointcloudList ([[float, float, float]])    : Filtered list

*/


static exactFilterPoints(float** pointcloud, int pointcloudLen, float zFilter){

    float filteredPointcloud[pointcloudLen][3];

    int filteredIndex = 0;
    for(int i = 0; i<pointcloudLen; i++){
        if(pointcloud[i][2] == zFilter){
            filteredPointcloud[filteredIndex][0] = pointcloud[i][0];
            filteredPointcloud[filteredIndex][1] = pointcloud[i][1];
            filteredPointcloud[filteredIndex][2] = pointcloud[i][2];
            filteredIndex++;
        }
    }

    float (*filteredPointcloud[3]) = malloc(sizeof(*filteredPointcloud) * pointcloudLen);
    return filteredPointcloud, filteredIndex + 1;
}

int main(int argc, char *argv[]) {
    int array[1][1];
    array[0] = 5;
    int pointCloudLen = 5;
    float zFilter = 4.0;
    float pointCloud[5][3] = {
        {1.0, 2.0, 4.0},
        {1.1, 2.1, 4.0},
        {1.2, 2.2, 4.1},
        {1.3, 2.3, 3.9},
        {1.4, 2.4, 4.0}
    };

    exactFilterPoints(pointCloud, pointCloudLen, zFilter);

}