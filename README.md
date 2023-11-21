# lidarview
A simple lidar viewer in OpenGL 1.x to support understanding lidar data and classification. 


Input:  A lidar point cloud in txt form,  obtained from a .las or .laz file with 'pdal translate'. 

```
pdal translate --writers.text.order="X,Y,Z,ReturnNumber,NumberOfReturns,Classification"
          --writers.text.keep_unspecified="false"  file.las  file.txt
```
Note: using LAStools:las2txt `las2txt -o file.las -o file.txt -parse xyznrc` gives different format and will need some adjustments


Has options to filter by first and last return, and number of returns; has options to filter by classification codes (ground, building, vegetation and other).


