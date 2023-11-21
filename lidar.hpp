#ifndef __CLASSIFY_HPP
#define  __CLASSIFY_HPP

#include <vector>
using namespace std; 




typedef struct _lidar_point {
  float x,y,z; 
  float intensity;

  int return_number; //the number of this return
  int nb_of_returns; //how many returns this pulse has

  int code;  //classification code read from file 
  
  int mycode; //classification code that we'll assign to this point

  //there's more info available for a point (such as
  //scanDirectionF;ag, EdgeOfFlightLine, etc) but we don't store it

} lidar_point;


typedef struct _lidar_data {

  vector<lidar_point> data; 

  //bounding box
  float  minx, maxx, miny, maxy, minz, maxz; 
  
} lidar_point_cloud; 



/*
  reads lidar points from file and  populates points
  
  NOTE: file.txt must be obtained from file.las with 'pdal translate'
  reads the points from file in global array points
*/
void read_lidar_from_file(char* fname, lidar_point_cloud* lp); 


//adds point p  to  points
void lidar_add_point(lidar_point_cloud* lp, lidar_point p); 


//returns size (= nb points) 
static inline int size(lidar_point_cloud points) {
  return points.data.size();
}




/* ************************************************************ 
lidar classification codes

0 never classified 
1 unassigned 
2 ground 
3 low vegetation 
4 medium vegetation 
5 high vegetation
6 building 
7 low point (noise)
8 model key-point (mass point)
9 water
10 railroad
11 road
12 overlap 
13 wire-guard (shield)
14 wire-conductor (phase)
15 transmission tower
17 bridge
18 hight point (noise)
19-255 reserved for asprs definition
*/

/* for every point p, it sets p.mycode to one of the codes above */
void classify(lidar_point_cloud & points);


#endif 
