
#include "lidar.hpp"
#include <vector>
using namespace std; 




//adds point p  to  lp 
void lidar_add_point(lidar_point_cloud* lp, lidar_point p) {

  //make sure its a valid pointer 
  assert(lp);

  //add the point
  lp->data.push_back(p);
  
  //update bounding box
  if (size(*lp) == 1) {
    lp->minx = lp->maxx = p.x; 
    lp->miny = lp->maxy = p.y; 
    lp->minz =lp-> maxz = p.z; 
  } else {
    if (lp->minx > p.x) lp->minx=p.x; 
    if (lp->maxx < p.x) lp->maxx = p.x; 
    if (lp->miny > p.y) lp->miny=p.y; 
    if (lp->maxy < p.y) lp->maxy = p.y; 
    if (lp->minz > p.z) lp->minz=p.z; 
    if (lp->maxz < p.z) lp->maxz = p.z; 
  }
} 



/*
  reads lidar points from file and  populates points
  
  NOTE: file.txt must be obtained from file.las with 'pdal translate'
  reads the points from file in global array points
*/
void read_lidar_from_file(char* fname, lidar_point_cloud* points) {

  
  FILE* file = fopen(fname, "r"); 
  if (!file) {
    printf("read_lidar:from_file: cannot open file %s\n",  fname);
    exit(1); 
  }

  //read first line
  //"X","Y","Z","Intensity","ReturnNumber","NumberOfReturns",
  //"ScanDirectionFlag","EdgeOfFlightLine","Classification","ScanAngleRank",
  //"UserData","PointSourceId","GpsTime"
  char line[1000];
  if (fgets(line,sizeof line,file)!= NULL) /* read a line from a file */ {
    printf("%s",line); //print the file contents on stdout.
  } else {
    printf("read_lidar_fom_file: cannot read from file\n");
    exit(1); 
  } 
  
  lidar_point p;
  float retnb, nbret, classif; //dir, edge, angle, user, pid, gps;
  while (1) {
    /*
      if (fscanf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
	       &p.x, &p.y, &p.z,
	       &p.intensity, &retnb, &nbret,
	       &dir, &edge, &classif,
	       &angle, &user, &pid,
	       &gps
	       ) < 13)
    */
    if (fscanf(file, "%f,%f,%f,%f,%f,%f", &p.x, &p.y, &p.z,&retnb, &nbret, &classif) < 6)    { 
      //either an error or all done 
      break; 
    }
    //we got another point
    p.return_number = (int) retnb;
    p.nb_of_returns = (int)nbret;
    p.code=(int)classif;
    p.mycode=0; //everything unclassified
    
    //printf("point: x=%f, y=%f, z=%f, intensity=%f return_nb=%d nb_returns=%d classif=%d\n",
    //  p.x, p.y, p.z, p.intensity, p.return_number, p.nb_of_returns, p.code); 
	
    //insert the point 
    lidar_add_point(points, p); 
    
  } //while 

  //done reading points 
  fclose(file); 
  
  //print info about the points that were read 
  printf("read total %d points\n", (int)size(*points)); 
  printf("\tbounding box:  x=[%.2f, %.2f], y=[%.2f,%.2f], z=[%.2f,%f.2]\n",
	 points->minx, points->maxx, points->miny, points->maxy, points->minz, points->maxz); 

  
  return; 
}




/* ************************************************************ 
lidar classification codes

0 bever classified 
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
void classify(lidar_point_cloud & points) {

  //float minheight, max_height; 
  for (int i=0; i< size(points); i++) {

    points.data[i].mycode = 6; 
  } 
  
} 




