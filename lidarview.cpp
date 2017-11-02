/* lidarview file.txt 

   Reads a lidar point cloud in txt form and renders the points in
   3D. Has options to filter by first and last return, and number of
   returns; has options to filter by classification codes (ground,
   building, vegetation and other).

   The lidar file is obtained from a .las or .laz file with
   LAStools:las2txt; 
   
   last2txt -o file.las -o file.txt -parse xyznrc

   NOTE: -parse xyznrc in this order

   keypress: 

   l/r/u/d/f/bx/X,y/Y,z/Z: translate and rotate
   w: toggle wire/filled polygons
   v,g,h,o: toggle veg, ground, buildings,other on/off
   c: cycle through colormaps (one color, based on code, based on your code)
   t: cycle through filter  options: first-return, lsat return, many-returns, all-returns

   OpenGL 1.x
   Laura Toma
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
//this allows this code to compile both on apple and linux platforms
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <vector>

using namespace std; 




typedef struct _lidarPoint {
  float x,y,z; 
  int nb_of_returns; //how many returns this pulse has
  int return_number; //the number of this return
  int code;  //classification code read from file 
  
  int mycode; //classification code assigned by us 
} lidarPoint;


//the array of lidar points; note: this needs to be global because it
//needs to be rendered
vector<lidarPoint>  points;

//bounding box of the points, updated when the terrain is loaded from file
float minx, maxx, miny, maxy, minz, maxz; 

const int WINDOWSIZE = 500; 

//whenever the user rotates and translates the scene, we update these
//global translation and rotation
GLfloat pos[3] = {0,0,0};
GLfloat theta[3] = {0,0,0};

// draw polygons line or filled. This will be used when rendering the
// surface.
GLint fillmode = 0; 



/* ************************************************************ */
/* FILTERING POINTS BY THEIR RETURN SITUATION */
/* A LiDAR point has a return number and a number of returns (for its
   pulse).  A pulse may get several returns over vegetation, lets say
   3, and this will result in three points:

   return number/number of returns
   1/3 x y z
   2/3 x y z
   3/3 x y z 
   note the x,y,z may be different for different return numbers 

   For bare earth, you will only have one return: 
   1/1


   If ALL_RETURN, all points  are included
   IF FIRST_RETURN, only the first returns are included, ie points with return number=1

   If LAST_RETURN, only the last returns are included, i.e points with
   1/1 and points with 2/2 or 3/3 or 4/4
*/
#define ALL_RETURN 0 
#define FIRST_RETURN 1
#define LAST_RETURN 2
#define MORE_THAN_ONE_RETURN 3
#define NB_WHICH_RETURN_OPTIONS 4
// WHICH_RETURN  cycles through all options via keypress 'm'
int WHICH_RETURN = ALL_RETURN; 



/* ************************************************************ */
/* FILTERING POINTS BY THEIR classification code */
/* 
lidar classification codes  [from ...somewhere on internet]

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

//These are used by render() to decide  what points to render.
//By default draw everything; each one of these classes can be toggled
//on/off in keypress()
int GROUND=1; 
int VEG=1; 
int BUILDING=1; 
int OTHER=1; 




/* **************************************** */
/* chosing a color map: 

   If COLORMAP == ONE_COLOR:  draw all filtered points in one color 

   If COLORMAP == CODE_COLOR: draw all filtered points with a color
   based on their classification code p.code which was read from the
   las file

   If COLORMAP == MYCODE_COLOR:  draw all filtered points with a color
   based on their classification code p.mycode which we computed 

   COLORMAP starts by default as ONE_COLOR and cycles through all
   options via keypress 'c'.
*/
#define ONE_COLOR 0  
#define CODE_COLOR 1 
#define MYCODE_COLOR 2
#define NB_COLORMAP_CHOICES 3
int COLORMAP = ONE_COLOR; 



//predefine some colors for convenience
GLfloat red[3] = {1.0, 0.0, 0.0};
GLfloat green[3] = {0.0, 1.0, 0.0};
GLfloat blue[3] = {0.0, 0.0, 1.0};
GLfloat black[3] = {0.0, 0.0, 0.0};
GLfloat white[3] = {1.0, 1.0, 1.0};
GLfloat gray[3] = {0.5, 0.5, 0.5};
GLfloat yellow[3] = {1.0, 1.0, 0.0};
GLfloat magenta[3] = {1.0, 0.0, 1.0};
GLfloat cyan[3] = {0.0, 1.0, 1.0};

/* from https://www.opengl.org/discussion_boards/showthread.php/132502-Color-tables  */
GLfloat brown[3] = { 0.647059, 0.164706, 0.164706}; 
GLfloat DarkBrown[3] = { 0.36, 0.25, 0.20}; 
GLfloat DarkTan[3] = { 0.59, 0.41, 0.31};
GLfloat Maroon[3]= { 0.556863, 0.137255, 0.419608}; 
GLfloat DarkWood[3] = { 0.52, 0.37, 0.26}; 

GLfloat  Copper[3] = { 0.72,  0.45,  0.20};

GLfloat green1[3] = {.5, 1, 0.5};
GLfloat green2[3] = {0.0, .8, 0.0};
GLfloat green3[3] = {0.0, .5, 0.0};
GLfloat ForestGreen[3] = { 0.137255, 0.556863, 0.137255};
GLfloat MediumForestGreen[3] = { 0.419608 , 0.556863 , 0.137255}; 
GLfloat LimeGreen[3] ={ 0.196078,  0.8 , 0.196078}; 

GLfloat Orange[3] = { 1, .5, 0}; 
GLfloat Silver[3] = { 0.90, 0.91, 0.98};
GLfloat Wheat[3] = { 0.847059 , 0.847059, 0.74902}; 





/* forward declarations of functions */
void display(void);
void keypress(unsigned char key, int x, int y);

void draw_points(); 
void draw_xy_rect(GLfloat z, GLfloat* col); 
void draw_xz_rect(GLfloat y, GLfloat* col); 
void draw_yz_rect(GLfloat x, GLfloat* col); 
void cube(GLfloat side); 
void filledcube(GLfloat side); 
void draw_axes(); 

//reads the points from file in global array points
void readPointsFromFile(char* fname);




/* NOTE: file.txt must be obtained from file.las with las2txt with
   -parse xyznrc in this order

   last2txt -o file.las -o file.txt -parse xyznrc
 */
//reads the points from file in global array points
void readPointsFromFile(char* fname) {
  
  FILE* file = fopen(fname, "r"); 
  if (!file) {
    printf("cannot open file %s\n",  fname);
    exit(1); 
  }

  lidarPoint p; 
  while (1) {
    //-parse xyzcr
    if (fscanf(file, "%f %f %f %d %d %d", 
	       &p.x, &p.y, &p.z, &p.nb_of_returns, &p.return_number, &p.code) <6)  break; 
    
    //else: if we are here,  we got another point
    //printf("reading %f, %f, %f\n", p.x, p.y, p.z); 
    
    //insert the point in points[] array 
    p.mycode=0; //everything unclassified
    points.push_back(p); 
    
    //update bounding box
    if (points.size() == 1) {
      minx=maxx = p.x; 
      miny=maxy = p.y; 
      minz=maxz = p.z; 
    } else {
      if (minx > p.x) minx=p.x; 
      if (maxx < p.x) maxx = p.x; 
      if (miny > p.y) miny=p.y; 
      if (maxy < p.y) maxy = p.y; 
      if (minz > p.z) minz=p.z; 
      if (maxz < p.z) maxz = p.z; 
    }
  } //while 
  
  fclose(file); 

  //print info 
  printf("total %d points in  [%f, %f], [%f,%f], [%f,%f]\n", 
	 (int)points.size(), minx, maxx, miny,maxy, minz, maxz); 
}



int main(int argc, char** argv) {

  //read number of points from user
  if (argc!=2) {
    printf("usage: %s file.txt\n", argv[0]);
    exit(1); 
  }
  //this allocates and initializes the array that holds the points
  readPointsFromFile(argv[1]); 


  /* OPEN GL STUFF */
  /* open a window and initialize GLUT stuff */
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(WINDOWSIZE, WINDOWSIZE);
  glutInitWindowPosition(100,100);
  glutCreateWindow(argv[0]);

  /* register callback functions */
  glutDisplayFunc(display); 
  glutKeyboardFunc(keypress);
  
  /* OpenGL init */
  /* set background color black*/
  glClearColor(0, 0, 0, 0);  
  glEnable(GL_DEPTH_TEST); 

  /* setup the camera (i.e. the projection transformation) */ 
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60, 1 /* aspect */, 1, 10.0); /* the frustrum is from z=-1 to z=-10 */
  /* camera is at (0,0,0) looking along negative z axis */
  
  //initialize the translation to bring the points in the view frustrum which is [-1, -10]
  pos[2] = -2;

  //initialize rotation to look at it from above 
  theta[0] = -45; 

  
  /* start the event handler */
  glutMainLoop();

  return 0;
}




/* this function is called whenever the window needs to be rendered */
void display(void) {

  //clear the screen
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //clear all modeling transformations 
  glMatrixMode(GL_MODELVIEW); 
  glLoadIdentity();

  /* The default GL window is x=[-1,1], y= [-1,1] with the origin in
     the center.  The view frustrum was set up from z=-1 to z=-10. The
     camera is at (0,0,0) looking along negative z axis.
  */ 

 /* First we translate and rotate our local reference system with the
    user transformation. pos[] represents the cumulative translation
    entered by the user, and theta[] the cumulative rotation entered
    by the user */
  glTranslatef(pos[0], pos[1], pos[2]);  
  glRotatef(theta[0], 1,0,0); //rotate theta[0] around x-axis, etc 
  glRotatef(theta[1], 0,1,0);
  glRotatef(theta[2], 0,0,1);
  
  /* We translated the local reference system where we want it to be; now we draw the
     object in the local reference system.  */
  draw_points();  
    
  //don't need to draw a cube but I found it nice for perspective 
  cube(1); //draw a cube of size 1

  glFlush();
}



/* this function is called whenever  key is pressed */
void keypress(unsigned char key, int x, int y) {

  switch(key) {

  case '2': 
    //3d orthogonal projection, view from straight above
    glMatrixMode(GL_PROJECTION);
    //the view frustrum is z=[0, -20]
    glOrtho(-1, 1, -1, 1, 0,-20); //left, right, top, bottom, near, far
    
    //initial view is from (0,0,-5) ie above the terrain looking straight down
    pos[0]=pos[1]=0; pos[2] = -7; 
    //initial view: no rotation
    theta[0]=theta[1] = theta[2]= 0; 
    glutPostRedisplay();
    break;
  
  case '3': 
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, 1 /* aspect */, 1, 10.0); /* the frustrum is from z=-1 to z=-10 */
    /* camera is at (0,0,0) looking along negative z axis */
    //initialize the translation; view frustrum is z= [-1, -10] and
    //initial position (0,0,-2)
    pos[0]=pos[1]=0; pos[2] = -2;
    //initialize rotation to look  from above 
    theta[1] = theta[2] = 0;  theta[0] = -45; 
    glutPostRedisplay();
    break;

  case 'c': 
    //change the colormap  
    COLORMAP= (COLORMAP+1) % NB_COLORMAP_CHOICES; 
    switch (COLORMAP) {
    case ONE_COLOR: 
      printf("colormap: one color\n"); 
      break; 
    case CODE_COLOR: 
      printf("colormap: by code\n"); 
      break; 
    case MYCODE_COLOR: 
      printf("colormap: by mycode\n"); 
      break; 
    default: 
      printf("colormap: unknown. oops, something went wrong.\n"); 
      exit(1); 
    }
    glutPostRedisplay();
    break;

  case 't': 
    //render points with one/more than one return 
    WHICH_RETURN = (WHICH_RETURN + 1) % NB_WHICH_RETURN_OPTIONS; 
    switch (WHICH_RETURN) {
    case ALL_RETURN: 
      printf("draw all returns\n"); 
      break; 
    case FIRST_RETURN: 
      printf("draw only first return (i.e.points with return_number=1)\n"); 
      break; 
    case LAST_RETURN: 
      printf("draw only last return (i.e. points with return_number = number_of_returns)\n");
      break; 
    case MORE_THAN_ONE_RETURN: 
      printf("draw only points that has >1 returns\n"); 
      break; 
    default: 
      break; 
    }
    glutPostRedisplay();
    break;

  case 'g': 
    //toggle off rendering ground points   (code=2)
    GROUND = !GROUND; 
    glutPostRedisplay();
    break;

  case 'v': 
    //toggle off rendering vegetation points  (code=3,4,5)
    VEG = !VEG; 
    glutPostRedisplay();
    break;

  case 'h':
    //toggle off rendering building points  (code=6)
    BUILDING = !BUILDING; 
    glutPostRedisplay();
    break;

  case 'o': 
    //toggle off rendering "other" ie points that are not ground, vegetation or building 
    OTHER=!OTHER; 
    glutPostRedisplay();
    break;

    //ROTATIONS 
  case 'x':
    theta[0] += 5.0; 
    glutPostRedisplay();
    break;
  case 'y':
    theta[1] += 5.0;
    glutPostRedisplay();
    break;
  case 'z':
    theta[2] += 5.0;
    glutPostRedisplay();
    break;
  case 'X':
    theta[0] -= 5.0; 
    glutPostRedisplay();
    break;
  case 'Y':
    theta[1] -= 5.0; 
    glutPostRedisplay();
    break;
  case 'Z':
    theta[2] -= 5.0; 
    glutPostRedisplay();
    break;
    
    //TRANSLATIONS 
    //backward (zoom out)
  case 'b':
    pos[2] -= 0.1; 
    glutPostRedisplay();
    break;
    //forward (zoom in)
  case 'f':
    pos[2] += 0.1; 
    //glTranslatef(0,0, 0.5);
    glutPostRedisplay();
    break;
    //down 
  case 'd': 
     pos[1] -= 0.1; 
    //glTranslatef(0,0.5,0);
    glutPostRedisplay();
    break;
    //up
  case 'u': 
    pos[1] += 0.1; 
    //glTranslatef(0,-0.5,0);
    glutPostRedisplay();
    break;
    //left 
  case 'l':
    pos[0] -= 0.1; 
    glutPostRedisplay();
    break;
    //right
  case 'r':
    pos[0] += 0.1; 
    glutPostRedisplay();
    break;

    //fillmode 
  case 'w': 
    fillmode = !fillmode; 
     glutPostRedisplay();
    break;

  case 'q':
    exit(0);
    break;
  } 
}//keypress









/* x is a value in [minx, maxx]; it is mapped to [-1,1] */
GLfloat xtoscreen(GLfloat x) {
  //return (-1 + 2*x/WINDOWSIZE); 
  return (-1 + 2*(x-minx)/(maxx-minx)); 
}


/* y is a value in [miny, maxy]; it is mapped to [-1,1] */
GLfloat ytoscreen(GLfloat y) {
  return (-1 + 2*(y-miny)/(maxy-miny)); 
}

/* z is a value in [minz, maxz]; it is mapped so that [0, maxz] map to [0,1] */
GLfloat ztoscreen(GLfloat z) {
    return (-1 + 2*(z-minz)/(maxz-minz))/1.5; 
}


//set color based on p.code
void setColorByCode(lidarPoint p) {
  switch (p.code) {
  case 0: //never classified
    glColor3fv(yellow); 
    break; 
  case 1: //unnasigned 
    glColor3fv(Orange); 
    break; 
  case 2: //ground 
    //    glColor3fv(DarkWood); 
    glColor3fv(DarkBrown); 
    break; 
  case 3: //low vegetation 
    glColor3fv(LimeGreen); 
    break;
  case 4: //medium vegetation 
    glColor3fv(MediumForestGreen); 
    break;
  case 5: //high vegetation 
    glColor3fv(ForestGreen); 
    break;
  case 6: //building 
    //glColor3fv(Tan); 
    glColor3fv(Copper); 
    break;
  case 7: //noise
    glColor3fv(magenta); 
    break;
  case 8: //reserved 
    glColor3fv(white); 
    break;
  case 9: //water 
    glColor3fv(blue); 
    break;
  case 10: //rail 
    glColor3fv(gray); 
    break;
  case 11: //road surface 
    glColor3fv(gray); 
    break;
  case 12:  //reserved
    glColor3fv(white); 
    break;
  case 13: 
  case 14: //wire
    glColor3fv(gray); 
    break;
  case 15: //transmission tower
    glColor3fv(Wheat); 
    break;
  case 16: //wire 
  case 17: //bridge deck 
    glColor3fv(blue); 
    break;
  case 18: //high noise
    glColor3fv(magenta); 
    break;
  default: 
    printf("panic: encountered unknown code >18"); 
  }
} //setColorByCode



//put your own colormap here  based on p.mycode
void setColorByMycode(lidarPoint p) {

  glColor3fv(blue);
}

//draw everything with one color 
void  setColorOneColor(lidarPoint p) {

   glColor3fv(yellow); //yellow should be a constant...
  return; 
}



//point p has passed all the filters and must be rendered. Set its
//color. 
void setColor(lidarPoint p) { 

  if (COLORMAP == ONE_COLOR) {
    //draw all points with same color 
    setColorOneColor(p); 
 
  } else if (COLORMAP == MYCODE_COLOR) {
    setColorByCode(p); 
  
  } else if (COLORMAP == CODE_COLOR) {
    setColorByMycode(p); 
  
  } else {
    printf("unkown colormap options.oops.\n");
    exit(1); 
  }
} //setColor()






/* ****************************** */
/* Draw the array of points stored in global variable points[].  

   NOTE: The points are in the range x=[minx, maxx], y=[miny,
   maxy], z=[minz, maxz] and they must be mapped into
   x=[-1,1], y=[-1, 1], z=[-1,1]
  */
void draw_points(){
  

  //else, NOCOLOR must be 0, so we render with color based on codes 
  glBegin(GL_POINTS); 
  int i;
  lidarPoint p; 
  for (i=0; i < points.size(); i++) {
   
    p = points[i]; 
    
    //FIRST FILTER BY RETURN
    if (WHICH_RETURN==FIRST_RETURN) // we only want the first returns
      if (p.return_number!=1) continue;
    if (WHICH_RETURN==LAST_RETURN) // we only want the last returns
      if (p.return_number !=p.nb_of_returns) continue;
    if (WHICH_RETURN==MORE_THAN_ONE_RETURN) //we only want pulses that had more than 1 return 
      if (p.nb_of_returns ==1) continue;
    //if (WHICH_RETURN==ALL_RETURN)  // we want all points so keep going 


    //all the points that made it here must have the right nb of returns that we want 

    //NEXT FILTER BY CODE
    //if this point is 2 and we dont want to render ground, skip it 
    if (p.code == 2 && !GROUND )  continue; 

    //if this point is 3,4,5 and we don't want to draw the vegetation,skip it
    if ((p.code == 3||p.code == 4||p.code == 5) && !VEG)  continue; 

    //if this point if 6 and we don't want to draw teh buildings, skip it 
    if (p.code == 6 && !BUILDING) continue; 

    //if this point is "other" and we don't want to draw "other" skip it 
    if ((p.code == 0 || p.code ==1 || p.code >6) && !OTHER) continue; 
    
    //all the points that made it here must be drawn
    
    
    //set the color of this point; 
    setColor(p); 
   

    glVertex3f(xtoscreen(points[i].x),
	       ytoscreen(points[i].y), 
	       ztoscreen(points[i].z)); 
  }
  
  glEnd(); 
}//draw_points




//draw a square x=[-side,side] x y=[-side,side] at depth z
void draw_xy_rect(GLfloat z, GLfloat side, GLfloat* col) {

  glColor3fv(col);
  glBegin(GL_POLYGON);
  glVertex3f(-side,-side, z);
  glVertex3f(-side,side, z);
  glVertex3f(side,side, z);
  glVertex3f(side,-side, z);
  glEnd();
}


//draw a square y=[-side,side] x z=[-side,side] at given x
void draw_yz_rect(GLfloat x, GLfloat side, GLfloat* col) {
  
  glColor3fv(col);
  glBegin(GL_POLYGON);
  glVertex3f(x,-side, side);
  glVertex3f(x,side, side);
  glVertex3f(x,side, -side);
  glVertex3f(x,-side, -side);
  glEnd();
}


//draw a square x=[-side,side] x z=[-side,side] at given y
void draw_xz_rect(GLfloat y, GLfloat side, GLfloat* col) {

  glColor3fv(col);
  glBegin(GL_POLYGON);
  glVertex3f(-side,y, side);
  glVertex3f(-side,y, -side);
  glVertex3f(side,y, -side);
  glVertex3f(side,y, side);
  glEnd();
}

//draw a cube 
void cube(GLfloat side) {
  GLfloat f = side, b = -side;
 
  if (fillmode) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  } else {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  }


  /* back face  BLUE*/
  draw_xy_rect(b,side, blue);
 /* front face  RED*/
  draw_xy_rect(f,side, red);
  /* side faces  GREEN*/
  draw_yz_rect(b, side, green);
  draw_yz_rect(f, side, green);
  //up, down faces missing to be able to see inside 

  /* middle z=0 face CYAN*/
  draw_xy_rect(0, side, cyan);
  /* middle x=0 face WHITE*/
  draw_yz_rect(0,side, gray);
  /* middle y=0 face  pink*/
  draw_xz_rect(0, side, magenta);
}



//draw a filled cube  [-side,side]^3
void filledcube(GLfloat side) {
  
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  
  /* back, front faces */
  draw_xy_rect(-side,side, yellow);
  draw_xy_rect(side,side, yellow);
  
  /* left, right faces*/
  draw_yz_rect(-side, side, yellow);
  draw_yz_rect(side, side, yellow);
  
  /* up, down  faces  */
  draw_xz_rect(side,side, yellow);
  draw_xz_rect(-side,side, yellow);
}
