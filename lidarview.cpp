/* lidarview file.txt 

   Reads a lidar point cloud in txt form and renders the points in
   3D. Has options to filter by first and last return, and number of
   returns; has options to filter by classification codes (ground,
   building, vegetation and other).

   The lidar file is obtained from a .las or .laz file with 'pdal
   translate'
   
   NOte: using LAStools:las2txt `last2txt -o file.las -o file.txt
   -parse xyznrc` gives different format and will need some
   adjustments


   keypress: 

   l/r/u/d/f/bx/X,y/Y,z/Z: translate and rotate
   w: toggle wire/filled polygons
   v,g,h,o: toggle veg, ground, buildings,other on/off
   c: cycle through colormaps (one color, based on code, based on your code)
   t: cycle through filter  options: first-return, last return, many-returns, all-returns

   OpenGL 1.x
   Laura Toma
*/

#include "lidar.hpp"


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <GLUT/glut.h>

#include <vector>
using namespace std; 


//the lidar points. note: this needs to be global because it needs to
//be rendered
lidar_point_cloud lpoints;


// graphics 
const int WINDOWSIZE = 500; 

//pos[] and theta[] represent the translation and rotation on x,y and
//z axes, respectively. They represent how the user wants to look at
//the terrain. The user can change them through keypresses. To rotate,
//press x,X, y,Y, z,Z to rotate around x, y, z respectively. To move
//up/down, left.right, front/back
GLfloat pos[3] = {0,0,0};
GLfloat theta[3] = {0,0,0};


// draw polygons line or filled. This will be used when rendering the
// surface.
GLint fillmode = 0; 



/* ************************************************************ */
/* FILTERING POINTS BY THEIR RETURN */
/* A LiDAR point has a return number and a number of returns (for its
   pulse). Vegetation usually gives in >1 returns.  Bare earth and
   buildings have 1 return.

   If ALL_RETURN, all points  are included
   
   IF FIRST_RETURN, only the first returns are included, ie points with return_number=1

   If LAST_RETURN, only the last returns are included, i.e points with
   return_nb = nb_of_returns
*/


const int ALL_RETURN = 0;  
const int FIRST_RETURN = 1; 
const int LAST_RETURN = 2; 
const int MORE_THAN_ONE_RETURN = 3; 
const int ONE_RETURN = 4;

int which_return = ALL_RETURN; 


/* These are used to decide which points to render.  By default draw
   everything; each one of these flags can be toggled on/off in keypress()
*/
int RENDER_GROUND = 1; 
int RENDER_VEG = 1; 
int RENDER_BUILDING = 1; 
int RENDER_OTHER = 1; 




/* **************************************** */
/* chosing a color map: 

   If COLORMAP == ONE_COLOR:  draw points in one color 

   If COLORMAP == CODE_COLOR: draw points with a color based on p.code
   (which was read from the las file)

   If COLORMAP == MYCODE_COLOR: draw points with a color based on
   p.mycode (computed by us)

   COLORMAP starts by default as ONE_COLOR and cycles through all
   options via keypress 'c'.
*/
const int ONE_COLOR = 0;   
const int CODE_COLOR = 1; 
const int MYCODE_COLOR =2;
const int NB_COLORMAP_CHOICES =3;

//COLORMAP cycles through all choices  via keypress 'c'
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
GLfloat brown[3] = { 0.647059, 0.164706, 0.164706}; 
GLfloat DarkBrown[3] = { 0.36, 0.25, 0.20}; 
GLfloat ForestGreen[3] = { 0.137255, 0.556863, 0.137255};
GLfloat MediumForestGreen[3] = { 0.419608 , 0.556863 , 0.137255}; 
GLfloat LimeGreen[3] ={ 0.196078,  0.8 , 0.196078}; 
GLfloat Orange[3] = { 1, .5, 0}; 



//the length (maxx-minx), width (maxy-miny) and height (maxz-minz)  of the dataset 
double dim_x, dim_y, dim_z; 

//copy from lpoints, for faster access during rendering 
double minx, maxx, miny, maxy, minz, maxz;

//scale is used to map the points to [-1,1] x [-1, 1] x [-1, 1]. Scale
//is the same on all dimensions. It is initialized after reading
//points from file to either 1/dim_x or 1/dim_y, whichever is
//smallest.
double scale = 1; 

//the heights can be vertically exagerated. Controlled by keypress >, <
double  Z_EXAGERRATION  = 1; 




/* forward declarations of functions */
void display(void);
void keypress(unsigned char key, int x, int y);

void draw_points(); 
void draw_xy_rect(GLfloat z, GLfloat* col); 
void draw_xz_rect(GLfloat y, GLfloat* col); 
void draw_yz_rect(GLfloat x, GLfloat* col); 
void cube(GLfloat side); 
void draw_axes(); 
GLfloat xtoscreen(GLfloat x);
GLfloat ytoscreen(GLfloat y);
GLfloat ztoscreen(GLfloat z); 






/************************************************************/
int main(int argc, char** argv) {

  //read number of points from user
  if (argc!=2) {
    printf("usage: %s file.txt\n", argv[0]);
    exit(1); 
  }

  //this populates the global that holds the points
  read_lidar_from_file(argv[1], &lpoints); 
  
  //set the length, width and height of the datasetm to be used in graphics
  minx = lpoints.minx;
  maxx = lpoints.maxx;
  miny = lpoints.miny;
  maxy = lpoints.maxy;
  minz = lpoints.minz;
  maxz = lpoints.maxz;
  
  dim_x = lpoints.maxx - lpoints.minx; 
  dim_y = lpoints.maxy - lpoints.miny; 
  dim_z = lpoints.maxz - lpoints.minz; 
  scale = (dim_x > dim_y) ? 1.0/dim_x: 1.0/dim_y; 
  printf("\tdim_x = %.1f, dim_y = %.1f, dim_z=%.1f, scale=%f\n", dim_x, dim_y, dim_z, scale); 

  
 
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
  glEnable(GL_DEPTH_TEST); //enable OpenGL hidden surface removal

  /* setup the camera (i.e. the projection transformation) */ 
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60, 1 /* aspect */, 1, 100.0); 
  /* the frustrum is from z=-1 to z=-100; camera is at (0,0,0) looking
     down the negative Z-axis */
  
  //set initial view: bring all z-values (which are in [-1, 1] down by
  //3 to bring them in the view frustrum of [-1, -100].
  pos[0] = pos[1] = 0; pos[2] = -3;  
  
  //initialize rotation to see terrain tilted, rather than top-down
  theta[0] = -45; theta[1] = theta[2] = 0; 
  
  /* start the graphics event handler */
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

  /* The default GL window is x=[-1,1], y= [-1,1], z=[-1, 1] with the
     origin in the center.  The view frustrum was set up from z=-1 to
     z=-100. The camera is at (0,0,0) looking down negative z-axis.
  */ 

  /* First we translate and rotate our local reference system with the
     user transformation, which represents how the user wants to look
     at the terrain. pos[] represents the cumulative translation and
     theta[] the cumulative rotation entered by the user through
     keypresses */
  glTranslatef(pos[0], pos[1], pos[2]);  
  glRotatef(theta[0], 1,0,0); //rotate theta[0] around x-axis
  glRotatef(theta[1], 0,1,0);//rotate theta[1] around y-axis
  glRotatef(theta[2], 0,0,1);//rotate theta[2] around z-axis
  
  /* We translated the local reference system where we want it to be;
     now we draw the objects in the local reference system.  */
  draw_points();  
    
  //don't need to draw a cube but I found it nice for perspective 
  //cube(1); //draw a cube of size 1

  glFlush();
}


void print_options() {

  printf("press: \n");

  printf("\tc: to cycle through colorings\n");

  printf("\t1: draw only  points on pulses with 1 return \n");
  printf("\t2: draw only first returns i.e. points with return_number=1)\n");
  printf("\t3: draw only last returns i.e. points with return_number = number_of_returns\n");
  printf("\t4: draw only points that have >1 returns\n"); 
  printf("\t5: draw all returns\n"); 

  printf("\t+/-: zoom in/out\n");
  printf("\t>/<: increase/decrase vertical exageration of heights\n");
 
  printf("\tg: toggle showing points  with code=ground (2)\n");
  printf("\tv: toggle showing points  with code=vegetation (3,4,5)\n");
  printf("\th: toggle showing points  with code=buildings(6)\n");
  printf("\to: toggle showing points  with code=other(...)\n");


  printf("\tx/X,y/Y,z/Z: rotate\n");
  printf("\tf/b/u/d/l/r: forward/back/up/down/left/right\n");

   printf("\tq: exit\n");
  
} 

/* this function is called whenever  key is pressed */
void keypress(unsigned char key, int x, int y) {

  switch(key) {

  case 'a': 
    //3d orthogonal projection, view from straight above
    glMatrixMode(GL_PROJECTION);
    //glLoadIdentity();
    //the view frustrum is z=[0, -10]
    glOrtho(-1, 1, -1, 1, 0,-10); //left, right, top, bottom, near, far
    
    //set initial view: bring all z-values (which are in [-1, 1] down
    //by 5 to bring them in the view frustrum of [0, -10].
    pos[0] = pos[1] = 0; pos[2] = -5; 
    //use default view, look straight down the negative z-axis.
    theta[0] = theta[1] = theta[2] = 0; 
    glutPostRedisplay();
    break;
  
  case 'p': 
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, 1 /* aspect */, 1, 100.0); /* the frustrum is from z=-1 to z=-100 */
    /* camera is at (0,0,0) looking along negative z axis */
    
    //set initial view: brings all z-values (which are in [-1, 1])
    //down by 3 to bring them in the view frustrum of [-1, -100]
    pos[0]=pos[1]=0; pos[2] = -3;
    //by default we look at the terrain from (0,0,0) down the negative
    //z-axis. rotate around x to get a tilted view.
   theta[0] = -45;   theta[1] = theta[2] = 0;  
    glutPostRedisplay();
    break;

  case 'c': 
    //cycle through  the colormaps options 
    COLORMAP= (COLORMAP+1) % NB_COLORMAP_CHOICES; 

    switch (COLORMAP) {
    case ONE_COLOR: 
      printf("colormap: one color\n"); 
      break; 
    case CODE_COLOR: 
      printf("colormap: by code\n");
      printf("\t: 1 not classified: yellow\n");
      printf("\t: 2 not assigned: orange\n");
      printf("\t: 3 ground: dark brown\n");
      printf("\t: 4 low veg: lime green\n");
      printf("\t: 5 med veg: med green\n");
      printf("\t: 6 high veg : forest green\n");
      printf("\t: 7, 18 noise: magenta\n");
      printf("\t: 8, 12 reserved: white\n");
      printf("\t: 9 water: blue\n");
      printf("\t: 10,11,14,15,16,17: rail, roads, wires, towers: gray\n");
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

 
    // filter based on returns    
  case '1':
    printf("1: draw only points with nb_returns = 1\n"); 
    which_return = ONE_RETURN;
    glutPostRedisplay();
    break; 
  case '2':
    printf("2: draw only first returns i.e. points with return_number=1)\n");
    which_return = FIRST_RETURN;
    glutPostRedisplay();
    break; 
  case '3':
    printf("3: draw only last returns i.e. points with return_number = number_of_returns\n");
    which_return = LAST_RETURN;
    glutPostRedisplay();
    break;
  case '4':
    printf("4: draw only points that have >1 returns\n"); 
    which_return = MORE_THAN_ONE_RETURN; 
    glutPostRedisplay();
    break; 
  case '5':
    printf("5: draw all returns\n"); 
    which_return = ALL_RETURN;
    glutPostRedisplay();
    break; 

 
    
  case 'g': 
    //toggle off rendering ground points   (code=2)
    RENDER_GROUND = !RENDER_GROUND; 
    glutPostRedisplay();
    break;

  case 'v': 
    //toggle off rendering vegetation points  (code=3,4,5)
    RENDER_VEG = !RENDER_VEG; 
    glutPostRedisplay();
    break;

  case 'h':
    //toggle off rendering building points  (code=6)
    RENDER_BUILDING = !RENDER_BUILDING; 
    glutPostRedisplay();
    break;

  case 'o': 
    //toggle off rendering "other" ie points that are not ground, vegetation or building 
    RENDER_OTHER = !RENDER_OTHER; 
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
    //backward, move away from terrain 
  case 'b':
    pos[2] -= 0.1;
    glutPostRedisplay();
    break;
    //forward, move towards terrain 
  case 'f':
    pos[2] += 0.1; 
    glutPostRedisplay();
    break;
    //down 
  case 'd': 
     pos[1] -= 0.1; 
     glutPostRedisplay();
    break;
    //up
  case 'u': 
    pos[1] += 0.1; 
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

  case '+': //zoom in 
    scale *= 1.1; 
    glutPostRedisplay();
    break;
  case '-': //zoom in 
    scale /= 1.1; 
    glutPostRedisplay();
    break;
    
  case '>': 
    Z_EXAGERRATION *= 1.1; 
     glutPostRedisplay();
    break;

  case '<': 
    Z_EXAGERRATION /= 1.1; 
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
  print_options();
  
}//keypress



//this function is called to set the color of a point based on p.code
void setColorByCode(lidar_point p) {
  switch (p.code) {
  case 0: //never classified
    glColor3fv(yellow); 
    break; 
  case 1: //unnasigned 
    glColor3fv(Orange); 
    break; 
  case 2: //ground 
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
    glColor3fv(red); 
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
    glColor3fv(gray); 
    break;
  case 16: //wire 
  case 17: //bridge deck 
    glColor3fv(gray); 
    break;
  case 18: //high noise
    glColor3fv(magenta); 
    break;
  default: 
    printf("panic: encountered unknown code >18"); 
  }
} //setColorByCode



//this function is called to set the color of a point p based on
//p.myCode
void setColorByMycode(lidar_point p) {

  //fill in 
  
  glColor3fv(gray);
}

//draw everything with one color 
void  setColorOneColor(lidar_point p) {

   glColor3fv(yellow); //yellow should be a constant
  return; 
}



//This function is called to set the color of a point p before it is
//rendered
void setColor(lidar_point p) { 

  if (COLORMAP == ONE_COLOR) {
    //draw all points with same color 
    setColorOneColor(p); 
 
  } else if (COLORMAP == CODE_COLOR) {
    setColorByCode(p); 
  
  } else if (COLORMAP == MYCODE_COLOR) {
    setColorByMycode(p); 
  
  } else {
    printf("unkown colormap options.\n");
    exit(1); 
  }
} //setColor()






/* ****************************** */
/* Draw the points.  

   NOTE: The points are in the range x=[minx, maxx], y=[miny,
   maxy], z=[minz, maxz] and they must be mapped into
   x=[-1,1], y=[-1, 1], z=[-1,1]
  */
void draw_points(){
  
  //the actual points 
  vector<lidar_point> data = lpoints.data;
  
  lidar_point p; 
  glBegin(GL_POINTS); 
  for (int i=0; i < data.size(); i++) {

    //current point; do we want to include it in the rendering? 
    p = data[i]; 
    
    //FIRST FILTER BY RETURN
    if (which_return == FIRST_RETURN) // we only want first returns
      if (p.return_number!=1) continue;

    if (which_return == LAST_RETURN) // we only want last returns
      if (p.return_number !=p.nb_of_returns) continue;

    if (which_return == MORE_THAN_ONE_RETURN) //we only want pulses that have > 1 return 
      if (p.nb_of_returns ==1) continue;

    if (which_return == ONE_RETURN) //we only want pulses that have > 1 return 
      if (p.nb_of_returns > 1) continue;


    
    //if (which_return==ALL_RETURN)  // we want all points so keep going 


    //if point made it here, it has the return we want

    //NEXT FILTER BY CODE
    //if this point is 2 and we dont want to render ground, skip it 
    if (p.code == 2 && !RENDER_GROUND )  continue; 

    //if this point is 3,4,5 and we don't want to draw the vegetation,skip it
    if ((p.code == 3||p.code == 4||p.code == 5) && !RENDER_VEG)  continue; 

    //if this point if 6 and we don't want to draw teh buildings, skip it 
    if (p.code == 6 && !RENDER_BUILDING) continue; 

    //if this point is "other" and we don't want to draw "other" skip it 
    if ((p.code == 0 || p.code ==1 || p.code >6) && !RENDER_OTHER) continue; 
    
    //if point made it here, it needs to be rendered 
    
    //set the color of this point
    setColor(p);
    
    //tell openGL to render it
    glVertex3f(xtoscreen(data[i].x),
	       ytoscreen(data[i].y), 
	       ztoscreen(data[i].z));
    
  }
  
  glEnd(); 
}//draw_points











/* x is a value in [minx, maxx]; it is mapped to [-1,1] */
GLfloat xtoscreen(GLfloat x) {
  //map x to [-1, 1]
  //return (-1 + 2*(x-minx)/(maxx-minx)); 

  //map x keeping the aspect ratio of the dataset
  double diff = 2 * (1.0 - dim_x*scale); 
  double xnew =  -1 + diff/2 +  2*(x - minx)* scale; 
  // printf("x=%.1f\n", xnew); 
  return xnew;
}


/* y is a value in [miny, maxy]; it is mapped to [-1,1] */
GLfloat ytoscreen(GLfloat y) {
  //map y to [-1, 1]
  //return (-1 + 2*(y-miny)/(maxy-miny)); 
  
  //map y keeping the aspect ratio of the dataset
  double diff = 2 * (1.0 - dim_y*scale); 
  double ynew =  -1 + diff/2 +  2*(y - miny)* scale; 
  //printf("y=%.1f\n", ynew); 
  return ynew;
}

/* z is a value in [minz, maxz]; it is mapped so that [minz, maxz] map to [0,1] */
GLfloat ztoscreen(GLfloat z) {
  //map z to [0, 1] 
  // return ((z-minz)/(maxz-minz)); 

  //map at the same scale as on xy
  // double zscale = 1/(maxz-minz);
  //zscale = zscale/Z_EXAGERRATION;
  return (z-minz)* scale * Z_EXAGERRATION; 
}




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


