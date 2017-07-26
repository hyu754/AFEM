
#include <GL/glew.h>
#include <GL/wglew.h>
#include <GL/freeglut.h>
#include <vector>
#include <map>
#include <iostream>
#include <algorithm>
#include <glm/glm.hpp>
#include "AFEM_visualization.hpp"
#include "AFEM_simulation.hpp"
AFEM::Simulation *global_ptr;
using namespace std;  //change
const int width = 1024, height = 1024;

#define EPSILON 0.001f
#define EPS2  EPSILON*EPSILON
size_t total_points=0;
float timeStep =  1/60.0f;
float currentTime = 0;
float accumulator = timeStep;
int selected_index = -1;
 

struct Tetrahedron {
	int indices[4];			//indices
	float volume;			//volume 
	float plastic[6];		//plasticity values
	glm::vec3 e1, e2, e3;	//edges
	glm::mat3 Re;			//Rotational warp of tetrahedron.
	glm::mat3 Ke[4][4];		//Stiffness element matrix
	glm::vec3 B[4];			//Jacobian of shapefunctions; B=SN =[d/dx  0     0 ][wn 0  0]
							//                                  [0    d/dy   0 ][0 wn  0]
							//									[0     0   d/dz][0  0 wn]
							//									[d/dy d/dx   0 ]
							//									[d/dz  0   d/dx]
							//									[0    d/dz d/dy]
};


int oldX=0, oldY=0;
float rX=15, rY=0;
int state =1 ;
float dist=-2.5f;
const int GRID_SIZE=10;
 

glm::vec3 gravity=glm::vec3(0.0f,-9.81f,0.0f);  


GLint viewport[4];
GLdouble MV[16];
GLdouble P[16];
double zoom = 60;
glm::vec3 Up=glm::vec3(0,1,0), Right, viewDir;

LARGE_INTEGER frequency;        // ticks per second
LARGE_INTEGER t1, t2;           // ticks
float frameTimeQP=0;
float frameTime =0 ;

float startTime =0, fps=0;
int totalFrames=0;


char info[MAX_PATH]={0};

int total_tetrahedra =0;

vector<glm::vec3> Xi;		//Model coordinates
vector<glm::vec3> X;		//Current coordinates
vector<glm::vec3> V;		//Velocity
vector<float> mass;			//Mass matrix
vector<glm::vec3> F;		//Force
vector<bool> IsFixed;		//Fixed point
glm::mat3 I=glm::mat3(1);	//3x3 identity matrix
  
typedef map<int, glm::mat3> matrix_map;
vector<matrix_map> K_row;
vector<matrix_map> A_row;
typedef matrix_map::iterator matrix_iterator;
vector<glm::vec3> F0;
vector<glm::vec3> b;

//For Conjugate Gradient 
vector<glm::vec3> residual;
vector<glm::vec3> prev1;
vector<glm::vec3> update;
 
float tiny      = 1e-010f;       // TODO: Should be user controllable
float tolerence = 0.001f;   
int i_max = 20;

bool bUseStiffnessWarping = true;


void OnMouseDown(int button, int s, int x, int y)
{
	if (s == GLUT_DOWN) 
	{
		
		oldX = x; 
		oldY = y; 
		int window_y = (height - y);
		float norm_y = float(window_y)/float(height/2.0);
		int window_x = x ;
		float norm_x = float(window_x)/float(width/2.0);

		float winZ=0;
		glReadPixels( x, height-y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
		double objX=0, objY=0, objZ=0;
		gluUnProject(window_x,window_y, winZ,  MV,  P, viewport, &objX, &objY, &objZ);
		glm::vec3 pt(objX,objY, objZ); 
		printf("\nObj [ %3.3f,%3.3f,%3.3f ]",objX, objY, objZ);
		size_t i=0;
		for(i=0;i<total_points;i++) {			 
			if( glm::distance(X[i],pt)<0.01) {
				selected_index = i;
				
				printf("Intersected at %d\n",i);
				printf("Pt [ %3.3f,%3.3f,%3.3f ]\n",X[i].x, X[i].y, X[i].z);
				break;
			}
		}
	}	

	if(button == GLUT_MIDDLE_BUTTON)
		state = 0;
	else
		state = 1;

	if(s==GLUT_UP) {
		selected_index= -1;
	
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}
}  

void OnMouseMove(int x, int y)
{
	if(selected_index == -1) {
		if (state == 0)
			dist *= (1 + (y - oldY)/60.0f); 
		else
		{
			rY += (x - oldX)/5.0f; 
			rX += (y - oldY)/5.0f; 
		} 
	} else {
		 
		float delta = 1000/abs(dist);
		float valX = (x - oldX)/delta; 
		float valY = (oldY - y)/delta; 
		if(abs(valX)>abs(valY))
			glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
		else 
			glutSetCursor(GLUT_CURSOR_UP_DOWN);

		V[selected_index] = glm::vec3(0); 
		X[selected_index].x += Right[0]*valX ;
		float newValue = X[selected_index].y + Up[1]*valY;
		if(newValue>0)
			X[selected_index].y = newValue;
		X[selected_index].z += Right[2]*valX + Up[2]*valY;
	}
	oldX = x; 
	oldY = y; 

	glutPostRedisplay(); 
}


void DrawGrid()
{
	glBegin(GL_LINES);
	glColor3f(0.1f, 0.5f, 0.5f);
	for(int i=-GRID_SIZE;i<=GRID_SIZE;i++)
	{
		glVertex3f((float)i,0,(float)-GRID_SIZE);
		glVertex3f((float)i,0,(float)GRID_SIZE);

		glVertex3f((float)-GRID_SIZE,0,(float)i);
		glVertex3f((float)GRID_SIZE,0,(float)i);
	}
	glEnd();
}


 

void InitGL() {  
	 
#if 0
	GenerateBlocks(10, 5, 3, 0.1f, 0.1f, 0.1f);
	total_tetrahedra = tetrahedra.size();

	total_points = X.size();
	mass.resize(total_points);

	//copy positions to buffer 
	A_row.resize(total_points);
	K_row.resize(total_points);
	b.resize(total_points);
	V.resize(total_points);
	F.resize(total_points);
	F0.resize(total_points);
	residual.resize(total_points);
	update.resize(total_points);

	prev1.resize(total_points);

	//fill in V
	memset(&(V[0].x), 0, total_points*sizeof(glm::vec3));

#endif // 0

	startTime = (float)glutGet(GLUT_ELAPSED_TIME);
	currentTime = startTime;

	// get ticks per second
	QueryPerformanceFrequency(&frequency);

// start timer
QueryPerformanceCounter(&t1);
glEnable(GL_DEPTH_TEST);
glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//glPointSize(10);
try{
//	wglSwapIntervalEXT(0);
	throw 1;
}
catch (int e){
	std::cout << "f" << std::endl;
}



}

void OnReshape(int nw, int nh) {
	glViewport(0, 0, nw, nh);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)nw / (GLfloat)nh, 0.1f, 100.0f);

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_PROJECTION_MATRIX, P);

	glMatrixMode(GL_MODELVIEW);
}

void OnRender() {
	size_t i = 0;
	float newTime = (float)glutGet(GLUT_ELAPSED_TIME);
	frameTime = newTime - currentTime;
	currentTime = newTime;
	//accumulator += frameTime;

	//Using high res. counter
	QueryPerformanceCounter(&t2);
	// compute and print the elapsed time in millisec
	frameTimeQP = (t2.QuadPart - t1.QuadPart) * 1000.0f / frequency.QuadPart;
	t1 = t2;
	accumulator += frameTimeQP;

	++totalFrames;
	if ((newTime - startTime) > 1000)
	{
		float elapsedTime = (newTime - startTime);
		fps = (totalFrames / elapsedTime) * 1000;
		startTime = newTime;
		totalFrames = 0;
	}

	sprintf_s(info, "FPS: %3.2f, Frame time (GLUT): %3.4f msecs, Frame time (QP): %3.3f, Stiffness Warp: %s", fps, frameTime, frameTimeQP, bUseStiffnessWarping ? "On" : "Off");
	glutSetWindowTitle(info);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0f, 1.0f, 1.0f, 0.0f);
	glLoadIdentity();
	glTranslatef(0, 0, dist);
	glRotatef(rX, 1, 0, 0);
	glRotatef(rY, 0, 1, 0);

	glGetDoublev(GL_MODELVIEW_MATRIX, MV);
	viewDir.x = (float)-MV[2];
	viewDir.y = (float)-MV[6];
	viewDir.z = (float)-MV[10];
	Right = glm::cross(viewDir, Up);



	global_ptr->run();

	//draw grid
	DrawGrid();

	glColor3f(0.75, 0.75, 0.75);
	glBegin(GL_LINES);

	for (int i = 0; i < global_ptr->element_vec.size(); i++) {
		/*n1 = in_pos[in_element->nodes_in_elem[0]];
		n2 = in_pos[in_element->nodes_in_elem[1]];
		n3 = in_pos[in_element->nodes_in_elem[2]];
		n4 = in_pos[in_element->nodes_in_elem[3]];*/
		AFEM::position_3D p1 = global_ptr->pos_array[global_ptr->element_array[i].nodes_in_elem[0]];
		AFEM::position_3D p2 = global_ptr->pos_array[global_ptr->element_array[i].nodes_in_elem[1]];
		AFEM::position_3D p3 = global_ptr->pos_array[global_ptr->element_array[i].nodes_in_elem[2]];
		AFEM::position_3D p4 = global_ptr->pos_array[global_ptr->element_array[i].nodes_in_elem[3]];



		glVertex3f(p4.x, p4.y, p4.z);		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p4.x, p4.y, p4.z);		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p4.x, p4.y, p4.z);		glVertex3f(p3.x, p3.y, p3.z);

		glVertex3f(p1.x, p1.y, p1.z);		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p1.x, p1.y, p1.z);		glVertex3f(p3.x, p3.y, p3.z);

		glVertex3f(p2.x, p2.y, p2.z);		glVertex3f(p3.x, p3.y, p3.z);
	}
	glEnd();


	//draw points	
	
	glEnable(GL_POINT_SMOOTH);
	glPointSize((GLfloat)5.0);
	glBegin(GL_POINTS);

	

	for (int i = 0; i < global_ptr->element_vec.size(); i++) {
	
		glColor3f((float)!0, (float)1, (float)0);



		AFEM::position_3D p1 = global_ptr->pos_array[global_ptr->element_array[i].nodes_in_elem[0]];
		AFEM::position_3D p2 = global_ptr->pos_array[global_ptr->element_array[i].nodes_in_elem[1]];
		AFEM::position_3D p3 = global_ptr->pos_array[global_ptr->element_array[i].nodes_in_elem[2]];
		AFEM::position_3D p4 = global_ptr->pos_array[global_ptr->element_array[i].nodes_in_elem[3]];


		

		glVertex3f(p4.x, p4.y, p4.z);		
		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p3.x, p3.y, p3.z);
		glVertex3f(p2.x, p2.y, p2.z);
		

	}




	glEnd();

	glPointSize((GLfloat)20.0);
	//draw stat points
	glBegin(GL_POINTS);

	

	for (int i = 0; i < global_ptr->element_vec.size(); i++) {
		glColor3f((float)0.4, (float)1.0, (float)0.5);
		for (int j = 0; j < global_ptr->stationary_vec.size(); j++){
			for (int k = 0; k < 4; k++){
				if (global_ptr->stationary_array[j].node_number == global_ptr->element_array[i].nodes_in_elem[k]){
					glVertex3f(global_ptr->element_array[i].position_info[k].x, global_ptr->element_array[i].position_info[k].y, global_ptr->element_array[i].position_info[k].z);
				}
			}

		}
		




		//AFEM::position_3D p1 = global_ptr->element_array[i].position_info[0];
		//AFEM::position_3D p2 = global_ptr->element_array[i].position_info[1];
		//AFEM::position_3D p3 = global_ptr->element_array[i].position_info[2];
		//AFEM::position_3D p4 = global_ptr->element_array[i].position_info[3];



		//glVertex3f(p4.x, p4.y, p4.z);
		//glVertex3f(p1.x, p1.y, p1.z);
		//glVertex3f(p3.x, p3.y, p3.z);
		//glVertex3f(p2.x, p2.y, p2.z);

	}




	glEnd();

	glutSwapBuffers(); 
}

void OnShutdown() {
	
}



glm::mat3 ortho_normalize(glm::mat3 A) {
	glm::vec3 row0(A[0][0],A[0][1],A[0][2]);
	glm::vec3 row1(A[1][0],A[1][1],A[1][2]);
	glm::vec3 row2(A[2][0],A[2][1],A[2][2]);

	float L0 = glm::length(row0);
	if(L0) 
		row0 /= L0;

	row1 -=  row0 * glm::dot(row0 , row1);
	float L1 = glm::length(row1);
	if(L1) 
		row1 /= L1;

	row2 = glm::cross( row0 , row1);

	return glm::mat3(row0,
					 row1,
					 row2);
}


 



void OnIdle() {	

	/*
	//Semi-fixed time stepping
	if ( frameTime > 0.0 )
	{
	const float deltaTime = min( frameTime, timeStep );
	StepPhysics(deltaTime );
	frameTime -= deltaTime;    		
	}
	*/

	//Fixed time stepping + rendering at different fps	
	if ( accumulator >= timeStep )
	{	 
		
		accumulator -= timeStep;
	}

	glutPostRedisplay();
}
void GroundCollision()  
{
	for(size_t i=0;i<total_points;i++) {	
		if(X[i].y<0) //collision with ground
			X[i].y=0;
	}
}


void OnKey(unsigned char key, int, int) {
	/*switch(key) {
		case ' ': bUseStiffnessWarping=!bUseStiffnessWarping;	break;
	}*/

	if (key == 'z'){
		zoom = zoom + 10;
	}
	else if (key == 'x'){
		zoom = zoom - 10;
	}
	//printf("Stiffness Warping %s\n",bUseStiffnessWarping?"On":"Off");
	glutPostRedisplay();
}

void AFEM::Visualization::run_visualization() {
	int argc = 1;
	char *argv[1] = { (char*)"Something" };
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("GLUT OpenCloth - Co-rotated Linear FEM with Stiffness Warping");

	glutDisplayFunc(OnRender);
	glutReshapeFunc(OnReshape);
	glutIdleFunc(OnIdle);

	glutMouseFunc(OnMouseDown);
	glutMotionFunc(OnMouseMove);	

	glutKeyboardFunc(OnKey);
	glutCloseFunc(OnShutdown);

	glewInit();
	InitGL();

	puts("Press ' ' to toggle stiffness warping on/off\n");
	global_ptr = simulation_class;
	glutMainLoop();		
}
