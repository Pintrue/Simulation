#include <GLUT/glut.h>

static int shoulder = 0, elbow1 = 11, elbow2 = 10;
float _camPos[3];
int _xRot, _yRot, _zRot, _zoom;

void initializeGL () {
    glClearColor(0.1, 0.1, 0.1, 0.0);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
}

void display(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT);

    gluLookAt(_camPos[0],_camPos[1],_camPos[2],0,0,0,0,0,1);

    // Rotate in x, y, z
    glRotated(_xRot / 16.0, 1.0, 0.0, 0.0);
    glRotated(_yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(_zRot / 16.0, 0.0, 0.0, 1.0);
    // Zoom in/out
    glScalef(_zoom, _zoom, 1.0f);

	glColor3f (1.0, 0.0, 0.0);
   	glPushMatrix();
	// glTranslatef(0.0, 0.475, 0.0);
	glRotatef((GLfloat) shoulder, 0.0, 1.0, 0.0);
	glPushMatrix();
	glScalef(0.68, 0.475, 0.80);
	glutWireCube(1.0);
	glPopMatrix();

	glColor3f (0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.475 / 2.0, 0.0); // to next pivot point
	glTranslatef(0.0, 0.49 / 2.0, -0.305);
	glPushMatrix();
	glScalef(0.637, 0.49, 0.51);
	glutWireCube(1.0);
	glPopMatrix();

	glColor3f (1.0, 0.0, 0.0);
	// glTranslatef(0.0, 0.49 / 2.0, 0.51 / 2); // to next pivot point
	glRotatef((GLfloat) elbow1, -1.0, 0.0, 0.0);
	glTranslatef(0.0, 0.38 / 2.0, 1.0 / 2.0);
	// glTranslatef(0.0, 0.11, 0.557);
	glPushMatrix();
	glScalef(0.325, 0.38, 1.0);
	glutWireCube(1.0);
	glPopMatrix();

	glColor3f (0.0, 1.0, 0.0);
	glTranslatef(0.0, 0, 0.513);
	// glTranslatef(1.0, 0.0, 0.0);
	glRotatef((GLfloat) elbow2, 1.0, 0.0, 0.0);
	glTranslatef(0.0, 0.25 / 2.0, -1.60 / 2.0);
	glPushMatrix();
	glScalef(0.22, 0.22, 1.6);
	glutWireCube(1.0);
	glPopMatrix();

	glPopMatrix();
	glutSwapBuffers();
}

int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize (700, 700); 
	// glutInitWindowPosition (100, 100);
	glutCreateWindow (argv[0]);
	initializeGL();
	glutDisplayFunc(display); 
	// glutReshapeFunc(reshape);
	// glutSpecialFunc(specialKeys);
	// glutKeyboardFunc(keyboard);
	glutMainLoop();
	return 0;
}