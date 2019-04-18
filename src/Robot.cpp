#include <stdio.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

#define SHOULDER_INIT_ANGLE 0
#define ELBOW1_INIT_ANGLE 11
#define ELBOW2_INIT_ANGLE 10

static int shoulder = 0, elbow1 = 11, elbow2 = 10;
GLfloat cameraDistance = 12, cameraAngle = 0;

void init(void) {

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_FLAT);
}

void
draw_floor(void) {
	glDisable(GL_LIGHTING);
	glBegin(GL_QUADS);
		glVertex3f(-18.0, 0.0, 27.0);
		glVertex3f(27.0, 0.0, 27.0);
		glVertex3f(27.0, 0.0, -18.0);
		glVertex3f(-18.0, 0.0, -18.0);
	glEnd();
	glEnable(GL_LIGHTING);
}
// void draw_floor() {
// 	glColor4f(.0,.6,0,1); 
// 	glPushMatrix(); 
// 	glScalef(4, 0.01, 4); 
// 	glBegin(GL_QUADS); 
// 	glNormal3f(-1,0,1); 
// 	glTexCoord2f(0,0); 
// 	glVertex3f(-0.5, 0, 0.5);
// 	glNormal3f(1,0,1); 
// 	glTexCoord2f(0,1); 
// 	glVertex3f(0.5, 0, 0.5);
// 	glNormal3f(1,0,-1);
// 	glTexCoord2d(1,1); 
// 	glVertex3f(0.5, 0 , -0.5);
// 	glNormal3f(-1, 0, -1);
// 	glTexCoord2d(1,0); 
// 	glVertex3f(-0.5, 0, -0.5);
// 	glEnd(); 
// 	//glutSolidCube(1.0);
// 	glPopMatrix(); 
// }

void display(void) {
	glClear(GL_COLOR_BUFFER_BIT);

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

void reshape (int w, int h) {
	glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 1.0, 20.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef (0.0, 0.0, -5.0);

	// glViewport(0, 0, w, h);
    
	// glMatrixMode(GL_PROJECTION);
	// glLoadIdentity();

	// gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 1.0, 20.0);
    // glFrustum (-1.0, 1.0, -1.0, 1.0, 1.5, 300.0);
    
	// glMatrixMode(GL_MODELVIEW);
	// glLoadIdentity();
}

void specialKeys(int key, int x, int y) {
    GLfloat distanceDelta = 1.0, angleDelta = 5 * M_PI / 180.0;
    if(key == GLUT_KEY_UP) {
        cameraDistance -= distanceDelta;
        cameraDistance = fmax((GLfloat)2.0, cameraDistance);
    }
    if(key == GLUT_KEY_DOWN) {
        cameraDistance += distanceDelta;
    }
    if(key == GLUT_KEY_LEFT) {
        cameraAngle -= angleDelta;
    }
    if(key == GLUT_KEY_RIGHT) {
        cameraAngle += angleDelta;
    }
    glutPostRedisplay();
}

void keyboard (unsigned char key, int x, int y) {
	switch (key) {
		case 'a':   /*  a key rotates at shoulder  */
			shoulder = (shoulder + 5) % 360;
			glutPostRedisplay();
			break;
		case 'd':
			shoulder = (shoulder - 5) % 360;
			glutPostRedisplay();
			break;
		case 'w':  /*  s key rotates at elbow1  */
			elbow1 = (elbow1 + 5) % 360;
			glutPostRedisplay();
			break;
		case 's':
			elbow1 = (elbow1 - 5) % 360;
			glutPostRedisplay();
			break;
		case 'e':  /*  d key rotates at elbow1  */
			elbow2 = (elbow2 + 5) % 360;
			glutPostRedisplay();
			break;
		case 'q':
			elbow2 = (elbow2 - 5) % 360;
			glutPostRedisplay();
			break;
		case 'R':
			shoulder = SHOULDER_INIT_ANGLE;
			elbow1 = ELBOW1_INIT_ANGLE;
			elbow2 = ELBOW2_INIT_ANGLE;
			glutPostRedisplay();
			break;
		default:
			break;
   }
}

int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize (700, 700); 
	// glutInitWindowPosition (100, 100);
	glutCreateWindow (argv[0]);
	init ();
	glutDisplayFunc(display); 
	glutReshapeFunc(reshape);
	glutSpecialFunc(specialKeys);
	glutKeyboardFunc(keyboard);
	glutMainLoop();
	return 0;

	// glutInit(&argc, argv);    
	// glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	// glutInitWindowSize(700, 700);
	// glutCreateWindow("Brazo Robot");
	// glutDisplayFunc(display);
	// glutReshapeFunc(reshape);
	// glutSpecialFunc(specialKeys);
	// glutKeyboardFunc(keyboard);

	// // Inicialización
	// glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Negro

	// glutMainLoop();
}