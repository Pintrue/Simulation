#include <stdio.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>

static int shoulder = 0, elbow = 0;

void init(void) {
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_FLAT);
}

void display(void) {
	glClear(GL_COLOR_BUFFER_BIT);
   	glPushMatrix();
	// glTranslatef(0.0, 0.475, 0.0);
	glScalef(0.68, 0.475, 0.80);
	glRotatef((GLfloat) shoulder, 0.0, 1.0, 0.0);
	glPushMatrix();
	glutWireCube(1.0);
	glPopMatrix();

	glTranslatef(0.0, 0.475, 0.0);
	glScalef(0.64, 0.49, 0.51);
	glTranslatef(0.0, 0.447, 0.305);
	// glRotatef(0.0, 0.0, 0.0, 0.0);
	glPushMatrix();
	glutWireCube(1.0);
	glPopMatrix();

	glTranslatef(0.0, 0.447, 0.305);
	glTranslatef(0.0, 0.0, 0.25);
	// glRotatef(90, 0, 1.0, 0.0);
	glRotatef((GLfloat) elbow, -1.0, 0.0, 0.0);
	// glTranslatef(0.0, 0.0, 10.0);
	glPushMatrix();
	glScalef(0.325, 0.38, 1.0);
	glutWireCube(1.0);
	glPopMatrix();

	// glTranslatef(1.0, 0.0, 0.0);
	// glRotatef((GLfloat) elbow, 0.0, 0.0, 1.0);
	// glTranslatef(1.0, 0.0, 0.0);
	// glPushMatrix();
	// glScalef(2.0, 0.4, 1.0);
	// glutWireCube(1.0);
	// glPopMatrix();

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
	// int offset = 50;
    // int dis = (w > h ? h : w) - offset * 2;
      
    // //配置显示物体屏幕的大小
    // glViewport(offset, offset, (GLsizei)dis, (GLsizei)dis);
    // printf("reshape: w=%d, h=%d, dis=%d\n", w, h, dis);
  
    // glMatrixMode(GL_PROJECTION);
    // glLoadIdentity();
  
    // glOrtho(-1.5, 1.5, -1.5, 1.5, 0, 10);
    // //gluOrtho2D(-1.5, 1.5, -1.5, 1.5);
  
    // glMatrixMode(GL_MODELVIEW);
	// glLoadIdentity();
}

void keyboard (unsigned char key, int x, int y)
{
	switch (key) {
		case 's':   /*  s key rotates at shoulder  */
			shoulder = (shoulder + 5) % 360;
			glutPostRedisplay();
			break;
		case 'S':
			shoulder = (shoulder - 5) % 360;
			glutPostRedisplay();
			break;
		case 'e':  /*  e key rotates at elbow  */
			elbow = (elbow + 5) % 360;
			glutPostRedisplay();
			break;
		case 'E':
			elbow = (elbow - 5) % 360;
			glutPostRedisplay();
			break;
		default:
			break;
   }
}

int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
   glutInitWindowSize (500, 500); 
   glutInitWindowPosition (100, 100);
   glutCreateWindow (argv[0]);
   init ();
   glutDisplayFunc(display); 
   glutReshapeFunc(reshape);
   glutKeyboardFunc(keyboard);
   glutMainLoop();
   return 0;
}