#ifndef GLUT_FUNCS_H
#define GLUT_FUNCS_H

void reshape(int w, int h);
void mousedown(int button, int state, int x, int y);
void mousemove(int x, int y);
void keyboard(unsigned char key, int /*x*/, int /*y*/);
void menu(int selection);
void display(void);

void init_glut(int argc, char** argv);

#endif