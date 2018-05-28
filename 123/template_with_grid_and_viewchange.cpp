//
// Created by Zhao Xiaodong on 2018/5/24.
//

#include <GLUT/glut.h>
#include <glm/glm.hpp>
#include <vector>
#include <iostream>

using namespace std;

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

const float POINT_SIZE = 5;
const float MOTION_SPEED = 5.0f;

const float CAMERA_DISTANCE = -20.0f;
int oldX = 0, oldY = 0;
float rotateX = 30;
float rotateY = 0;
GLint viewport[4];
GLdouble MV[16];
GLdouble P[16];
glm::vec3 upDirection = glm::vec3(0, 1, 0);
glm::vec3 rightDirection, viewDirection;


const int NUM_X = 20, NUM_Y = 20;
const int POINTS_NUM = (NUM_X + 1) * (NUM_Y + 1);

vector<GLuint> indices;
vector<glm::vec3> vertices;

void fillIndices(vector<GLuint> &indices, int numX, int numY) {
    GLuint *curr = &indices[0];
    for (int i = 0; i < numY; i++) {
        for (int j = 0; j < numX; j++) {
            int i0 = i * (numX + 1) + j;
            int i1 = i0 + 1;
            int i2 = i0 + (numX + 1);
            int i3 = i2 + 1;
            if ((j + i) % 2) {
                *curr++ = i0;
                *curr++ = i2;
                *curr++ = i1;
                *curr++ = i1;
                *curr++ = i2;
                *curr++ = i3;
            } else {
                *curr++ = i0;
                *curr++ = i2;
                *curr++ = i3;
                *curr++ = i0;
                *curr++ = i3;
                *curr++ = i1;
            }
        }
    }
}

void initClothVerticesPos(vector<glm::vec3> &vertices, int numX, int numY) {
    const float height = 3.0;
    const float lenU = 5.0;
    const float lenV = 5.0;
    const float offsetU = lenU / 2;
    const float offsetV = lenV / 2;
    int curr = 0;
    for (int j = 0; j <= numY; j++) {
        for (int i = 0; i <= numX; i++) {
            vertices[curr++] = glm::vec3(
                    float(i) / numX * lenU - offsetU,
                    height,
                    float(j) / numY * lenV - offsetV
            );
        }
    }
}

void init() {
    glEnable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPointSize(POINT_SIZE);

    vertices.resize(POINTS_NUM);
    initClothVerticesPos(vertices, NUM_X, NUM_Y);
    indices.resize(NUM_X * NUM_Y * 2 * 3); // 三角形数量 * 3
    fillIndices(indices, NUM_X, NUM_Y);
}

void drawGrid() {
    const int GRID_SIZE = 10;
    glBegin(GL_LINES);
    glColor3f(0.5f, 0.5f, 0.5f);
    for (int i = -GRID_SIZE; i <= GRID_SIZE; i++) {
        glVertex3f((float) i, 0, (float) -GRID_SIZE);
        glVertex3f((float) i, 0, (float) GRID_SIZE);

        glVertex3f((float) -GRID_SIZE, 0, (float) i);
        glVertex3f((float) GRID_SIZE, 0, (float) i);
    }
    glEnd();
}

void drawCloth() {
    glColor3f(1, 0, 0);
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < indices.size(); i += 3) {
        glm::vec3 p1 = vertices[indices[i]];
        glm::vec3 p2 = vertices[indices[i + 1]];
        glm::vec3 p3 = vertices[indices[i + 2]];
        glVertex3f(p1.x, p1.y, p1.z);
        glVertex3f(p2.x, p2.y, p2.z);
        glVertex3f(p3.x, p3.y, p3.z);
    }
    glEnd();
}

void drawSphere() {
    glColor3f(0, 0, 1);
    glPushMatrix();
    glTranslatef(0, 1.0, 0);
    glRotatef(90, 1, 0, 0);
    glutWireSphere(1, 24, 24);
    glPopMatrix();
}

void draw() {
    drawGrid();
    drawCloth();
    drawSphere();
}

void adjustCamera() {
    glTranslatef(0, 0, CAMERA_DISTANCE);
    glRotatef(rotateX, 1, 0, 0);
    glRotatef(rotateY, 0, 1, 0);

    glGetDoublev(GL_MODELVIEW_MATRIX, MV);
    viewDirection.x = (float) -MV[2];
    viewDirection.y = (float) -MV[6];
    viewDirection.z = (float) -MV[10];
    rightDirection = glm::cross(viewDirection, upDirection);


}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0, 1.0, 1.0, 1);
    glLoadIdentity();

    adjustCamera();

    draw();

    glutSwapBuffers();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (GLfloat) width / (GLfloat) height, 1.f, 100.0f);

    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_PROJECTION_MATRIX, P);

    glMatrixMode(GL_MODELVIEW);
}

void idle() {
    glutPostRedisplay();
}

void mouse(int button, int key, int x, int y) {
    if (key == GLUT_DOWN) {
        oldX = x;
        oldY = y;
    }
}

void motion(int x, int y) {
    rotateY += (x - oldX) / MOTION_SPEED;
    rotateX += (y - oldY) / MOTION_SPEED;
    oldX = x;
    oldY = y;

    glutPostRedisplay();
}

void close() {
    indices.clear();
    vertices.clear();
}

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutCreateWindow("Cloth Collision");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutWMCloseFunc(close);

    init();

    glutMainLoop();
    return 0;
}
//
// Created by Zhao Xiaodong on 2018/5/24.
//

