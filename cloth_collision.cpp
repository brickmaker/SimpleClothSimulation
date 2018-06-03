//
// Created by Zhao Xiaodong on 2018/5/24.
//

#include <GLUT/glut.h>
#include <glm/glm.hpp>
#include <vector>
#include <unordered_map>
#include <map>
#include <cmath>
#include <iostream>
#include "BVH.h"

using namespace std;

const float EPSILON = 0.02f;

// OpenGL Settings
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

const float POINT_SIZE = 5;
const float MOTION_SPEED = 5.0f;

// control
bool pause = true;

// Light
const GLfloat LIGHT_COLOR[] = {0.4, 0.4, 0.4, 1.0};
const GLfloat LIGHT_POS[] = {10, 10, 20, 1};

// Camera and View
const float CAMERA_DISTANCE = -10.0f;
int oldX = 0, oldY = 0;
float rotateX = 30;
float rotateY = 0;
GLint viewport[4];
GLdouble MV[16];
GLdouble P[16];
glm::vec3 upDirection = glm::vec3(0, 1, 0);
glm::vec3 rightDirection, viewDirection;
BVH *BVHTree;

// Cloth
const int NUM_X = 20, NUM_Y = 20;
const int POINTS_NUM = (NUM_X + 1) * (NUM_Y + 1);

// BALL
const glm::vec3 BALL_POS = glm::vec3(0.0f, 1.5f, 0.0f);
const float BALL_RADIUS = 1.0f;
const glm::vec3 BALL_ROTATE_SPEED = glm::vec3(0.0f, 10.0f, 0.0f);

// Springs
const float K_S_STRUCT = 0.5f;
const float K_D_STRUCT = -0.25f;
const float K_S_SHEAR = 0.5f;
const float K_D_SHEAR = -0.25f;
const float K_S_BEND = 0.85f;
const float K_D_BEND = -0.25f;

// Physics
const glm::vec3 GRAVITY = glm::vec3(0, -0.00981f, 0);
const float MASS = 0.5f;
const float DAMPING = -0.0125f;
const float FRACTION = 1.5f;
const float BOUNCH_FACTOR = -0.001f;
const float DELTA_TIME = 1.0f / 10.0f; // todo: fixed timestep

enum SpringType {
    SPRING_STRUCTURAL,
    SPRING_SHEAR,
    SPRING_BEND
};

vector<GLuint> indices;
vector<glm::vec3> vertices;
vector<glm::vec3> forces;
vector<glm::vec3> velocities;
vector<vector<pair<GLuint, GLuint>>> adjacentVertexPair;
vector<glm::vec3> normals;

struct Spring {
    int p1, p2;
    float restLenth;
    float kS, kD; // todo: ks kd?
    SpringType type; // todo: why type??
    Spring(int p1, int p2, float kS, float kD, SpringType type)
            : p1(p1), p2(p2), kS(kS), kD(kD), type(type) {
        glm::vec3 deltaPos = vertices[p1] - vertices[p2];
        restLenth = glm::length(deltaPos);
    }
};

vector<Spring> springs;

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

                adjacentVertexPair[i0].push_back(make_pair(i2, i1));
                adjacentVertexPair[i2].push_back(make_pair(i1, i0));
                adjacentVertexPair[i1].push_back(make_pair(i0, i2));
                adjacentVertexPair[i1].push_back(make_pair(i2, i3));
                adjacentVertexPair[i2].push_back(make_pair(i3, i1));
                adjacentVertexPair[i3].push_back(make_pair(i1, i2));
            } else {
                *curr++ = i0;
                *curr++ = i2;
                *curr++ = i3;
                *curr++ = i0;
                *curr++ = i3;
                *curr++ = i1;

                adjacentVertexPair[i0].push_back(make_pair(i2, i3));
                adjacentVertexPair[i2].push_back(make_pair(i3, i0));
                adjacentVertexPair[i3].push_back(make_pair(i0, i2));
                adjacentVertexPair[i0].push_back(make_pair(i3, i1));
                adjacentVertexPair[i3].push_back(make_pair(i1, i0));
                adjacentVertexPair[i1].push_back(make_pair(i0, i3));
            }
        }
    }
}

glm::vec3 getVertexNormal(GLuint index) {
    vector<pair<GLuint, GLuint >> &adjacents = adjacentVertexPair[index];
    glm::vec3 normalSum = glm::vec3(0);
    glm::vec3 p1 = vertices[index];
    for (pair<GLuint, GLuint> &adj : adjacents) {
        glm::vec3 p2 = vertices[adj.first];
        glm::vec3 p3 = vertices[adj.second];
        normalSum += glm::cross(p2 - p1, p3 - p1);
    }
    return glm::normalize(normalSum);
}

void calculateNormals() {
    for (int i = 0; i < POINTS_NUM; i++) {
        normals[i] = getVertexNormal(i);
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

void initSprings() {
    // horizontal
    for (int j = 0; j <= NUM_Y; j++) {
        for (int i = 0; i < NUM_X; i++) {
            springs.emplace_back(
                    j * (NUM_X + 1) + i,
                    j * (NUM_X + 1) + i + 1,
                    K_S_STRUCT,
                    K_D_STRUCT,
                    SPRING_STRUCTURAL
            );
        }
    }

    // vertical
    for (int i = 0; i <= NUM_X; i++) {
        for (int j = 0; j < NUM_Y; j++) {
            springs.emplace_back(
                    j * (NUM_X + 1) + i,
                    (j + 1) * (NUM_Y + 1) + i,
                    K_S_STRUCT,
                    K_D_STRUCT,
                    SPRING_STRUCTURAL
            );
        }
    }


    // shear 对角线
    for (int j = 0; j < NUM_Y; j++) {
        for (int i = 0; i < NUM_X; i++) {
            springs.emplace_back(
                    j * (NUM_X + 1) + i,
                    (j + 1) * (NUM_X + 1) + i + 1,
                    K_S_SHEAR,
                    K_D_SHEAR,
                    SPRING_SHEAR
            );
            springs.emplace_back(
                    (j + 1) * (NUM_X + 1) + i,
                    j * (NUM_X + 1) + i + 1,
                    K_S_SHEAR,
                    K_D_SHEAR,
                    SPRING_SHEAR
            );
        }
    }


    // bend
    for (int j = 0; j <= NUM_Y; j++) {
        for (int i = 0; i <= NUM_X - 2; i++) {
            springs.emplace_back(
                    j * (NUM_X + 1) + i,
                    j * (NUM_X + 1) + i + 2,
                    K_S_BEND,
                    K_D_BEND,
                    SPRING_BEND
            );
        }
    }
    for (int i = 0; i <= NUM_X; i++) {
        for (int j = 0; j <= NUM_Y - 2; j++) {
            springs.emplace_back(
                    j * (NUM_X + 1) + i,
                    (j + 2) * (NUM_X + 1) + i,
                    K_S_BEND,
                    K_D_BEND,
                    SPRING_BEND
            );
        }
    }
}

void init() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
//    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPointSize(POINT_SIZE);

    forces.resize(POINTS_NUM);
    velocities.resize(POINTS_NUM);
    normals.resize(POINTS_NUM);
    adjacentVertexPair.resize(POINTS_NUM);
    std::fill(velocities.begin(), velocities.end(), glm::vec3(0));
    std::fill(adjacentVertexPair.begin(), adjacentVertexPair.end(), vector<pair<GLuint, GLuint>>());

    vertices.resize(POINTS_NUM);
    initClothVerticesPos(vertices, NUM_X, NUM_Y);
    indices.resize(NUM_X * NUM_Y * 2 * 3); // 三角形数量 * 3
    fillIndices(indices, NUM_X, NUM_Y);

    initSprings();
    BVHTree = new BVH(0,0,NUM_X,NUM_Y);



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
    calculateNormals();
    glColor3f(1, 0, 0);
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < indices.size(); i += 3) {
        glm::vec3 p1 = vertices[indices[i]];
        glm::vec3 p2 = vertices[indices[i + 1]];
        glm::vec3 p3 = vertices[indices[i + 2]];
        glm::vec3 normal1 = normals[indices[i]];
        glm::vec3 normal2 = normals[indices[i + 1]];
        glm::vec3 normal3 = normals[indices[i + 2]];
        glNormal3f(normal1.x, normal1.y, normal1.z);
        glVertex3f(p1.x, p1.y, p1.z);
        glNormal3f(normal2.x, normal2.y, normal2.z);
        glVertex3f(p2.x, p2.y, p2.z);
        glNormal3f(normal3.x, normal3.y, normal3.z);
        glVertex3f(p3.x, p3.y, p3.z);
    }
    glEnd();
}

void drawSphere() {
    glColor3f(0, 0, 1);
    glPushMatrix();
    glTranslatef(BALL_POS.x, BALL_POS.y, BALL_POS.z);
    glRotatef(90, 1, 0, 0);
    glutSolidSphere(BALL_RADIUS, 24, 24);
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

    glEnable(GL_LIGHTING);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, LIGHT_COLOR);
    glLightfv(GL_LIGHT0, GL_POSITION, LIGHT_POS);
    glLightfv(GL_LIGHT0, GL_AMBIENT, LIGHT_COLOR);
    glEnable(GL_LIGHT0);
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

// dynamics

void computeForces() {
    // gravity and damping
    for (int i = 0; i < POINTS_NUM; i++) {
        forces[i] = glm::vec3(0);
        forces[i] += GRAVITY;
        forces[i] += DAMPING * velocities[i];
    }
    // spring
    for (int i = 0; i < springs.size(); i++) {
        glm::vec3 dX = vertices[springs[i].p1] - vertices[springs[i].p2];
        glm::vec3 dV = velocities[springs[i].p1] - velocities[springs[i].p2];
        float len = glm::length(dX);

        float fS = -springs[i].kS * (len - springs[i].restLenth);
        float fD = springs[i].kD * (glm::dot(dV, dX) / len);
        glm::vec3 f = (fD + fD) * glm::normalize(dX);

        forces[springs[i].p1] += f;
        forces[springs[i].p2] -= f;
    }
    // friction
    // todo: only for ball friction
    for (int i = 0; i < POINTS_NUM; i++) {
        if (glm::length(vertices[i] - BALL_POS) < BALL_RADIUS + 2 * EPSILON) {
            glm::vec3 normal = glm::normalize(BALL_POS - vertices[i]);
            glm::vec3 r = glm::vec3(vertices[i].x, 0, vertices[i].z);
            glm::vec3 ballTangentSpeed = glm::cross(BALL_ROTATE_SPEED, r);
            glm::vec3 pressure = forces[i] - glm::dot(forces[i], normal);
            glm::vec3 friction = glm::length(pressure) * FRACTION * glm::normalize(ballTangentSpeed - velocities[i]);
            forces[i] += friction;
        }
    }
}

void integrateExplicitEuler(float dT) {
    for (int i = 0; i < POINTS_NUM; i++) {
        vertices[i] += dT * velocities[i];
        velocities[i] += forces[i] / MASS * dT;

        if (vertices[i].y < 0)
            vertices[i].y = 0;
    }
}

void integrate(float dT) {
    integrateExplicitEuler(dT);
}

void ballCollision() {
    for (int i = 0; i < POINTS_NUM; i++) {
        glm::vec3 deltaPos = vertices[i] - BALL_POS;
        float distance = glm::length(deltaPos);
        if (distance < BALL_RADIUS + EPSILON) {
            // move it to surface
            glm::vec3 move = glm::normalize(deltaPos) * (BALL_RADIUS + EPSILON - distance);
            vertices[i] += move;
//            velocities[i] = glm::vec3(0);
            velocities[i] += glm::normalize(deltaPos) * glm::dot(velocities[i], -glm::normalize(deltaPos));
        }
    }
}

void collision() {
    ballCollision();
    BVHTree->collide();
}

void dynamicInverse() {
    for (int i = 0; i < springs.size(); i++) {
        glm::vec3 deltaPos = vertices[springs[i].p1] - vertices[springs[i].p2];
        float distance = glm::length(deltaPos);
        if (distance > springs[i].restLenth) {
            glm::vec3 move = (distance - springs[i].restLenth) / 2 * glm::normalize(deltaPos);
            // todo: 速度的变化，与长度的关系？？？
            velocities[springs[i].p1] -= move;
            velocities[springs[i].p2] += move;
        }
    }
}

void simulate(float dT) {
    computeForces();
    integrate(dT);
    collision();
    dynamicInverse();
}

void idle() {
    if (!pause)
        simulate(DELTA_TIME);
    glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case ' ':
            pause = !pause;
            break;
        default:
            break;
    }
}

void mouse(int button, int key, int x, int y) {
    if (key == GLUT_DOWN) {
        oldX = x;
        oldY = y;
    }

    if (key == GLUT_UP) {
        glutSetCursor(GLUT_CURSOR_INHERIT);
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
    delete BVHTree;
}

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutCreateWindow("Cloth Collision");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutWMCloseFunc(close);

    init();

    glutMainLoop();
    return 0;
}
