//
// Created by suuny on 2018/6/3.
//

#ifndef OPENGL123_BVH_NODE_H
#define OPENGL123_BVH_NODE_H

#include <glm/glm.hpp>
#include "collision_detect/vec.h"


class BVH_Node {
public:
    bool isleaf;
    Vec3d minBox, maxBox;
    int node[3];
    BVH_Node *_leftChild;
    BVH_Node *_rightChild;

public:
    BVH_Node(int startX, int startY, int endX, int endY);

    BVH_Node(int index1, int index2, int index3);

    ~BVH_Node();

    void refit();

    /* CCD     */
    void collide(BVH_Node *target);

    bool leafCollide(BVH_Node *target);

    void selfCollide();

    bool isLeaf() { return isleaf; };

    bool overlaps(BVH_Node *box);
};


#endif //OPENGL123_BVH_NODE_H
