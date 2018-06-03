//
// Created by suuny on 2018/6/3.
//
#include <iostream>
#include <vector>
#include "BVH_Node.h"
#include "collision_detect/rootparitycollisiontest.h"
using namespace std;

extern const int NUM_X = 20, NUM_Y = 20;
extern const float DELTA_TIME = 1.0f / 10.0f;
//extern const float DELTA_TIME;
extern vector<glm::vec3> vertices;
extern vector<glm::vec3> forces;
extern vector<glm::vec3> velocities;
const double ERROR_BOUND = 0;

int XYtoIndex(int X ,int Y)
{
    return X * (NUM_X+1) + Y;
}

Vec3d glmToVec3d(glm::vec3 t)
{
    return Vec3d(t.x, t.y, t.z);
}

Vec3d glmToVec3d(int index)
{
    return Vec3d(vertices[index].x, vertices[index].y, vertices[index].z);
}

Vec3d newVec3d(int index)
{
    return glmToVec3d(vertices[index]+DELTA_TIME*velocities[index]);

}

void stopMove(BVH_Node *temp)
{
    velocities[temp->node[0]]=glm::vec3(0);
    velocities[temp->node[1]]=glm::vec3(0);
    velocities[temp->node[2]]=glm::vec3(0);
}

bool Intersect_VF(int x0,int x1,int x2,int x3)
{
    // 0 point 123 tri
    if(x0==x1||x0==x2||x0==x3) //check if is the same point
        return false;
    //test vertex-triangle
    Vec3d verts_old[4] = {glmToVec3d(x0),glmToVec3d(x1),glmToVec3d(x2),glmToVec3d(x3)};
    Vec3d verts_new[4] = {newVec3d(x0),newVec3d(x1),newVec3d(x2),newVec3d(x3)};

    bool is_edge_edge = false;

    rootparity::RootParityCollisionTest test(
            verts_old[0], verts_old[1], verts_old[2], verts_old[3],
            verts_new[0], verts_new[1], verts_new[2], verts_new[3], is_edge_edge);
    bool ret = test.run_test();
    if(ret)
    {
        //cout<<"VF: "<<x0<<" "<<x1<<" "<<x2<<" "<<x3<<endl;
        test.run_test();

    }
    return ret;

}

bool Intersect_EE(int x0,int x1,int x2,int x3)
{
    if(x0==x2||x0==x3||x1==x2||x1==x3)
        return false;

    Vec3d verts_old[4] = {glmToVec3d(x0),glmToVec3d(x1),glmToVec3d(x2),glmToVec3d(x3)};
    Vec3d verts_new[4] = {newVec3d(x0),newVec3d(x1),newVec3d(x2),newVec3d(x3)};

    bool is_edge_edge = true;

    rootparity::RootParityCollisionTest test(
            verts_old[0], verts_old[1], verts_old[2], verts_old[3],
            verts_new[0], verts_new[1], verts_new[2], verts_new[3], is_edge_edge);
    return test.run_test();
}


BVH_Node::BVH_Node(int startX,int startY,int endX, int endY):isleaf(false)
{
    node[0] = -1;
    node[1] = -1;
    node[2] = -1;

    if(endX-startX==1 && endY-startY==1)
    {
        // 这里没有用到那个诡异的三角形分割
        _leftChild = new BVH_Node(XYtoIndex(startX, startY), XYtoIndex(startX + 1, startY), XYtoIndex(startX, startY + 1));
        _rightChild = new BVH_Node(XYtoIndex(startX + 1, startY), XYtoIndex(startX, startY + 1),
                          XYtoIndex(startX + 1, startY + 1));
        return;
    }

    if(endX - startX > endY-startY)
    {
        int middleX = (startX + endX)/2;
        _leftChild = new BVH_Node(startX,startY,middleX,endY);
        _rightChild = new BVH_Node(middleX,startY,endX,endY);
        return;
    }
    else
    {
        int middleY = (startY + endY)/2;
        _leftChild = new BVH_Node(startX,startY,endX, middleY);
        _rightChild = new BVH_Node(startX,middleY,endX, endY);
        return;
    }

    assert(!"should not get there");
    return;

}


BVH_Node::BVH_Node(int index1,int index2,int index3):isleaf(true)
{
    _leftChild = NULL;
    _rightChild = NULL;
    node[0] = index1;
    node[1] = index2;
    node[2] = index3;
}


BVH_Node::~BVH_Node()
{
    if(!isleaf) {
        delete _leftChild;
        delete _rightChild;
    }
}


void BVH_Node::refit()
{
    if ( isLeaf() ) {
        minmax(glmToVec3d(vertices[node[0]]),glmToVec3d(vertices[node[1]]),glmToVec3d(vertices[node[2]]),minBox, maxBox);
        return ;
    }

    _leftChild->refit() ;
    _rightChild->refit() ;
    minmax(_leftChild->minBox,_leftChild->maxBox,_rightChild->minBox,_rightChild->maxBox,minBox, maxBox);

};

/* CCD     */
void BVH_Node::collide(BVH_Node *target)
{

    if ( overlaps(target) ) {
        if ( isLeaf() ){
            leafCollide(target) ;
            return ;
        } else {
            _leftChild->collide( target )	;
            _rightChild->collide( target )	;
        }
    }
};


bool BVH_Node::leafCollide(BVH_Node *target)
{
    bool ret = false;
    if ( target->isLeaf() ) {
        //这里要把相邻点顶点去掉
        for ( int i = 0 ; i < 3 ; i++ ) {

            // vertex A - face B
            ret |= Intersect_VF(target->node[i],node[0],node[1],node[2]);
        }

        for ( int i = 0 ; i < 3 ; i++ ) {

            // vertex B - face A
            ret |= Intersect_VF(node[i],target->node[0],target->node[1],target->node[2]);
        }


        // EE Test
        for ( int i = 0 ; i < 3 ; i++ ) {
            for ( int j = 0 ; j < 3 ; j++ ) {
                ret |= Intersect_EE(node[i],node[(i+1)%3],target->node[i],target->node[(i+1)%3]);
            } // end for j
        } // end for i

        if(ret == true)
        {
            stopMove(this);
            stopMove(target);
        }

    } else {
        if ( overlaps(target->_leftChild))
            ret |= leafCollide( target->_leftChild) ;
        if ( overlaps( target->_rightChild))
            ret |= leafCollide( target->_rightChild);
    }


    return ret;

};

void BVH_Node::selfCollide()
{
    if ( isLeaf())
        return ;

    _leftChild->collide( _rightChild ) ;
    _leftChild->selfCollide() ;
    _rightChild->selfCollide() ;

};


bool BVH_Node::overlaps(BVH_Node *box)
{
    if (minBox[0] - box->maxBox[0] >= ERROR_BOUND) return false;	// x
    if (minBox[1] - box->maxBox[1] >= ERROR_BOUND) return false;	// y
    if (minBox[2] - box->maxBox[2] >= ERROR_BOUND) return false;	// z

    if (ERROR_BOUND <= box->minBox[0] - maxBox[0]) return false;
    if (ERROR_BOUND <= box->minBox[1] - maxBox[1]) return false;
    if (ERROR_BOUND <= box->minBox[2] - maxBox[2]) return false;

    return true;

}


