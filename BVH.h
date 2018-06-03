//
// Created by suuny on 2018/6/3.
//

#ifndef OPENGL123_BVH_H
#define OPENGL123_BVH_H

#include "BVH_Node.h"

class CCD_Object ;

class BVH {

    friend class BVH_Node ;

private:
    BVH_Node _rootNode	;

public:
    //
    BVH(int startX,int startY,int endX, int endY):_rootNode(startX,startY,endX,endY)
    {
        _rootNode.refit();
    }
    ~BVH(void){};

    void refit( void )
    {
        _rootNode.refit() ;
    }

    void collide(void)
    {
        _rootNode.refit();
        _rootNode.selfCollide();
    }

};


#endif //OPENGL123_BVH_H
