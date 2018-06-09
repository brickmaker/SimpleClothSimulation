//
// Created by suuny on 2018/6/3.
//

#ifndef OPENGL123_BVH_H
#define OPENGL123_BVH_H

#include "BVH_Node.h"

class CCD_Object;

class BVH {

    friend class BVH_Node;

private:
    BVH_Node _rootNode1;
    BVH_Node _rootNode2;

public:
    //
    BVH(int startX, int startY, int endX, int endY,int length) : _rootNode1(startX, startY, endX, endY) ,
                                                      _rootNode2(startX + length, startY, endX + length, endY)
    {
        // 抱歉这里在合代码的时候可能xy的定义和前面不一样
        _rootNode1.refit();
        _rootNode2.refit();
    }

    ~BVH(void) {};

    void refit(void) {
        _rootNode1.refit();
        _rootNode2.refit();
    }

    void collide(void) {
        _rootNode1.refit();
        _rootNode2.refit();
//        _rootNode1.selfCollide();
//        _rootNode2.selfCollide();
        _rootNode1.collide(&_rootNode2);
        _rootNode1.collide(&_rootNode2);
        _rootNode1.collide(&_rootNode2);


    }

};


#endif //OPENGL123_BVH_H
