#include <iostream>

#include <GLUT/glut.h>
#include <osg/Timer>
#include <osgUtil/SceneView>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Quat>
#include <osg/Light>
#include <osg/LightSource>

#include "glutFuncs.h"

int main(int argc, char** argv) {
    std::cout << "Hellow, world" << std::endl;

    init_glut(argc, argv);
}