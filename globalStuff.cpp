/**
 * @file globalStuff.cpp
 */

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>
#include <string>
#include <iostream>

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>
#include "globalStuff.h"


//-- Declare global variables once
Eigen::VectorXd gStartConf;
Eigen::VectorXd gStartPos;  

Eigen::VectorXd gTargetConf;
Eigen::VectorXd gTargetPos;   
Eigen::VectorXd gTargetPose; 

std::string gTargetObjectName;

int gRobotId;
int gEEId; 
kinematics::BodyNode *gEENode;
std::string gEEName;
std::string gRobotName;

Eigen::VectorXi gLinks;

double gSizeX;
double gSizeY;
double gSizeZ;
double gResolution;
double gOriginX;
double gOriginY; 
double gOriginZ;
int gPadding;

/*
 * @function GetEE_XYZ
 */
Eigen::VectorXd GetEE_Pos( const Eigen::VectorXd &_q ) {

    mWorld->mRobots[gRobotId]->setDofs( _q, gLinks );
    mWorld->mRobots[gRobotId]->update();
    Eigen::MatrixXd pose = gEENode->getWorldTransform(); 
    Eigen::VectorXd xyz(3); xyz << pose(0,3), pose(1,3), pose(2,3);

    return xyz;
}
