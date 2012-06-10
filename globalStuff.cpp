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

std::vector<Eigen::VectorXd> gPosePath;

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

/**
 * @function SetTimeline
 */
void SetTimeline( std::vector<Eigen::VectorXd> _path, double _time ) {

  if( mWorld == NULL || _path.size() == 0 ) {
    std::cout << "--(!) Must create a valid plan before updating its duration (!)--" << std::endl;
    return;
  }  
  
  int numsteps = _path.size(); printf("NUm steps: %d \n", numsteps);
  double increment = _time / (double)numsteps;
  
  cout << "-->(+) Updating Timeline - Increment: " << increment << " Total T: " << _time << " Steps: " << numsteps << endl;
  
  frame->InitTimer( string("Plan"),increment );
  
  
  Eigen::VectorXd vals( gLinks.size() );
  
  for( size_t i = 0; i < numsteps; ++i ) {
    mWorld->mRobots[gRobotId]->setDofs( _path[i], gLinks );
    mWorld->mRobots[gRobotId]->update();
    
    frame->AddWorld( mWorld );
  }
}

/**
 * @function CheckCollisionConfig
 * @brief Check if there is a collision in the specified manipulator config
 */
bool CheckCollisionConfig( Eigen::VectorXd _q ) {
  mWorld->mRobots[gRobotId]->setDofs( _q, gLinks );
  mWorld->mRobots[gRobotId]->update();
  return mCollision->CheckCollisions();  
}
