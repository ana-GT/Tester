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


// ********** ROBOT HARD INFO ********

const int gNumRobotTypes = 4;
const char* gRobotNames[] = {"LWA4", "LWA3", "Barret", "Katana"};
const char* gEEId_A_Names[] = {"LJ7", "EE", "EE", "EE"};
const char* gEEId_B_Names[] = {"RJ7", NULL, NULL, NULL};
const wxString gWxRobotNames[] = { wxT( "LWA4" ), wxT("LWA3"), wxT("Barret"), wxT("Katana") };

//-- LWA3
const int sNum_LA_Links_LWA3 = 7;
const char* sLA_Ids_LWA3[sNum_LA_Links_LWA3] = {"L1", "L2", "L3", "L4", "L5", "L6", "FT"};

//-- Barret
const int sNum_LA_Links_Barret = 7;
const char* sLA_Ids_Barret[sNum_LA_Links_Barret] = {"B1", "B2", "B3", "B4", "B5", "B6", "B7"};

//-- Katana
const int sNum_LA_Links_Katana = 5;
const char* sLA_Ids_Katana[sNum_LA_Links_Katana] = {"K1", "K2", "K3", "K4", "K5"};

// ***********************************
//-- Declare global variables once

//-- ** ARM A : Start and Target **
Eigen::VectorXd gStartConf_A;
Eigen::VectorXd gStartPos_A;  
Eigen::VectorXd gTargetPos_A;   
Eigen::VectorXd gTargetPose_A;

std::string gTargetObjectName_A;

//-- ** ARM B : Start and Target **
Eigen::VectorXd gStartConf_B;
Eigen::VectorXd gStartPos_B;  
Eigen::VectorXd gTargetPos_B;   
Eigen::VectorXd gTargetPose_B;

std::string gTargetObjectName_B;

//-- ** Robot specific info **
int gRobotId;
std::string gRobotName;

//-- ** ARM A: Links **
int gNumLinks_A;
Eigen::VectorXi gLinks_A;
int gEEId_A;
kinematics::BodyNode *gEENode_A;
std::string gEEName_A;

//-- ** ARM B: Links **
int gNumLinks_B;
Eigen::VectorXi gLinks_B;
int gEEId_B;
kinematics::BodyNode *gEENode_B;
std::string gEEName_B;

double gSizeX;
double gSizeY;
double gSizeZ;
double gResolution;
double gOriginX;
double gOriginY; 
double gOriginZ;
int gPadding;

//-- ** OUTPUTS: POSE PATHS A & B **
std::vector<Eigen::VectorXd> gPosePath_A;
std::vector<Eigen::VectorXd> gPosePath_B;

/*
 * @function GetEE_XYZ
 */
Eigen::VectorXd GetEE_Pos( const Eigen::VectorXd &_q, eConfig _which ) {

  Eigen::MatrixXd pose;

  if( _which == ARM_A ) {
    mWorld->mRobots[gRobotId]->setDofs( _q, gLinks_A );
    mWorld->mRobots[gRobotId]->update();
    pose = gEENode_A->getWorldTransform(); 
  }
  else {
    mWorld->mRobots[gRobotId]->setDofs( _q, gLinks_B );
    mWorld->mRobots[gRobotId]->update();
    pose = gEENode_B->getWorldTransform(); 
  }

  Eigen::VectorXd xyz(3); xyz << pose(0,3), pose(1,3), pose(2,3);
  return xyz;
}

/**
 * @function SetTimeline
 */
void SetTimeline( std::vector<Eigen::VectorXd> _path, double _time, eConfig _which ) {

  if( mWorld == NULL || _path.size() == 0 ) {
    std::cout << "--(!) Must create a valid plan before updating its duration (!)--" << std::endl;
    return;
  }  
  
  int numsteps = _path.size(); printf("** Num steps: %d \n", numsteps);
  double increment = _time / (double)numsteps;
  
  cout << "-->(+) Updating Timeline - Increment: " << increment << " Total T: " << _time << " Steps: " << numsteps << endl;
  
  frame->InitTimer( string("Plan"),increment );
  
  if( _which == ARM_A ) {
    Eigen::VectorXd vals( gLinks_A.size() );
  
    for( size_t i = 0; i < numsteps; ++i ) {
      mWorld->mRobots[gRobotId]->setDofs( _path[i], gLinks_A );
      mWorld->mRobots[gRobotId]->update();      
      frame->AddWorld( mWorld );
    }
  }
  else {
    Eigen::VectorXd vals( gLinks_B.size() );
  
    for( size_t i = 0; i < numsteps; ++i ) {
      mWorld->mRobots[gRobotId]->setDofs( _path[i], gLinks_B );
      mWorld->mRobots[gRobotId]->update();      
      frame->AddWorld( mWorld );
    }
  }
}

/**
 * @function CheckCollisionConfig
 * @brief Check if there is a collision in the specified manipulator config
 */
bool CheckCollisionConfig( Eigen::VectorXd _q, eConfig _which ) {

  if( _which == ARM_A ) {
    mWorld->mRobots[gRobotId]->setDofs( _q, gLinks_A );
  }
  else {
    mWorld->mRobots[gRobotId]->setDofs( _q, gLinks_B );
  }
    mWorld->mRobots[gRobotId]->update();
    return mCollision->CheckCollisions();  
}
