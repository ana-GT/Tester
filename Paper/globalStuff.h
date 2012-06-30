/**
 * @file globalVariables.h	
 */


#ifndef _GLOBAL_STUFF_H_
#define _GLOBAL_STUFF_H_

#include <string>
#include <Eigen/Core>
#include <planning/Robot.h>
#include <planning/Object.h>
#include <kinematics/BodyNode.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>

// ******** ROBOT HARD INFO ********
extern const int gNumRobotTypes;
extern const wxString gWxRobotNames[];
extern const char* gRobotNames[];

//-- LWA3
extern const int sNum_LA_Links_LWA3;
extern const char* sLA_Ids_LWA3[];

//-- Barret
extern const int sNum_LA_Links_Barret;
extern const char* sLA_Ids_Barret[];

// ***************************************

//-- ** ARM A : Start and Target **
extern Eigen::VectorXd gStartConf_A;
extern Eigen::VectorXd gStartPos_A;  
extern Eigen::VectorXd gTargetPos_A;   
extern Eigen::VectorXd gTargetPose_A;

extern  std::string gTargetObjectName_A;

//-- ** ARM B : Start and Target **
extern Eigen::VectorXd gStartConf_B;
extern Eigen::VectorXd gStartPos_B;  
extern Eigen::VectorXd gTargetPos_B;   
extern Eigen::VectorXd gTargetPose_B;

extern  std::string gTargetObjectName_B;


//-- ** Robot specific info **
extern int gRobotId;
extern std::string gRobotName;

//-- ** ARM A: Links **
extern int gNumLinks_A;
extern  Eigen::VectorXi gLinks_A;
extern int gEEId_A;
extern kinematics::BodyNode *gEENode_A;
extern std::string gEEName_A;

//-- ** ARM B: Links **
extern int gNumLinks_B;
extern  Eigen::VectorXi gLinks_B;
extern int gEEId_B;
extern kinematics::BodyNode *gEENode_B;
extern std::string gEEName_B;


extern double gSizeX;
extern double gSizeY;
extern double gSizeZ;
extern double gResolution;
extern double gOriginX;
extern double gOriginY; 
extern double gOriginZ;
extern int gPadding;

//-- ** OUTPUTS: POSE PATHS A & B **
extern std::vector<Eigen::VectorXd> gPosePath_A;
extern std::vector<Eigen::VectorXd> gPosePath_B;

//-- General functions
Eigen::VectorXd GetEE_Pos( const Eigen::VectorXd &_q );
void SetTimeline( std::vector<Eigen::VectorXd> _path, double _time = 5.0 );
bool CheckCollisionConfig( Eigen::VectorXd _q );
#endif /** _GLOBAL_STUFF_H_ */
