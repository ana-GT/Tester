/**
 * @file globalVariables.h	
 */


#ifndef _GLOBAL_STUFF_H_
#define _GLOBAL_STUFF_H_

#include <wx/wx.h>
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
extern const char* gEEId_A_Names[];
extern const char* gEEId_B_Names[];

// **** IF YOU ADD A ROBOT, BE SURE TO ADD IT ALSO IN getLinksId at ConfigTab.cpp. That is all code you need to modify (as far as I remember)**** 

//-- MITSUBISHI
extern const int sNum_LA_Links_Mitsubishi;
extern const char* sLA_Ids_Mitsubishi[];

//-- LWA3
extern const int sNum_LA_Links_LWA3;
extern const char* sLA_Ids_LWA3[];

//-- Barret
extern const int sNum_LA_Links_Barret;
extern const char* sLA_Ids_Barret[];

//-- Katana
extern const int sNum_LA_Links_Katana;
extern const char* sLA_Ids_Katana[];

//-- Snake
extern const int sNum_LA_Links_Snake;
extern const char* sLA_Ids_Snake[];

//-- LWA4
extern const int sNum_LA_Links_LWA4;
extern const char* sLA_Ids_LWA4[];

extern const int sNum_RA_Links_LWA4;
extern const char* sRA_Ids_LWA4[];


// ***************************************

enum eConfig{ ARM_A, ARM_B};

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
Eigen::VectorXd GetEE_Pos( const Eigen::VectorXd &_q, eConfig _which = ARM_A );
void SetTimeline( std::vector<Eigen::VectorXd> _path, double _time = 5.0, eConfig _which = ARM_A );
bool CheckCollisionConfig( Eigen::VectorXd _q, eConfig _which = ARM_A );
bool CheckCollisionConfig( Eigen::VectorXd _q, Eigen::VectorXi _links );
#endif /** _GLOBAL_STUFF_H_ */
