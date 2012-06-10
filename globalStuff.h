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

extern Eigen::VectorXd gStartConf;
extern Eigen::VectorXd gStartPos;  /**< (x,y,z) */

extern Eigen::VectorXd gTargetConf;
extern Eigen::VectorXd gTargetPos;  /**< (x,y,z) */ 
extern Eigen::VectorXd gTargetPose; /**< (x,y,z,roll,pitch,yaw) */

extern  std::string gTargetObjectName;

  //-- Robot specific info
extern int gRobotId;
extern int gEEId; /**< End Effector ID */
extern kinematics::BodyNode *gEENode;
extern std::string gEEName;
extern std::string gRobotName;

extern  Eigen::VectorXi gLinks;

extern double gSizeX;
extern double gSizeY;
extern double gSizeZ;
extern double gResolution;
extern double gOriginX;
extern double gOriginY; 
extern double gOriginZ;
extern int gPadding;

extern std::vector<Eigen::VectorXd> gPosePath;

//-- General functions
Eigen::VectorXd GetEE_Pos( const Eigen::VectorXd &_q );
void SetTimeline( std::vector<Eigen::VectorXd> _path, double _time = 5.0 );
bool CheckCollisionConfig( Eigen::VectorXd _q );
#endif /** _GLOBAL_STUFF_H_ */
