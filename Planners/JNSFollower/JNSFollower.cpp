/**
 * @file JNS_Follower.cpp
 * @brief Read the .h heading for details :)
 * @author A.H.Q.
 * @date March 07th, 2012
 */

#include <planning/Robot.h>
#include <planning/Object.h>
#include <kinematics/BodyNode.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "JNSFollower.h"

#include <Eigen/LU>

/**
 * @function JNSFollower
 * @brief Constructor
 */
JNSFollower::JNSFollower() {
  mCopyWorld = false;
  mWorld = NULL;
}

/**
 * @function JNSFollower
 * @brief Constructor
 */
JNSFollower::JNSFollower( planning::World &_world, 
			  Collision *_collision,
			  bool _copyWorld, 
			  double _configStep ) {
  
  mCopyWorld = _copyWorld;
  
  if( mCopyWorld ) {
    printf( "Not implemented yet. Sorry -- achq \n" );
  } else {
    mWorld = &_world;
  }
  
  mCollision = _collision;
  mConfigStep = _configStep;
}

/**
 * @function ~JNSFollower
 * @brief Destructor
 */
JNSFollower::~JNSFollower() {

  if( mCopyWorld ) {
    delete mWorld;
  }
}

/**
 * @function planPath
 * @brief Main function
 */
std::vector< Eigen::VectorXd > JNSFollower::PlanPath( int _robotId,
						      const Eigen::VectorXi &_links,
						      const Eigen::VectorXd &_start,  
						      std::string _EEName,
						      int _EEId,
						      double _res,
						      const std::vector<Eigen::VectorXd> &_workspacePath ) {
  
  mRobotId = _robotId;
  mLinks = _links;
  
  mMaxIter = 10;
  mWorkspaceThresh = _res; // An error of half the resolution
  mEENode = mWorld->mRobots[mRobotId]->getNode( _EEName.c_str() );
  mEEId = _EEId;
  
  //-- Follow the path
  std::vector< Eigen::VectorXd > configPath;
  Eigen::VectorXd q;
  
  int numPoints = _workspacePath.size();
  
  //-- Initialize	
  q = _start;
  
  for( size_t i = 1; i < numPoints; ++i ) { // start from 1 since 0 is the current start position
    if( GoToEEPosCheckCollision( q, _workspacePath[i], configPath ) == false ) {
      printf(" --(x) An error here, stop following path \n"); break;
    }
  } 
  
  printf("End of Plan Path \n");
  return configPath;
  
}

/**
 * @function GetPseudoInvJac   
 */
Eigen::MatrixXd JNSFollower::GetPseudoInvJac( Eigen::VectorXd _q ) {
  Eigen::MatrixXd Jaclin = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size() );
  Eigen::MatrixXd JaclinT = Jaclin.transpose();
  Eigen::MatrixXd Jt;
  Jt = JaclinT*( (Jaclin*JaclinT).inverse() );
  return Jt;
}

/**
 * @function GoToEEPos
 */
bool JNSFollower::GoToEEPos( Eigen::VectorXd &_q, 
			     Eigen::VectorXd _targetPos, 
			     std::vector<Eigen::VectorXd> &_workspacePath ) {
  
  Eigen::VectorXd dPos;
  Eigen::VectorXd dConfig;
  int iter;
  
  //-- Initialize
  dPos = ( _targetPos - GetEEPos(_q) ); // GetEEPos also updates the config to _q, so Jaclin use an updated value
  iter = 0;

  do {  
    Eigen::MatrixXd Jt = GetPseudoInvJac(_q);
    dConfig = Jt*dPos;
    
	/*
    if( dConfig.norm() > mConfigStep ) {
      double n = dConfig.norm();
      dConfig = dConfig *(mConfigStep/n);
    } */
    
    _q = _q + dConfig;
    _workspacePath.push_back( _q );
    
    dPos = (_targetPos - GetEEPos(_q) );
    iter++;
    
  } while( dPos.norm() > mWorkspaceThresh && iter < mMaxIter );
  
  if( iter >= mMaxIter ) { return false; }
  else { return true; }
  
}

/**
 * @function GoToEEPosCheckCollision
 */
bool JNSFollower::GoToEEPosCheckCollision( Eigen::VectorXd &_q, 
			     Eigen::VectorXd _targetPos, 
			     std::vector<Eigen::VectorXd> &_workspacePath ) {
  
  Eigen::VectorXd dPos;
  Eigen::VectorXd dConfig;
  
  //-- Initialize
  dPos = ( _targetPos - GetEEPos(_q) ); // GetEEPos also updates the config to _q, so Jaclin use an updated value

    Eigen::MatrixXd Jt;
    Eigen::MatrixXd NS_Basis; 
    Eigen::MatrixXd NS_Coeff; 
    int NS_Dim;

    GetJacStuff( _q, Jt, NS_Basis, NS_Dim );

    NS_Coeff = Eigen::MatrixXd::Ones( NS_Dim, NS_Dim );

    Eigen::VectorXd temp = Jt*dPos;

  for( int i = 0; i < NS_Dim; ++i ) {
    temp = temp + NS_Basis.col(i)*NS_Coeff(i,i)*10;
  }

    dConfig = temp;        
    _q = _q + dConfig;
        
	_workspacePath.push_back( _q );

    /// Check collision
    mWorld->mRobots[mRobotId]->setDofs( _q, mLinks );

    if( mCollision->CheckCollisions() ) {   
      printf(" --(!) Collision \n"); 
    }
  
	return true;  
}

/**
 * @function GetJacStuff
 */
void JNSFollower::GetJacStuff( const Eigen::VectorXd &_q, Eigen::MatrixXd &_Jt, Eigen::MatrixXd &_NS_Basis, int &_NS_Dim  ) {
 
 Eigen::MatrixXd Jaclin = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size() );

 Eigen::MatrixXd JaclinT = Jaclin.transpose();
 _Jt = JaclinT*( (Jaclin*JaclinT).inverse() );

 Eigen::FullPivLU<Eigen::MatrixXd> J_LU( Jaclin );
 _NS_Basis = J_LU.kernel();

 _NS_Dim = _NS_Basis.cols();
 Eigen::MatrixXd normCoeff = Eigen::MatrixXd::Zero( _NS_Dim, _NS_Dim );

  // Normalize
  for( int i = 0; i < _NS_Dim; ++i ) {
    normCoeff(i, i) = (1.0/180.0*3.1416)*1.0/_NS_Basis.col(i).norm();
  }

  _NS_Basis = _NS_Basis*normCoeff;
}

/**
 * @function GetEEPos
 */
Eigen::VectorXd JNSFollower::GetEEPos( Eigen::VectorXd _q ) {
	
  // Get current XYZ position
  mWorld->mRobots[mRobotId]->setDofs( _q, mLinks );
  mWorld->mRobots[mRobotId]->update();
  
  Eigen::MatrixXd qTransform = mEENode->getWorldTransform();
  Eigen::VectorXd qXYZ(3); qXYZ << qTransform(0,3), qTransform(1,3), qTransform(2,3);
  
  return qXYZ;
}
 
