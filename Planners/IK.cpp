/**
 * @file IK.cpp
 */

#include <planning/Robot.h>
#include <planning/Object.h>
#include <kinematics/BodyNode.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include <utils/UtilsRotation.h>
#include "IK.h"

#include <Eigen/LU>

/**
 * @function IK
 * @brief Constructor
 */
IK::IK() {
  mWorld = NULL;
}

/**
 * @function IK
 * @brief Constructor
 */
IK::IK( planning::World &_world, 
	Collision *_collision ) {
  
  mWorld = &_world;
  mCollision = _collision;

  /** CHANGE THIS ACCORDING TO THE PROBLEM */
  mPoseThresh = 0.01;
  mMaxIter = 5;
}

/**
 * @function ~IK
 * @brief Destructor
 */
IK::~IK() {

}

/**
 * @function Track
 * @brief Main function
 */
std::vector< Eigen::VectorXd > IK::Track( int _robotId,
					  const Eigen::VectorXi &_links,
					  const Eigen::VectorXd &_start,  
					  std::string _EEName,
					  int _EEId,
					  std::vector<int> _constraints,
					  const std::vector<Eigen::VectorXd> _WSPath ) {

  //-- Get robot information
  mRobotId = _robotId;
  mLinks = _links;
  mNumLinks = mLinks.size();
  mEENode = mWorld->mRobots[mRobotId]->getNode( _EEName.c_str() );
  mEEId = _EEId;
  mNumConstraints = _constraints.size();

  //-- Get constraints information
  mConstraints.resize(0);
  for( size_t i = 0; i < _constraints.size(); ++i ) {
    if( _constraints[i] != 0 ) {
      mConstraints.push_back( i );
    }
  }
    
  //-- Track path
  std::vector< Eigen::VectorXd > jointPath;
  Eigen::VectorXd q;
  
  int numPoints = _WSPath.size();
  
  //-- Initialize	
  q = _start;
  
  for( size_t i = 1; i < numPoints; ++i ) { 
    try{
      if( GoToPose( q, _WSPath[i], jointPath ) == false ) {
	throw "GoToPose returned false"; 
      }
    } catch(string msg) {
      std::cout << "--MESSAGE: " << msg << endl;
    }
      
  } 
  
  printf(" ----- Track End ----- \n");
  return jointPath;
  
}

/**
 * @function GoToPose
 */
bool IK::GoToPose( Eigen::VectorXd &_q, 
		   Eigen::VectorXd _targetPose, 
		   std::vector<Eigen::VectorXd> &_jointPath ) {
  
  
  Eigen::VectorXd q; // curent config
  Eigen::VectorXd dq;
  Eigen::VectorXd s; // current pose
  Eigen::VectorXd ds; // pose error
  std::vector<Eigen::VectorXd> temp;
  int numIter;

  // Initialize
  q = _q;
  s = GetPose( q );
  ds = GetPoseError( s, _targetPose );
  numIter = 0;

  while( ds.norm() > mPoseThresh && numIter < mMaxIter ) {
    dq = GetGeneralIK( q, ds );
    q += dq; 
    temp.push_back( q );
    s = GetPose(q);
    ds = GetPoseError( s, _targetPose );
    numIter++;
  };
  
  // Output
  if( numIter < mMaxIter && ds.norm() < mPoseThresh ) {
    _jointPath = temp;
    return true;
  } 
  else{
    return false;
  }

}  

/**
 * @function GetPose
 * @brief Get x,y,z,r,p,y (only the ones specified in your constraint
 */
Eigen::VectorXd IK::GetPose( Eigen::VectorXd _q ) {

  Eigen::VectorXd temp;
  Eigen::VectorXd s;

  mWorld->mRobots[mRobotId]->setDofs( _q, mLinks );
  mWorld->mRobots[mRobotId]->update();

  Eigen::MatrixXd Tw; Tw = mEENode->getWorldTransform();
  Eigen::Matrix3d Rot; Rot = Tw.block(0,0,3,3);
  Eigen::Vector3d rpy; rpy = utils::rotation::matrixToEuler( Rot, utils::rotation::XYZ ); 

  temp << Tw(0,3), Tw(1,3), Tw(2,3), rpy(0), rpy(1), rpy(2);

  // Return the constrained variables
  s.resize(mNumConstraints);

  for( int i = 0; i < mNumConstraints;i++ ) {
    int n = mConstraints[i];
    s(i) = temp(n);
  }

  return s;
}

/**
 * @function GetPoseError
 */
Eigen::VectorXd IK::GetPoseError( Eigen::VectorXd _s1, Eigen::VectorXd _s2 ) {
  Eigen::VectorXd ds;
  ds = ( _s1 - _s2 );

  return ds;
}

/**
 * @function GetGeneralIK
 */
Eigen::VectorXd IK::GetGeneralIK( Eigen::VectorXd _q, Eigen::VectorXd _ds ) {
  Eigen::VectorXd dq;
  dq = GetJps(_q)*_ds;
  return dq;
}

/**
 * @function GetJ
 */
Eigen::MatrixXd IK::GetJ( const Eigen::VectorXd &_q ) {
  Eigen::MatrixXd Jlin = mEENode->getJacobianLinear();
  Eigen::MatrixXd Jang = mEENode->getJacobianAngular();
  
  Eigen::MatrixXd J( mNumConstraints, mNumLinks );

  for( size_t i = 0; i < mNumConstraints; ++i ) {
    if( mConstraints[i] < 3 ) {
      J.row(i) = Jlin.row(i);
    }
    else {
      J.row(i) = Jang.row( mConstraints[i] - 3 );
    }
  } 

  return J;
}

/**
 * @function GetJps
 */
Eigen::VectorXd IK::GetJps( const Eigen::MatrixXd _q ) {
  
  Eigen::VectorXd J = GetJ(_q);
  Eigen::VectorXd Jps;
  Eigen::MatrixXd Jt = J.transpose();
  Jps = Jt*( (J*Jt).inverse() );

  return Jps;
}									    
									   


