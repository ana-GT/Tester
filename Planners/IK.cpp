/**
 * @file IK.cpp
 * @author A. Huaman
 * @brief Implementation of IK functions
 * @date 2012-06-08
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
  mMaxIter = 3;
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
  printf("*** Robot information *** \n");
  mRobotId = _robotId;
  mLinks = _links;
  mNumLinks = mLinks.size();
  mEENode = mWorld->mRobots[mRobotId]->getNode( _EEName.c_str() );
  mEEId = _EEId;
 

  //-- Get constraints information
  printf("*** Constraint information *** \n");
  mConstraints.resize(0);
  for( int i = 0; i < _constraints.size(); ++i ) {
    if( _constraints[i] != 0 ) {
      mConstraints.push_back( i );
    }
  }

  mNumConstraints = mConstraints.size();
    
  //-- Track path
  printf("*** Track Path *** \n");
  std::vector< Eigen::VectorXd > jointPath;
  Eigen::VectorXd q;
  
  int numPoints = _WSPath.size();
  
  //.. Initialize	
  q = _start;
  
  for( int i = 1; i < numPoints; ++i ) { 
    try{
      if( GoToPose( q, _WSPath[i], jointPath ) == false ) {
	throw "GoToPose returned false"; 
      }
    } catch(const char *msg) {
      std::cout << "--Exception!: " << msg << endl;
    }
      
  } 
  
  printf(" *** Track End *** \n");
  return jointPath;
  
}

/**
 * @function GoToPose
 */
bool IK::GoToPose( Eigen::VectorXd &_q, 
		   Eigen::VectorXd _targetPose, 
		   std::vector<Eigen::VectorXd> &_jointPath ) {
  
  Eigen::VectorXd q; // current config
  Eigen::VectorXd dq;
  Eigen::VectorXd ds; // pose error
  std::vector<Eigen::VectorXd> temp;
  int numIter;

  // Initialize
  q = _q;
  ds = GetPoseError( GetPose( q ), _targetPose );
  numIter = 0;

  while( ds.norm() > mPoseThresh && numIter < mMaxIter ) {

    dq = GetGeneralIK( q, ds );
    q = q + dq; 
    temp.push_back( q );
    ds = GetPoseError( GetPose(q), _targetPose );
    numIter++;
  };
  
  // Output
  if( numIter < mMaxIter && ds.norm() < mPoseThresh ) {
    _jointPath.insert( _jointPath.end(), temp.begin(), temp.end() );
    _q = q;
    return true;
  } 
  else{
    printf("-- ERROR GoToPose: Iterations: %d -- ds.norm(): %.3f \n", numIter, ds.norm() );
    return false;
  }

}  

/**
 * @function GetPose
 * @brief Get x,y,z,r,p,y (only the ones specified in your constraint)
 */
Eigen::VectorXd IK::GetPose( Eigen::VectorXd _q ) {

  Eigen::VectorXd temp(6);
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
  ds = ( _s2 - _s1 );

  return ds;
}

/**
 * @function GetGeneralIK
 */
Eigen::VectorXd IK::GetGeneralIK( Eigen::VectorXd _q, Eigen::VectorXd _ds ) {
  return GetJps(_q)*_ds;
}

/**
 * @function GetJ
 * @brief Calculate Jacobian
 */
Eigen::MatrixXd IK::GetJ( const Eigen::VectorXd &_q ) {

  mWorld->mRobots[mRobotId]->update();
  
  Eigen::MatrixXd J( 6, mNumLinks );
  Eigen::MatrixXd Jc( mNumConstraints, mNumLinks );

  J.topLeftCorner( 3, mNumLinks ) = mEENode->getJacobianLinear().topRightCorner( 3, mNumLinks );
  J.bottomLeftCorner( 3, mNumLinks ) = mEENode->getJacobianAngular().topRightCorner( 3, mNumLinks );
  
  for( size_t i = 0; i < mNumConstraints; ++i ) {
      Jc.row(i) = J.row( mConstraints[i] );
  } 

  return Jc;
}

/**
 * @function GetJps
 * @brief Calculate Pseudo-Inverse Jacobian
 */
Eigen::MatrixXd IK::GetJps( const Eigen::VectorXd _q ) {

  Eigen::MatrixXd J = GetJ(_q);
  Eigen::MatrixXd Jt = J.transpose();

  return Jt*( (J*Jt).inverse() );
}									    
									   


