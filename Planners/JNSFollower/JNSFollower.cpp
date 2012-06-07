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

/***  CONSTS */
const double JNSFollower::sMinCoeff = -10;
const double JNSFollower::sMaxCoeff = 10;
const int JNSFollower::sNumCoeff = 10;
const double JNSFollower::sdCoeff = ( sMaxCoeff - sMinCoeff ) / (1.0*sNumCoeff);

/**
 * @function JNSFollower
 * @brief Constructor
 */
JNSFollower::JNSFollower() {
  mCopyWorld = false;
  mWorld = NULL;

  sCoeff = new double[sNumCoeff];
  for( int i = 0; i < sNumCoeff; ++i ) {
    sCoeff[i] = sMinCoeff + sdCoeff*i;
  }
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

  sCoeff = new double[sNumCoeff];
  for( int i = 0; i < sNumCoeff; ++i ) {
    sCoeff[i] = sMinCoeff + sdCoeff*i;
  }
}

/**
 * @function ~JNSFollower
 * @brief Destructor
 */
JNSFollower::~JNSFollower() {

  if( mCopyWorld ) {
    delete mWorld;
  }

  if( sCoeff != NULL ) {
    delete [] sCoeff;
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
    Eigen::VectorXd dq_Particular = Jt*dPos;
    Eigen::VectorXd dq_Temp;

	Eigen::VectorXd minLim;
	Eigen::VectorXd maxLim;

	GetLimits( minLim, maxLim );


    // First check if the minimum solution works
    dq_Temp = dq_Particular;

    if( CheckCollisionConfig( _q + dq_Temp ) == false && IsInLim(_q + dq_Temp, minLim, maxLim ) == true ) {
      _q = _q + dq_Temp;
      _workspacePath.push_back( _q );
	  printf("Min solution \n");
      return true;
    } 
    // If not, search the nullspace
    else {
		if( IsInLim(_q + dq_Temp, minLim, maxLim ) == false ) {printf("Out of limits \n");};
      printf("--> Search nullspace \n");
      for( int a = 0; a < sNumCoeff; ++a ) {
	for( int b = 0; b < sNumCoeff; ++b ) {
	  for( int c = 0; c < sNumCoeff; ++c ) {
	    for( int d = 0; d < sNumCoeff; ++d ) {
	      // Coeff
	      Eigen::VectorXd coeff(4); coeff << sCoeff[a], sCoeff[b], sCoeff[c], sCoeff[d];
	      dq_Temp = dq_Particular + NS_Basis*coeff ;
	      
	      // Check collisions
	      if( CheckCollisionConfig( _q + dq_Temp ) == false && 
		  ( _targetPos - GetEEPos(_q + dq_Temp) ).norm() <  mWorkspaceThresh &&
			IsInLim(_q + dq_Temp, minLim, maxLim ) == true ) {  
		_q = _q + dq_Temp;
		_workspacePath.push_back( _q );
		printf("Found it! -- \n");
		return true;
	      }
	    } // for a
	  } // for b
	} // for c
      } // for d
      printf("Did not find it, damn! \n");
      _q = _q + dq_Particular;
      _workspacePath.push_back( _q );
      return true;
    }    
}

/**
 * @function CheckCollisionConfig
 * @brief Return true if there IS collisions. If it is false we are cool
 */
bool JNSFollower::CheckCollisionConfig( Eigen::VectorXd _q ) {
  
  mWorld->mRobots[mRobotId]->setDofs( _q, mLinks );
  mWorld->mRobots[mRobotId]->update();
  return mCollision->CheckCollisions();  
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
    normCoeff(i, i) = (10.0/180.0*3.1416)*1.0/_NS_Basis.col(i).norm();
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

/**
 * @function GetLimits
 */
bool JNSFollower::GetLimits( Eigen::VectorXd &_minLim, Eigen::VectorXd &_maxLim ) {

	int n = mLinks.size();
	_minLim.resize(n);
	_maxLim.resize(n);

	for( int i = 0; i < n; ++i ) {
		_minLim(i) = mWorld->mRobots[mRobotId]->getDof( mLinks[i] )->getMin();
		_maxLim(i) = mWorld->mRobots[mRobotId]->getDof( mLinks[i] )->getMax();
	}
	
	return true;
}

/**
 * @function IsInLim
 * @brief Check if links values are within limits
 */
bool JNSFollower::IsInLim( const Eigen::VectorXd &_val, const Eigen::VectorXd &_minLim, const Eigen::VectorXd &_maxLim ) {

	int n = _val.size();
	for( int i = 0; i < n; ++i ) {
		if( _val(i) < _minLim(i) || _val(i) > _maxLim(i) ) {
			return false;
		}
	}
	return true;
}
