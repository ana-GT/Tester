/**
 * @file IKSearch_BT2.cpp
 * @author A. Huaman
 * @date 2012-06-24
 */

#include "IKSearch.h"

/**
 * @function Track_BT2
 */


std::vector<Eigen::VectorXd> IKSearch::Track_BT2( int _robotId,
						  const Eigen::VectorXi &_links,
						  const Eigen::VectorXd &_start,
						  std::string _EEName,
						  int _EEId,
						  std::vector<int> _constraints,
						  const std::vector<Eigen::VectorXd> _WSPath,
						  int _maxChain,
						  int _numCoeff,
						  double _minCoeff,
						  double _maxCoeff ) {


  //-- Get coefficients for nullspace
  GetCoeff( _numCoeff, _minCoeff, _maxCoeff );
  
  //-- Get robot and constraints info
  GetGeneralInfo( _robotId, _links, _start, _EEName, _EEId, _constraints );

  //-- Get coeff for JRM Measurement
  GetCoeff_JRM();

  //-- Track path with backtrack 2
  int numPoints = _WSPath.size();
  printf("*** Track Start - IK BT2 (%d points) *** \n", numPoints );
  std::vector<Eigen::VectorXd> qPath;
  Eigen::VectorXd q;

  bool found;
  int newIndex;

  std::vector<Eigen::VectorXd> qSet;
  std::vector<Eigen::VectorXd> cSet;

  //.. Initialize
  q = _start;
  qPath.push_back( q );
  qSet.push_back( q );
  NSSet.push_back( qSet );

  for( size_t i = 1; i < numPoints; ++i ) {
    printf("--> Workspace path [%d] \n", i );
    if( GenerateNSSet( qPath[i-1], _WSPath[i], qSet, cSet ) == false ) {
      
      //-- Backtrack
      found = false;
      for( size_t j = 0; j < NSSet[i-1].size(); ++j ) {
	if( GenerateNSSet( NSSet[i-1][j], _WSPath[i], qSet, cSet ) == true ) {
	  found = true; newIndex = j;
	  qPath[i-1] = NSSet[i-1][j];
	  break;
	}
      }
      if( found == true ) {
	printf("-- [%d] Backtrack made to element %d in NS Set \n", i, newIndex );
      }
      else {
	printf("-- [%d] Backtrack failed - Stopping \n", i );
	return qPath;
      }
    }
    
    //-- Reorder the set
    //Sort( qSet );
    NSSet.push_back( qSet );
    q = NSSet[i][0];
    qPath.push_back(q);
  }
  
  printf( "*** Track End - IK BT2 *** \n" );
  return qPath;
}

/**
 * @function GenerateNSSet
 */
bool IKSearch::GenerateNSSet( Eigen::VectorXd _q,
			      Eigen::VectorXd _s,
			      std::vector<Eigen::VectorXd> &_qSet,
			      std::vector<Eigen::VectorXd> &_coeffSet ) {
  
  //-- Reset, just in case
  _qSet.resize(0);
  _coeffSet.resize(0);
  
  //-- Brute-force search
  Eigen::VectorXd dq;
  Eigen::VectorXd qp;
  Eigen::VectorXd qh;
  Eigen::VectorXd qtemp;
  Eigen::MatrixXd ns;
  Eigen::VectorXd ds;
  Eigen::MatrixXd J;

  ds = GetPoseError( GetPose(_q), _s );
  J = GetJ( _q );
  qp = _q + GetJps(J)*ds;
  ns = GetNS_Basis( J );

  int count = 0;
  int countvalid = 0;

  for( size_t a = 0; a < mNumCoeff; ++a ) {
    for( size_t b = 0; b < mNumCoeff; ++b ) {
      for( size_t c = 0; c < mNumCoeff; ++c ) {
	for( size_t d = 0; d < 1; ++d ) {   // *** TRIAL!!! *** 
	  //-- Coefficients
	  Eigen::VectorXd coeff(4); 
	  coeff << mCoeff[a], mCoeff[b], mCoeff[c], mCoeff[d];
	  qh = qp + ns*coeff;

	  //-- First check it is legal
	  if( GetPoseError( _s, GetPose(qh) ).norm() < mPoseThresh ) {
	    countvalid++;
	    
	    //-- Then check collisions and limits
	    if( CheckCollisionConfig( qh ) == false &&
		IsInLim( qh ) == true ) {
	      //-- Add to set
	      count++;
	      _qSet.push_back( qh );
	      _coeffSet.push_back( coeff );
	    }

	  }

	} //-- for d
      } //-- for c
    } //-- for b
  } //-- for a

  if( _qSet.size() > 0 ) {
    return true;
  } else {
    return false;
  }
}
