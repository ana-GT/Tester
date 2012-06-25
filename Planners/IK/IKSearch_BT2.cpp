/**
 * @file IKSearch_BT2.cpp
 * @author A. Huaman
 * @date 2012-06-24
 */

#include "IKSearch.h"
#include "BinaryHeap.h"

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

  std::vector<Eigen::VectorXd> qSet;
  std::vector<int> heapSet;
  std::vector<double> valSet;

  //.. Initialize
  q = _start;
  qPath.push_back( q );

  qSet.push_back( q );
  NSSet.push_back( qSet );

  heapSet.push_back(0);
  NSPriority.push_back( heapSet );

  valSet.push_back( JRM_Measure(q) );
  NSValues.push_back( valSet);

  for( size_t i = 1; i < numPoints; ++i ) {
    printf("--> Workspace path [%d] \n", i );
    if( GenerateNSSet( qPath[i-1], _WSPath[i], qSet, heapSet, valSet ) == false ) {
      
      //-- Backtrack
      found = false;
      while( true && NSPriority[i-1].size() > 0 ) {
	found = GenerateNSSet( NSSet[i-1][ NSPriority[i-1][0] ], _WSPath[i], qSet, heapSet, valSet );
	if( found == true ) {
	  qPath[i-1] = NSSet[i-1][ NSPriority[i-1][0] ];
	  break;
	}
	else {
	  Heap_Pop( NSPriority[i-1], NSValues[i-1] );
	}
      }
      if( found == true ) {
	printf("-- [%d] Backtrack made to element %d in NS Set \n", i, NSPriority[i-1][0] );
      }
      else {
	printf("-- [%d] Backtrack failed - Stopping \n", i );
	return qPath;
      }
    }
    
    
    NSSet.push_back( qSet );
    NSPriority.push_back( heapSet );
    NSValues.push_back( valSet );
    q = NSSet[i][ NSPriority[i][0] ];
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
			      std::vector<int> &_prioritySet,
			      std::vector<double> &_valSet ) {
  
  //-- Reset, just in case
  _qSet.resize(0);
  _prioritySet.resize(0);
  _valSet.resize(0);

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
	      // _coeffSet.push_back( coeff );
	    }

	  }

	} //-- for d
      } //-- for c
    } //-- for b
  } //-- for a

  if( _qSet.size() > 0 ) {
    SortNS( _qSet, _valSet, _prioritySet );
    return true;
  } else {
    return false;
  }
}

/**
 * @function SortNS
 */
void IKSearch::SortNS( std::vector<Eigen::VectorXd> _configs, 
		       std::vector<double> &_vals,
		       std::vector<int> &_priority) {
  
  int n = _configs.size();

  for( size_t i = 0; i < n; ++i ) {
    _vals.push_back( JRM_Measure(_configs[i]) );
    Heap_Insert( i, _priority, _vals );
  }
  
}

