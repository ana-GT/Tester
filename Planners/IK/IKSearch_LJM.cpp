/**
 * @file IKSearch_LJM.cpp
 * @brief LJM Special functions
 */

#include "IKSearch.h"
#include <algorithm>

/**
 * @function TrackLJM
 * @brief Main function
 */
std::vector< Eigen::VectorXd > IKSearch::Track_LJM( int _robotId,
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
  
  //-- Get coeff for nullspace
  GetCoeff( _numCoeff, _minCoeff, _maxCoeff );
  
  //-- Get Robot and constraints info
  GetGeneralInfo( _robotId, _links, _start,_EEName, _EEId, _constraints );
  
  //-- Get coeff for JRM Measurement
  GetCoeff_JRM();
  
  //-- Track path
  printf("*** Track Start -- IK Search LJM *** \n");
  std::vector< Eigen::VectorXd > jointPath;
  Eigen::VectorXd q;
  
  int numPoints = _WSPath.size();
  
  //.. Initialize	
  q = _start;
  
  int mWindow = 2;
  for( size_t i = 1; i < numPoints - mWindow + 1; ++i ) {
    printf("[%d] ... Tracking LJM ... \n ", i );
    std::vector<Eigen::VectorXd> windowPathSubset;
    for( size_t j = i; j < i + mWindow; ++j ) {
      windowPathSubset.push_back( _WSPath[j] );
    } 
 
    if( GoToPose_LJM( q, windowPathSubset, jointPath ) == false ) {
      printf( " [%d] (!) -- GoToPose returned false \n", i );
      break;
    }      
  } 
  
  printf(" *** Track End -- IK Search *** \n");
  return jointPath;  
}

/**
 * @function GoToPose_LJM
 */
bool IKSearch::GoToPose_LJM( Eigen::VectorXd &_q, 
			     std::vector<Eigen::VectorXd> _targetWindow, 
			     std::vector<Eigen::VectorXd> &_jointPath ) {
  
  Eigen::VectorXd q; 
  Eigen::VectorXd dq;
  Eigen::VectorXd ds; 
  std::vector<Eigen::VectorXd> temp;
  
  // Initialize
  q = _q;
  ds = GetPoseError( GetPose( q ), _targetWindow[0] );

  // TEMP VARIABLES

  std::vector<Eigen::VectorXd> qSet;
  std::vector<Eigen::VectorXd> cSet;
  std::vector<Eigen::VectorXd> qSet1;
  std::vector<Eigen::VectorXd> cSet1;

  // Find the possible solutions for targetWindow[0]
  Getdq_LJM( q, _targetWindow[0], qSet, cSet );

  for( int i = 0; i < qSet.size(); ++i ) {
    if( Getdq_LJM( qSet[i], _targetWindow[1], qSet1, cSet1 ) == true ) {
      _q = qSet[i];
      temp.push_back( _q );
      _jointPath.insert( _jointPath.end(), temp.begin(), temp.end() );
      return true;
    }    
  }

  printf( "Did not find solution with window = 2 \n" );
  return false;
}  

/**
 * @function Getdq_LJM
 */
bool IKSearch::Getdq_LJM( Eigen::VectorXd _q, 
			  Eigen::VectorXd _s,
			  std::vector<Eigen::VectorXd> &_configSet,
			  std::vector<Eigen::VectorXd> &_coeffSet ) {

  std::vector<double> _valSet;

  //-- Direct search
  Eigen::VectorXd dq;
  Eigen::VectorXd qp;
  Eigen::VectorXd qh;
  Eigen::VectorXd qtemp;
  Eigen::MatrixXd ns;
  Eigen::VectorXd ds;
  Eigen::MatrixXd J;

  ds = GetPoseError( GetPose( _q ), _s );
  J = GetJ(_q);
  qp = GetJps(J)*ds;
  ns = GetNS_Basis( J );

  bool found = false;
  int count = 0;
  int countvalid = 0;

  Eigen::VectorXd mindq;
  double minJRM; double tempJRM;
  
  std::cout<< "Getdq_LJM" << std::endl;
  for( int a = 0; a < mNumCoeff; ++a ) {
    for( int b = 0; b < mNumCoeff; ++b ) {
      for( int c = 0; c < mNumCoeff; ++c ) {
	for( int d = 0; d < 1; ++d ) { // *** TRIAL!! ***
	  // Coeff
	  Eigen::VectorXd coeff(4); coeff << mCoeff[a], mCoeff[b], mCoeff[c], mCoeff[d];
	  qh = qp + ns*coeff ;
	  
	  if( GetPoseError(_s, GetPose(_q + qh)).norm() <  mPoseThresh  ) {
	    countvalid++;
	    
	    // Check collisions
	    if( CheckCollisionConfig( _q + qh ) == false && 
		IsInLim( (_q + qh) ) == true ) {  

	      found = true; count++;
	      double val = JRM_Measure( _q + qh );

	      _configSet.push_back( _q + qh );
	      _coeffSet.push_back( coeff );
	      _valSet.push_back( val );

	      if( mindq.size() == 0 ) {
		mindq = qh;
		minJRM = val;
	      }
	      else {
		if( val < minJRM ) {
		  minJRM = val;
		  mindq = qh;
		}
	      }
	      
	    } // end if Collision	
	  } // end if GetPoseError
	  
	} // for a
      } // for b
    } // for c
  } // for d

  if( found == true ) {
    _configSet[0] = mindq + _q;
    SortVector( _valSet );
  }
  return found;
}

/**
 * @function SortVector
 */
std::vector<int> IKSearch::SortVector( std::vector<double> _vals ) {

  int n = _vals.size();
  std::vector<int> indices(n);
  for( size_t i = 0; i < n; ++i ) {
    indices[i] = i;
  }

  // Sortea
  auto bool f = [](int i, int j ) { return (_vals[i] < _vals[j]); } 
  std::sort( indices.begin(), 
	     indices.begin() +5, f() );
  
  return indices;
}
