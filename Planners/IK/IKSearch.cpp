/**
 * @file IKSearch.cpp
 * @brief Implementation of IK Naive Search
 * @author A. Huaman
 */

#include "IKSearch.h"

/**
 * @function IKSearch
 * @brief Constructor
 */
IKSearch::IKSearch( planning::World &_world,
		    Collision *_collision) 
  : IK( _world, _collision ) {

  mNSNorm = 5.0 / 180.0*3.1416; // norm 5 degrees

}

/**
 * @function ~IKSearch
 * @brief Destructor
 */
IKSearch::~IKSearch() {

  if( mCoeff != NULL ){
    delete [] mCoeff;
  }
}

/**
 * @function Track
 * @brief Main function
 */
std::vector< Eigen::VectorXd > IKSearch::Track( int _robotId,
						const Eigen::VectorXi &_links,
						const Eigen::VectorXd &_start,  
						std::string _EEName,
						int _EEId,
						std::vector<int> _constraints,
						const std::vector<Eigen::VectorXd> _WSPath ) {

  //-- Get coeff for nullspace
  GetCoeff();

  //-- Get Robot and constraints info
  GetGeneralInfo( _robotId, _links, _start,_EEName, _EEId, _constraints );

  //-- Get coeff for JRM Measurement
  GetCoeff_JRM();

  //-- Track path
  printf("*** Track Start -- IK Search *** \n");
  std::vector< Eigen::VectorXd > jointPath;
  Eigen::VectorXd q;
  
  int numPoints = _WSPath.size();
  
  //.. Initialize	
  q = _start;
  
  for( int i = 1; i < numPoints; ++i ) { 
    if( GoToPose( q, _WSPath[i], jointPath ) == false ) {
      printf( " [%d] (!) -- GoToPose returned false \n", i ); 
    }      
  } 
  
  printf(" *** Track End -- IK Search *** \n");
  return jointPath;  
}
 

/**
 * @function GoToPose
 */
bool IKSearch::GoToPose( Eigen::VectorXd &_q, 
			  Eigen::VectorXd _targetPose, 
			  std::vector<Eigen::VectorXd> &_jointPath ) {
  
  Eigen::VectorXd q; 
  Eigen::VectorXd dq;
  Eigen::VectorXd ds; 
  std::vector<Eigen::VectorXd> temp;
  int numIter;
  
  // Initialize
  q = _q;
  ds = GetPoseError( GetPose( q ), _targetPose );
  numIter = 0;
  
  while( ds.norm() > mPoseThresh && numIter < mMaxIter ) {

    dq = Getdq( q, _targetPose );
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
    printf("-- EXIT GoToPose: Iterations: %d -- ds.norm(): %.3f \n", numIter, ds.norm() );
    return false;
  }
}  

/**
 * @function Getdq
 */
Eigen::VectorXd IKSearch::Getdq( Eigen::VectorXd _q, Eigen::VectorXd _s ) {

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
  
  std::cout<< "Search nullspace" << std::endl;
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
	      if( mindq.size() == 0 ) {
		mindq = qh;
		minJRM = JRM_Measure( mindq + _q );
	      }
	      else {
		tempJRM = JRM_Measure( _q + qh );
		if( tempJRM < minJRM ) {
		  minJRM = tempJRM;
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
    printf("Found %d solutions, choosing the one with JRM: %.3f - valids: %d  \n", count, minJRM, countvalid );
    return mindq ; 
  } else {
    printf("Did not find it, using min norm dq \n");
    return qp;
  }
}

/**
 * @function GetNS_Basis
 * @brief Find the nullspace basis with SVD (used to use LU)
 */
Eigen::MatrixXd IKSearch::GetNS_Basis( Eigen::MatrixXd _J ) {

  Eigen::JacobiSVD<Eigen::MatrixXd> J_SVD( _J, Eigen::ComputeFullV );
  int NSDim = mNumLinks - mNumConstraints;	
  Eigen::MatrixXd NS = ( J_SVD.matrixV() ).rightCols( NSDim );
  
  //-- Normalize
  Eigen::MatrixXd normCoeff = Eigen::MatrixXd::Zero( NSDim, NSDim );
  for( int i = 0; i < NSDim; ++i ) {
    normCoeff(i,i) = mNSNorm*1.0/NS.col(i).norm();
  }
  
  NS = NS*normCoeff;
  
  return NS;
}


/**
 * @function NS_ChainSearch
 */
std::vector<Eigen::VectorXd> IKSearch::NS_ChainSearch( int _robotId, 
						       const Eigen::VectorXi &_links,
						       const Eigen::VectorXd _NSConf,
						       std::string _EEName,
						       int _EEId,
						       std::vector<int> _constraints,
						       int _maxChain,
						       int _numCoeff,
						       double _minCoeff,
						       double _maxCoeff ) {

  std::vector<Eigen::VectorXd> chain;

  //-- Get Robot and constraints info
  GetGeneralInfo( _robotId, _links, _NSConf,_EEName, _EEId, _constraints );

  //-- Get coeff info
  GetCoeff( _numCoeff, _minCoeff, _maxCoeff );

  //-- Get coeff for JRM Measurement
  GetCoeff_JRM();

  //-- Initialize
  Eigen::VectorXd q = _NSConf;
  chain.push_back( q );

  Eigen::VectorXd coeff;
  std::vector<Eigen::VectorXd> qSet;
  std::vector<Eigen::VectorXd> coeffSet;
  NS_Search( q, coeff, qSet, coeffSet );

  chain.push_back( q );
/*
  for( size_t i = 0; i < _maxChain; ++i ) {
    if( NS_GetSample( q, coeff ) == false ) {
      printf(" (!) Stopped chain at %d because of collision or limits \n", i );
      return chain;
    }
    chain.push_back( q );
  }
*/
  for( size_t j = 0; j < qSet.size(); ++j ) {
	Eigen::VectorXd qTemp = qSet[j];
  	for( size_t i = 0; i < _maxChain; ++i ) {
		
    	if( NS_GetSample( qTemp, coeffSet[j] ) == false ) {
      		printf(" [%d](!) Stopped chain at %d because of collision or limits \n", j, i );
			break;
    	}
    	chain.push_back( qTemp );
	}
  }

 
  printf("** Happy Chain End ** \n");
  return chain;
}


/**
 * @function NS_Search
 */
void IKSearch::NS_Search( Eigen::VectorXd &_q,
			  Eigen::VectorXd &_coeff,
			  std::vector<Eigen::VectorXd> &_configSet,
			  std::vector<Eigen::VectorXd> &_coeffSet ) {

  //-- Search
  printf("*** Start - NS Search *** \n");

  Eigen::VectorXd q;
  Eigen::VectorXd p;
  Eigen::VectorXd qh;
  Eigen::VectorXd qtemp;
  Eigen::MatrixXd J;
  Eigen::MatrixXd ns;

  Eigen::VectorXd mindq;
  double minJRM; double tempJRM;
  Eigen::VectorXd minCoeff;

  //.. Initialize
  q = _q;
  p = GetPose(q);
  J = GetJ( q );
  ns = GetNS_Basis( J );
  
  int countvalid = 0;
  int count = 0;
  
  std::cout<< "Search nullspace" << std::endl;

  for( int a = 0; a < mNumCoeff; ++a ) {
    for( int b = 0; b < mNumCoeff; ++b ) {
      for( int c = 0; c < mNumCoeff; ++c ) {
	for( int d = 0; d < 1; ++d ) { // **** TRIAL!!! ***
	  // Coeff
	  Eigen::VectorXd coeff(4); coeff << mCoeff[a], mCoeff[b], mCoeff[c], mCoeff[d];
	  qh =  ns*coeff ;
	  qtemp = q + qh;
	  if( GetPoseError( p, GetPose( qtemp )).norm() <  0.0025  ) {
	    countvalid++;
	    
	    // Check collisions and lim
		if( CheckCollisionConfig( qtemp ) == false && 
		    IsInLim( qtemp ) == true ) {  
		  count++;
		  _configSet.push_back( qtemp );
		  _coeffSet.push_back( coeff );
		  
		  if( mindq.size() == 0 && qh.norm() != 0 ) {
		    mindq = qh;
		    minJRM = JRM_Measure( qtemp );
		    minCoeff = coeff;
		  }
		  else {
		    tempJRM = JRM_Measure( qtemp );
		    if( tempJRM < minJRM && qh.norm() != 0 ) {
		      minJRM = tempJRM;
		      mindq = qh;
		      minCoeff = coeff;
		    }
		  }

		} // end if Collision	
	  } // end if GetPoseError
	  
	} // for a
      } // for b
    } // for c
  } // for d
  printf("Found %d solutions - valids: %d  \n", count, countvalid );

  // Update
  _q = q + mindq;
  _coeff = minCoeff;
}


/**
 * @function NS_GetSample
 * @brief Input the coeff of the 
 */
bool  IKSearch::NS_GetSample( Eigen::VectorXd &_q, 
			      Eigen::VectorXd _coeff ) {

  //-- Search
  Eigen::VectorXd q;
  Eigen::VectorXd qh;
  Eigen::VectorXd p;
  Eigen::MatrixXd J;
  Eigen::MatrixXd ns;

  Eigen::VectorXd mindq;
  double minJRM; double tempJRM;

  //.. Initialize
  q = _q;
  p = GetPose(q);
  J = GetJ( q );
  ns = GetNS_Basis( J );

  qh =  ns*_coeff ;
  _q = q + qh;

  if( GetPoseError( p, GetPose( _q )).norm() >  0.0025 ) { 
    return false; 
  }
  
  if( CheckCollisionConfig( _q ) == true || 
      IsInLim( _q ) == false ) {
    return false;
  }

  return true;
}


/**
 * @function GetCoeff
 * @brief Calculate the coefficients
 */
void IKSearch::GetCoeff( int _numCoeff, 
			 double _minCoeff,
			 double _maxCoeff ) {

  mMinCoeff = _minCoeff;
  mMaxCoeff = _maxCoeff;
  mNumCoeff = _numCoeff;

  if( _numCoeff % 2 == 0 ) {
	printf( "** [IKSearch::GetCoeff] (!) Num coeff even (%d). Are you sure? \n", mNumCoeff );
  }

  mdCoeff = ( mMaxCoeff - mMinCoeff ) / ( mNumCoeff - 1 );
  mCoeff = new double[mNumCoeff];

  mCoeff[0] = mMinCoeff + mdCoeff*( mNumCoeff - 1 )/2.0;
  
  for( size_t i = 1; i <= ( mNumCoeff - 1 ) / 2; ++i ) {
    mCoeff[2*i - 1] = mCoeff[0] - mdCoeff*i;
	mCoeff[2*i] = mCoeff[0] + mdCoeff*i;
  }
}
 
 
/**
 * @function GetCoeff_JRM
 * @brief Get the coefficients for the JRM_Measure function
 */
void IKSearch::GetCoeff_JRM() {
  mCoeff1_JRM = ( mJointsMin + mJointsMax ) / 2.0;
  mCoeff2_JRM = ( mJointsMax - mJointsMin ) / 2.0;
}


/**
 * @function JRM_Measure  
 */
double IKSearch::JRM_Measure( Eigen::VectorXd _conf ) {
  
  // note, I am omitting the (1/2n) factor - same result and one less operation
  Eigen::VectorXd val = ( _conf - mCoeff1_JRM ).cwiseQuotient( mCoeff2_JRM );
  return val.dot(val);
}
