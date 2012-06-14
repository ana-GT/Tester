/**
 * @file IKSearch.cpp
 * @brief Implementation of IK Naive Search
 * @author A. Huaman
 */

#include "IKSearch.h"

/***  CONSTS */
const double IKSearch::sMinCoeff = -10;
const double IKSearch::sMaxCoeff = 10;
const int IKSearch::sNumCoeff = 8;
const double IKSearch::sdCoeff = ( sMaxCoeff - sMinCoeff ) / (1.0*sNumCoeff);

/**
 * @function IKSearch
 * @brief Constructor
 */
IKSearch::IKSearch( planning::World &_world,
		    Collision *_collision) 
  : IK( _world, _collision ) {

  mNSNorm = 5.0 / 180.0*3.1416; // norm 10 degrees

  sCoeff = new double[sNumCoeff];
  for( int i = 0; i < sNumCoeff; ++i ) {
    sCoeff[i] = sMinCoeff + sdCoeff*i;
  }
}

/**
 * @function ~IKSearch
 * @brief Destructor
 */
IKSearch::~IKSearch() {

  if( sCoeff != NULL ){
    delete [] sCoeff;
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
    try{
      printf(" -- GoToPose %d \n", i );
      if( GoToPose2( q, _WSPath[i], jointPath ) == false ) {
	throw "GoToPose returned false"; 
      }
    } catch(const char *msg) {
      std::cout << "--Exception!: " << msg << endl;
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
    printf("-- ERROR GoToPose: Iterations: %d -- ds.norm(): %.3f \n", numIter, ds.norm() );
    return false;
  }
}  


/**
 * @function Getdq
 */
Eigen::VectorXd IKSearch::Getdq( Eigen::VectorXd _q, Eigen::VectorXd _s ) {

  //-- Direct search
  Eigen::VectorXd qp;
  Eigen::VectorXd qtemp;
  Eigen::MatrixXd ns;

  Eigen::VectorXd ds; // pose error
  ds = GetPoseError( GetPose( _q ), _s );

  qp = GetJps(_q)*ds;
  ns = GetNS_Basis( GetJ(_q) );
  
  //-- Check if this guy works
  if( CheckCollisionConfig( _q + qp ) == false ) {
    return qp;
  }
  
  //-- If not, search the nullspace
  else{
    std::cout<< "Search nullspace" << std::endl;
    for( int a = 0; a < sNumCoeff; ++a ) {
      for( int b = 0; b < sNumCoeff; ++b ) {
	for( int c = 0; c < sNumCoeff; ++c ) {
	  for( int d = 0; d < sNumCoeff; ++d ) {
	    // Coeff
	    Eigen::VectorXd coeff(4); coeff << sCoeff[a], sCoeff[b], sCoeff[c], sCoeff[d];
	    qtemp = qp + ns*coeff ;
	    
	    // Check collisions
	    if( CheckCollisionConfig( _q + qtemp ) == false && 
		GetPoseError(_s, GetPose(_q + qtemp)).norm() <  mPoseThresh ) {  
	      printf("Found it! -- a: %d b: %d c: %d d: %d \n", a, b, c, d );
	      return qtemp;
	    }
	  } // for a
	} // for b
      } // for c
    } // for d
    printf("Did not find it, using min norm dq \n");
    return qp;
  }
 
}
 
// ** PARTICULAR FUNCTIONS **

/**
 * @function GoToPose
 */
bool IKSearch::GoToPose2( Eigen::VectorXd &_q, 
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

    dq = Getdq2( q, _targetPose );
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
 * @function Getdq2
 */
Eigen::VectorXd IKSearch::Getdq2( Eigen::VectorXd _q, Eigen::VectorXd _s ) {

  //-- Direct search
  Eigen::VectorXd dq;
  Eigen::VectorXd qp;
  Eigen::VectorXd qh;
  Eigen::VectorXd qtemp;
  Eigen::MatrixXd ns;

  Eigen::VectorXd ds; // pose error
  ds = GetPoseError( GetPose( _q ), _s );

  qp = GetJps(_q)*ds;
  ns = GetNS_Basis( GetJ(_q) );

  bool found = false;
  int count = 0;
  Eigen::VectorXd mindq;
  double minJRM; double tempJRM;
  
  //-- Check if this guy works
  //if( CheckCollisionConfig( _q + qp ) == false &&
  //  IsInLim( (_q + qp) ) == true ) {
  //  return qp;
  //}
  //-- If not, search the nullspace
  //else {
    std::cout<< "Search nullspace" << std::endl;
    for( int a = 0; a < sNumCoeff; ++a ) {
      for( int b = 0; b < sNumCoeff; ++b ) {
	for( int c = 0; c < sNumCoeff; ++c ) {
	  for( int d = 0; d < sNumCoeff; ++d ) {
	    // Coeff
	    Eigen::VectorXd coeff(4); coeff << sCoeff[a], sCoeff[b], sCoeff[c], sCoeff[d];
	    qh = qp + ns*coeff ;
	    
	    // Check collisions
	    if( CheckCollisionConfig( _q + qh ) == false && 
		GetPoseError(_s, GetPose(_q + qh)).norm() <  mPoseThresh &&
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
	    }

	  } // for a
	} // for b
      } // for c
    } // for d
    if( found == true ) {
      printf("Found %d solutions, choosing last with JRM: %.3f \n", count, minJRM );
      return mindq ; 
    } else {
      printf("Did not find it, using min norm dq \n");
      return qp;
    }
    //} // else  
}

/**
 * @function GetNS_Basis
 */
Eigen::MatrixXd IKSearch::GetNS_Basis( Eigen::MatrixXd _J ) {

  Eigen::FullPivLU<Eigen::MatrixXd> J_LU( _J );
  Eigen::MatrixXd NS = J_LU.kernel();

  //-- Normalize
  int NSDim = NS.cols();
  Eigen::MatrixXd normCoeff = Eigen::MatrixXd::Zero( NSDim, NSDim );
  
  for( int i = 0; i < NSDim; ++i ) {
    normCoeff(i,i) = mNSNorm*1.0/NS.col(i).norm();
  }

  NS = NS*normCoeff;

  return NS;
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
