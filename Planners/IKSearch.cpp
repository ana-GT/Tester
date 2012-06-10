/**
 * @file IKSearch.cpp
 * @brief Implementation of IK Naive Search
 * @author A. Huaman
 */

#include "IKSearch.h"

/***  CONSTS */
const double IKSearch::sMinCoeff = -10;
const double IKSearch::sMaxCoeff = 10;
const int IKSearch::sNumCoeff = 10;
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
 * @function Getdq
 */
Eigen::VectorXd IKSearch::Getdq( Eigen::VectorXd _q, Eigen::VectorXd _s ) {

  //-- Direct search
  Eigen::VectorXd qp;
  Eigen::VectorXd qtemp;
  Eigen::MatrixXd ns;

  Eigen::VectorXd ds; // pose error
  ds = GetPoseError( GetPose( _q ), _s );

  qp = GetGeneralIK(_q, ds);
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
	    if( CheckCollisionConfig( _q + qtemp ) == false && GetPoseError(_s, GetPose(_q + qtemp)).norm() <  mPoseThresh) {  
	      printf("Found it! -- \n");
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
