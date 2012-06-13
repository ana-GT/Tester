/**
 * @file IKGradient.cpp
 * @brief Implementation of IK Gradient (Liegeois)
 * @author A. Huaman
 */

#include "IKGradient.h"


/**
 * @function IKGradient
 * @brief Constructor
 */
IKGradient::IKGradient( planning::World &_world,
			Collision *_collision) 
  : IK( _world, _collision ) {
}

/**
 * @function ~IKGradient
 * @brief Destructor
 */
IKGradient::~IKGradient() {
}

/**
 * @function Track
 * @brief
 */
std::vector<Eigen::VectorXd> IKGradient::Track( int _robotId,
						const Eigen::VectorXi &_links,
						const Eigen::VectorXd &_start,
						std::string _EEName,
						int _EEId,
						std::vector<int> _constraints,
						const std::vector<Eigen::VectorXd> _WSPath,
						double _wJRA,
						double _wManip ) {

  //-- Get the weights
  mW_JRA = _wJRA;
  mW_Manip = _wManip;

  //-- Get Robot and constraints info
  GetGeneralInfo( _robotId, _links, _start,_EEName, _EEId, _constraints );
    
  //-- Get some data for the functions to minimize
  GetCoeff_dJRA();

  //-- Track path
  printf( "*** Track Start -- IK Gradient*** \n" );
  printf( " Weights: JRA: %.3f Manip: %.3f \n", mW_JRA, mW_Manip );
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
  
  printf(" *** Track End -- IK Gradient *** \n");
  return jointPath;

}

/**
 * @function GoToPose
 */
bool IKGradient::GoToPose( Eigen::VectorXd &_q, 
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
Eigen::VectorXd IKGradient::Getdq( Eigen::VectorXd _q, 
				   Eigen::VectorXd _s ) {
  printf("Call Gradient getdq \n");

  //-- Direct search
  Eigen::VectorXd qp;
  Eigen::VectorXd qh;
  Eigen::VectorXd ds; // pose error
  ds = GetPoseError( GetPose( _q ), _s );

  Eigen::MatrixXd J = GetJ(_q);
  Eigen::MatrixXd Jps = GetJps( J );
  qp = Jps*ds;
  
  //-- Don't check collision by now
  qh = mW_JRA*( Eigen::MatrixXd::Identity(mNumLinks, mNumLinks) - Jps*J  )*dJRA( _q );

  return qp + qh;
}

// ** SPECIFIC FUNCTIONS **

/**
 * @function dJRA
 */
Eigen::VectorXd IKGradient::dJRA( Eigen::VectorXd _conf ) {

  return ( _conf.cwiseProduct(mCoeff1_JRA) - mCoeff2_JRA );
}


/**
 * @function GetCoeff_dJRA
 * @brief Explicit
 */
void IKGradient::GetCoeff_dJRA() {

  mCoeff1_JRA.resize( mNumLinks );
  mCoeff2_JRA.resize( mNumLinks );

  for( size_t i = 0 ; i < mNumLinks; ++i ) {
    mCoeff1_JRA(i) = -1.0/( mNumLinks*pow(mJointsMax(i) - mJointsMin(i), 2) );
    mCoeff2_JRA(i) = mCoeff1_JRA(i)*( mJointsMin(i) + mJointsMax(i) )/2.0;
  }
  std::cout << "Coeff 1: " << mCoeff1_JRA.transpose() << std::endl;
  std::cout << "Coeff 2: " << mCoeff2_JRA.transpose() << std::endl;
}

