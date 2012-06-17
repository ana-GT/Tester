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

  mNSNorm = 5.0 / 180.0*3.1416; // norm 10 degrees

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
    for( int a = 0; a < mNumCoeff; ++a ) {
      for( int b = 0; b < mNumCoeff; ++b ) {
	for( int c = 0; c < mNumCoeff; ++c ) {
	  for( int d = 0; d < mNumCoeff; ++d ) {
	    // Coeff
	    Eigen::VectorXd coeff(4); coeff << mCoeff[a], mCoeff[b], mCoeff[c], mCoeff[d];
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
  int countvalid = 0;
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
    for( int a = 0; a < mNumCoeff; ++a ) {
      for( int b = 0; b < mNumCoeff; ++b ) {
	for( int c = 0; c < mNumCoeff; ++c ) {
	  for( int d = 0; d < mNumCoeff; ++d ) {
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
  NS_Search( q, coeff );
  chain.push_back( q );

  for( size_t i = 0; i < _maxChain; ++i ) {
    if( NS_GetSample( q, coeff ) == false ) {
      printf("Stopped chain at %d because of collision or limits \n", i );
      return chain;
    }
    chain.push_back( q );
  }
 
  printf("Happy Chain End!!! \n");
  return chain;
}

/**
 * @function NS_Search
 * @brief Find an area of the self-motion configurations
 */
std::vector<Eigen::VectorXd> IKSearch::NS_Search( Eigen::VectorXd &_q,
						  Eigen::VectorXd &_coeff ) {

  //-- Search
  printf("*** Start - NS Search *** \n");
  std::vector< Eigen::VectorXd > NSConfigs;
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
	for( int d = 0; d < mNumCoeff; ++d ) {
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
		  NSConfigs.push_back( qtemp );

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
  _q = q + mindq;
  _coeff = minCoeff;
  std::cout << "New q:" << _q.transpose() << std::endl;
  std::cout << "min dq:" << mindq.transpose() << std::endl;
  std::cout << "min Coeff: " << _coeff.transpose() << std::endl;
  return NSConfigs;
}


///////////////////////////
// TEMPORAL GUYS //

/**
 */
std::vector<Eigen::VectorXd> IKSearch::NS_ChainSearchTest( int _robotId, 
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
  std::vector<Eigen::VectorXd> coeffSet;
  std::vector<Eigen::VectorXd> qSet;
  qSet = NS_SearchTest( q, coeff, coeffSet );


  for( size_t i = 0; i < qSet.size(); ++i ) {
    Eigen::VectorXd qtemp = qSet[i];
    chain.push_back( qtemp ); 
    for( size_t j = 0; j < _maxChain; ++j ) {      
      if( NS_GetSample( qtemp, coeffSet[i] ) == false ) {
	printf("[%d] Stopped chain at %d because of collision or limits \n", i, j );
	break;
      }
      chain.push_back( qtemp );
    }
  }
 
  printf("Happy Chain End!!! \n");
  return chain;
  
}

/**
 * @function NS_SearchTest
 */
std::vector<Eigen::VectorXd> IKSearch::NS_SearchTest( Eigen::VectorXd &_q,
						      Eigen::VectorXd &_coeff,
						      std::vector<Eigen::VectorXd> &_coeffSet ) {

  //-- Search
  printf("*** Start - NS Search Test *** \n");
  std::vector< Eigen::VectorXd > NSConfigs;
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
	for( int d = 0; d < mNumCoeff; ++d ) {
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
		  NSConfigs.push_back( qtemp );
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
  _q = q + mindq;
  _coeff = minCoeff;
  std::cout << "New q:" << _q.transpose() << std::endl;
  std::cout << "min dq:" << mindq.transpose() << std::endl;
  std::cout << "min Coeff: " << _coeff.transpose() << std::endl;
  return NSConfigs;
}

//////////////////////////







/**
 * @function NS_GetSample
 * @brief Input the coeff of the 
 */
bool  IKSearch::NS_GetSample( Eigen::VectorXd &_q, 
			      Eigen::VectorXd _coeff ) {

  //-- Search
  printf("*** Start - NS Get Sample *** \n");
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
  //std::cout << "[GetSample] New q:" << _q.transpose() << std::endl;
  //std::cout << "[GetSample] min dq:" << qh.transpose() << std::endl;

    if( GetPoseError( p, GetPose( _q )).norm() >  0.0025 )
      { printf("NS Sample got a error pose, send false \n");
	return false; }

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

  mdCoeff = ( mMaxCoeff - mMinCoeff ) / ( 1.0*mNumCoeff );
  mCoeff = new double[mNumCoeff];
  for( size_t i = 0; i < mNumCoeff; ++i ) {
    mCoeff[i] = mMinCoeff + mdCoeff*i;
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
