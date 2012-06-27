/**
 * @file IKSearch_LJM.cpp
 * @brief LJM Special functions
 */

#include "IKSearch.h"
#include "BinaryHeap.h"

/**
 * @function Track_LA
 * @brief Look-ahead tracking
 */
std::vector< Eigen::VectorXd > IKSearch::Track_LA( int _robotId,
						   const Eigen::VectorXi &_links,
						   const Eigen::VectorXd &_start,  
						   std::string _EEName,
						   int _EEId,
						   std::vector<int> _constraints,
						   const std::vector<Eigen::VectorXd> _WSPath,
						   int _window,
						   int _maxChain,
						   int _numCoeff,
						   double _minCoeff,
						   double _maxCoeff ) {
  
  //-- Reset from (possible) old runs
  TrackReset();

  //-- Get coeff for nullspace
  GetCoeff( _numCoeff, _minCoeff, _maxCoeff );
  
  //-- Get Robot and constraints info
  GetGeneralInfo( _robotId, _links, _start,_EEName, _EEId, _constraints );
  
  //-- Get coeff for JRM Measurement
  GetCoeff_JRM();
  
  //-- Track path with look-ahead heuristic
  int numPoints = _WSPath.size();

  printf("*** Track Start -- IK  LA (%d points) - window size = %d *** \n", numPoints, _window );
  std::vector< Eigen::VectorXd > qPath;
  Eigen::VectorXd q;
  
  NSSet.resize( numPoints );
  NSPriority.resize( numPoints );
  NSValues.resize( numPoints );
  
  //.. Initialize	
  q = _start;
  qPath.push_back( q );

  NSSet[0].push_back( q );
  NSPriority[0].push_back(0);
  NSValues[0].push_back( JRM_Measure(q) );

  bool found;
  
  for( size_t i = 1; i < numPoints - _window; ++i ) {
    printf("--> Workspace path [%d] \n ", i );
    //-- Look Ahead for all
    found = LookAhead( i, 0, _window, NSSet, NSPriority, NSValues, qPath, _WSPath );

    if( found == false ) {
      printf("-- [%d] LookAhead failed -- Stopping \n", i );
      return qPath;
    }

    else {
      /** Push last q */
      qPath.push_back( NSSet[i][ NSPriority[i][0] ]);

      if( i == numPoints - _window - 1 ) {
	for( size_t j = i+1; j <= i+ _window; ++j ) {
	  qPath.push_back( NSSet[j][ NSPriority[j][0] ] );
	}
      }

      //-- Clean for next one
      ClearAhead_LA( i, _window, NSSet, NSPriority, NSValues );
    }

  }

  printf("*** Track LookAhead End - IK LA *** \n");
  return qPath;
}

/**
 * @function LookAhead
 * @brief Similar to ForwardSearch of BT but with a few mods
 */
bool IKSearch::LookAhead( int _i,
			  int _window,
			  int _maxWindow,
			  std::vector< std::vector<Eigen::VectorXd> > &_qSet,
			  std::vector< std::vector<int> > &_heapSet,
			  std::vector< std::vector<double> > &_valSet,
			  std::vector< Eigen::VectorXd > &_qPath,
			  const std::vector<Eigen::VectorXd> &_WSPath ) {
  
  int i = _i;
  int w = _window;
  int maxW = _maxWindow;
  bool found = false;
  bool b;

  if( w == 0 ) {
    b = GenerateNSSet( _qPath[i+w-1], _WSPath[i+w], _qSet[i+w], _heapSet[i+w], _valSet[i+w] );
    if( b == false ) { return false; }
  }

  //-- Clean the posterior stuff and the present value
  //ClearAhead_LA( i, w, _qSet, _heapSet, _valSet );

  while( _heapSet[ i+w ].size() > 0 &&
	 found == false ) {
    bool b = GenerateNSSet( _qSet[ i+w ][ _heapSet[i+w][0] ], 
			    _WSPath[i+w+1], 
			    _qSet[i+w+1], _heapSet[i+w+1], _valSet[i+w+1] );

    //-- If found 
    if( b == true ) {
      //-- Still not in the frontier (i-w,... i-3, i-2, i-1, i)
      if( w < maxW - 1 ) {
	found = LookAhead( i, w+1, maxW, _qSet, _heapSet, _valSet, _qPath, _WSPath );
      }
      else {
	found = true;
      }
    }
    /** Go next one */
    else {
      Heap_Pop( _heapSet[i+w], _valSet[i+w] );
    }
      
  } // end main while


  //-- If it got out of while but did not find a solution
  if( found == false ) {
    if( w == 0 ) {
      return false;
    }
    else {
      Heap_Pop( _heapSet[ i + w -1 ], _valSet[ i+ w - 1 ] );
    }
  }

  //-- Got out of the loop and found a solution
  else {
      return true;
  }

}

/**
 * @function ClearAhead
 */
void IKSearch::ClearAhead_LA( int _i, 
			      int _w,
			      std::vector< std::vector<Eigen::VectorXd> > &_qSet,
			      std::vector< std::vector<int> > &_heapSet,
			      std::vector< std::vector<double> > &_valSet ) {

    for( size_t j = 1; j <= _w; ++j ) {
      _qSet[ _i + j ].resize(0);
      _heapSet[ _i + j ].resize(0);
      _valSet[ _i + j ].resize(0);
    }
  return;
}

