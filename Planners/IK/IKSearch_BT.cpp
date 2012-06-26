/**
 * @file IKSearch_BT.cpp
 * @author A. Huaman
 * @date 2012-06-26
 */

#include "IKSearch.h"
#include "BinaryHeap.h"

/**
 * @function Track_BT
 */
std::vector<Eigen::VectorXd> IKSearch::Track_BT( int _robotId,
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

  //-- Get coefficients for nullspace
  GetCoeff( _numCoeff, _minCoeff, _maxCoeff );
  
  //-- Get robot and constraints info
  GetGeneralInfo( _robotId, _links, _start, _EEName, _EEId, _constraints );

  //-- Get coeff for JRM Measurement
  GetCoeff_JRM();

  //-- Track path with backtrack 2
  int numPoints = _WSPath.size();

  printf("*** Track Start - IK BT (%d points) - window size = %d *** \n", numPoints, _window );
  std::vector<Eigen::VectorXd> qPath;
  Eigen::VectorXd q;

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

  bool found;

  for( size_t i = 1; i < numPoints; ++i ) {
    printf("--> Workspace path [%d] \n", i );
    if( GenerateNSSet( qPath[i-1], _WSPath[i], qSet, heapSet, valSet ) == false ) {
      
      //-- Backtrack
      found = BackTrack( i, 1, _window, NSSet, NSPriority, NSValues, qPath, _WSPath );

      if( found == true ) {
	printf("-- [%d] Backtrack made to element in NS Set \n", i );
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
  
  printf( "*** Track End - IK BT *** \n" );
  return qPath;
}

/**
 * @function BackTrack
 */
bool IKSearch::BackTrack( int _i, 
			  int _window, 
			  int _maxWindow, 
			  std::vector< std::vector<Eigen::VectorXd> > &_qSet,
			  std::vector< std::vector<int> > &_heapSet,
			  std::vector< std::vector<double> > &_valSet,
			  std::vector< Eigen::VectorXd > &_qPath,
			  const std::vector<Eigen::VectorXd> &_WSPath ) {

  /** If window < maxWindow */
  if( _window < _maxWindow ) {
    if( ForwardSearch( _i, _window, _maxWindow, _qSet, _heapSet, _valSet, _qPath, _WSPath ) == false ) {
      return BackTrack( _i, _window + 1, _maxWindow, _qSet, _heapSet, _valSet, _qPath, _WSPath );
    }
    else {
      return true;
    }
  }
  /** If  window > maxWindow*/
  else {
    return ForwardSearch( _i, _window, _maxWindow, _qSet, _heapSet, _valSet, _qPath, _WSPath );
  }
}

/**
 * @function ForwardSearch
 */
bool IKSearch::ForwardSearch( int _i,
			      int _window,
			      int _maxWindow,
			      std::vector< std::vector<Eigen::VectorXd> > &_qSet,
			      std::vector< std::vector<int> > &_heapSet,
			      std::vector< std::vector<double> > &_valSet,
			      std::vector< Eigen::VectorXd > &_qPath,
			      const std::vector<Eigen::VectorXd> &_WSPath ) {

  bool found = false;
  int i = _i;
  int w = _window;
  int maxW = _maxWindow;

  //-- Clean the posterior stuff and the present value
  ClearAhead( i, w, _qSet, _heapSet, _valSet );

  while( _heapSet[ i-w ].size() > 0 &&
	 found == false ) {
    bool b = GenerateNSSet( _qSet[ i-w ][ _heapSet[i-w][0] ], 
			    _WSPath[i-w+1], 
			    _qSet[i-w+1], _heapSet[i-w+1], _valSet[i-w+1] );

    //-- If found 
    if( b == true ) {
      //-- Still not in the frontier (i-w,... i-3, i-2, i-1, i)
      if( w > 1 ) {
	found = ForwardSearch( i, w-1, maxW, _qSet, _heapSet, _valSet, _qPath, _WSPath );
      }
      else {
	found = true;
      }
    }
    /** Go next one */
    else {
      Heap_Pop( _heapSet[i-w], _valSet[i-w] );
    }
      
  } // end main while


  //-- If it got out of while but did not find a solution
  if( found == false ) {
    if( w == maxW ) {
      return false;
    }
    else {
      Heap_Pop( _heapSet[ i - (w+1)], _valSet[ i-(w+1) ] );
    }
  }

  //-- Got out of the loop and found a solution
  else {
    if( w == maxW ) {
      UpdateBackTrack( i, w, _qSet, _heapSet, _qPath );
    }
      return true;
  }

}

/**
 * @function UpdateBackTrack
 * @brief Update the backtracked values that were renewed 
 */
void IKSearch::UpdateBackTrack( int _i,
				int _window,
				std::vector< std::vector<Eigen::VectorXd> > &_qSet,
				const std::vector< std::vector<int> > &_heapSet,
				std::vector< Eigen::VectorXd > &_qPath ) {

  for( size_t ind = 1; ind <= _window; ++ind ) {
    _qPath[ _i - ind ] = _qSet[_i - ind][ _heapSet[_i - ind][0] ];
  }
}

/**
 * @function ClearAhead
 */
void IKSearch::ClearAhead( int _i, 
			   int _w,
			   std::vector< std::vector<Eigen::VectorXd> > &_qSet,
			   std::vector< std::vector<int> > &_heapSet,
			   std::vector< std::vector<double> > &_valSet ) {
  
  if( _w == 0 ) {
    return;
  }

  else {
    for( size_t j = 1; j <= _w; ++j ) {
      _qSet[ _i-j ].resize(0);
      _heapSet[ _i-j ].resize(0);
      _valSet[ _i-j ].resize(0);

      Heap_Pop( _heapSet[ _i-(_w+1) ], _valSet[ _i-(_w+1) ] );
    }
  }
}
