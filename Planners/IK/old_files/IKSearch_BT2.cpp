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
      while( found == false && NSPriority[i-1].size() > 0 ) {
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
 * @function Track_BT3
 */
std::vector<Eigen::VectorXd> IKSearch::Track_BT3( int _robotId,
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
  printf("*** Track Start - IK BT3 (%d points) *** \n", numPoints );
  std::vector<Eigen::VectorXd> qPath;
  Eigen::VectorXd q;

  bool found;

  std::vector<Eigen::VectorXd> qSet;
  std::vector<int> heapSet;
  std::vector<double> valSet;

  std::vector<Eigen::VectorXd> qSet_1;
  std::vector<int> heapSet_1;
  std::vector<double> valSet_1;

  //.. Initialize
  q = _start;
  qPath.push_back( q );

  qSet.push_back( q );
  NSSet.push_back( qSet );

  heapSet.push_back(0);
  NSPriority.push_back( heapSet );

  valSet.push_back( JRM_Measure(q) );
  NSValues.push_back( valSet);

  bool check_1;
  bool check_2;

  for( size_t i = 1; i < numPoints; ++i ) {
    printf("--> Workspace path [%d] \n", i );
    if( GenerateNSSet( qPath[i-1], _WSPath[i], qSet, heapSet, valSet ) == false ) {
      
      //-- Backtrack (i-1)
      Heap_Pop( NSPriority[i-1], NSValues[i-1] );
      found = false;
      check_1 = false;
      check_2 = false;

      while( NSPriority[i-1].size() > 0 && found == false ) {
	check_1 = GenerateNSSet( NSSet[i-1][ NSPriority[i-1][0] ], _WSPath[i], qSet, heapSet, valSet );
	if( check_1 == true ) {
	  qPath[i-1] = NSSet[i-1][ NSPriority[i-1][0] ];
	  found = true;
	  break;
	}
	else {
	  Heap_Pop( NSPriority[i-1], NSValues[i-1] );
	}
      } // end while (i-1)

      if( check_1 == true ) {
	printf("-- [%d] Backtrack 1 made to element %d in NS Set \n", i, NSPriority[i-1][0] );
      }

      //-- Backtrack (i-2)
      else {
	printf("-- [%d] Backtrack 1 failed - Starting Backtrack 2 \n", i );

	Heap_Pop( NSPriority[i-2], NSValues[i-2] );
	//-- Reset (i-1)
	NSValues[i-1].clear();
	NSSet[i-1].clear();
	NSPriority[i-1].clear();

	printf("Priority size 2: %d \n", NSPriority[i-2].size() );
	int count = 0;
	while( NSPriority[i-2].size() > 0 && found == false ) {
	  printf("BT2:: %d \n", count );
	  check_2 = GenerateNSSet( NSSet[i-2][ NSPriority[i-2][0] ], _WSPath[i-1], qSet_1, heapSet_1, valSet_1 );
	  if( check_2 == true ) {
	    while( heapSet_1.size() > 0 && found == false ) {
	      check_1 = GenerateNSSet( qSet_1[ heapSet_1[0] ], _WSPath[i], qSet, heapSet, valSet );
	      if( check_1 == true ) {

		NSPriority[i-1] = heapSet_1;
		NSValues[i-1] = valSet_1;
		NSSet[i-1] = qSet_1;
		
		qPath[i-2] = NSSet[i-2][ NSPriority[i-2][0] ];
		qPath[i-1] = NSSet[i-1][ NSPriority[i-1][0] ];
		found = true;
		printf("Found with check 1 and 2! \n");
		break;
	      } // end if check 1
	      else {
		Heap_Pop( heapSet_1, valSet_1 );
	      }
	    } //-- end while [i-1]
	    
	    if( found == false ) {
	      Heap_Pop( NSPriority[i-2], NSValues[i-2] );
	    }
	  } // end if check_2
	  else {
	    Heap_Pop( NSPriority[i-2], NSValues[i-2] );
	  }

	  count++;
	} // while [i-2]

	if( found == true ) {
	  printf("[%d] Backtrack 2 made to element %d in NS Set \n", i, NSPriority[i-2][0], NSPriority[i-1][0] );
	}
	else {
	  printf("-- [%d] Backtrack 2 failed - Stopping \n", i );
	  return qPath;
	}

      } //-- end of backtrack 2
      
    } // end if -- only executes when no NS generated
    
    
    NSSet.push_back( qSet );
    NSPriority.push_back( heapSet );
    NSValues.push_back( valSet );
    int ab = NSSet.size() - 1;
    printf("[%d] NS Set size %d \n", ab, NSSet[ab].size() );
    q = NSSet[i][ NSPriority[i][0] ];
    qPath.push_back(q);
  }
  
  printf( "*** Track End - IK BT2 *** \n" );
  return qPath;
}


