/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved	
 * Author(s): Ana C. Huaman Quispe <ahuaman3@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "LJM2.h"

/////////////////// Search Functions ////////////////////////


/**
 * @function FindVarietyPaths2
 * @brief Using Free Space
 */
std::vector< std::vector<Eigen::Vector3i> > LJM2::FindVarietyPaths2( int _x1, int _y1, int _z1,
									   			   	  		        int _x2, int _y2, int _z2, 
																    int _times, float _alpha ) {

	mAlpha = _alpha;
	std::vector< std::vector<Eigen::Vector3i> > paths;
    std::vector<Eigen::Vector3i> path;
	std::vector< std::vector<int> > nodePaths;
	std::vector<int> allNodePaths;

	time_t ts; time_t tf; double dt; 
	//-- Find the nearest points
	int startIndex = ref( _x1, _y1, _z1 );
	int targetIndex = ref( _x2, _y2, _z2 );

	printf("--> Start: %d Target: %d \n", startIndex, targetIndex );	

	InitSearch();

	for( size_t i = 0; i < _times; ++i ) {

		path.resize(0);
        //ts = clock();
		path = FindPath( startIndex, targetIndex );
        //tf = clock();
        printf("--[%d] Found Path of size %d \n", i, path.size() ); 
        //printf(" Time elapsed: %.3f  \n", (double) (tf - ts)/CLOCKS_PER_SEC );
		paths.push_back( path );
		nodePaths.push_back( mPath );

		//-- Update the values
        //ts = clock();
		ResetSearch();
        //tf = clock();
        //printf("--[%d] Search : Time elapsed: %.3f  \n", i, (double) (tf - ts)/CLOCKS_PER_SEC );
		allNodePaths = JoinPaths( nodePaths );
        ts = clock();
		UpdateNodeValues( allNodePaths ); 
        //tf = clock();
        //printf("--[%d] UpdateNodes : Time elapsed: %.3f  \n", i, (double) (tf - ts)/CLOCKS_PER_SEC );
	}

	return paths;
}


/**
 * @function NodePathToWorkspacePath
 */
std::vector< std::vector<Eigen::VectorXd> > LJM2::NodePathToWorkspacePath( std::vector< std::vector<Eigen::Vector3i> > _nodePath ) {
	
	std::vector< std::vector<Eigen::VectorXd> > workspacePath;

	for( size_t i = 0; i < _nodePath.size(); ++i ) {
		std::vector<Eigen::VectorXd> path;
		for( size_t j = 0; j < _nodePath[i].size(); ++j ) {
			Eigen::VectorXd temp(3);
			GridToWorld( _nodePath[i][j](0), _nodePath[i][j](1), _nodePath[i][j](2), temp(0), temp(1), temp(2) );
			path.push_back( temp );			
		}
		workspacePath.push_back( path );
	}	

	return workspacePath;
}

/**
 * @function ShortestPath
 * @brief This is in the whole free space
 */
std::vector<Eigen::Vector3i> LJM2::ShortestPath( int _n1, int _n2 ) {

	std::vector<Eigen::Vector3i> path;
	printf( "--> Start Shortest Path search \n" );


	int nodeStart = _n1;
	int nodeTarget = _n2;


	//-- Fill start space
	Node3D *node;
	node = &mNodes[nodeStart];
	node->s.costG = 0;
	node->s.costH = CostHeuristic( nodeStart, nodeTarget )*sNominalValue;
	node->s.costF = node->s.costG + node->s.costH;
	node->s.parent = -1;
	node->s.status = IN_NO_SET;

	//-- Push it into the open set
	PushOpenSet( nodeStart );

	//-- Loop
	int x;
	int count = 0;

	while( count < sMaxIter ) {
		count++;
		//-- Remove top node in OpenSet
		try {
			x = PopOpenSet();
		} catch( int i ) {
			printf( "-- (%d) No more nodes to pop out \n", i );
			break;
		}

		//-- Check if it is goal
		if( x == nodeTarget ) {
			printf( "--> Found a path! \n" ); TracePath( x, path ); break;
		}
		
		//-- Add node to closed set
		mNodes[x].s.status = IN_CLOSED_SET;
		std::vector<int> neighbors = mGeometricNeighbors[x];

		//-- 
		for( int i = 0; i < neighbors.size(); i++ ) {
			if( mNodes[ neighbors[i] ].s.status == IN_CLOSED_SET ) {
				continue;	
			}
			
			int y = mNodes[ neighbors[i] ].index; // Same as neighbors[i] actually
			float tentative_G_score = mNodes[x].s.costG + EdgeCost( x, y, sNominalValue );
			
			if( mNodes[y].s.status != IN_OPEN_SET ) {
				node = &mNodes[y];
				node->s.parent = x;
				node->s.costG = tentative_G_score;
				node->s.costH = CostHeuristic( y, nodeTarget )*sNominalValue;
				node->s.costF = node->s.costG + node->s.costH;
				PushOpenSet(y);
			}
			else {
				if( tentative_G_score < mNodes[y].s.costG ) {
					node = &mNodes[y];
					node->s.parent = x;
					node->s.costG = tentative_G_score;
					node->s.costH = CostHeuristic( y, nodeTarget )*sNominalValue;
					node->s.costF = node->s.costG + node->s.costH;
					//-- Reorder your OpenSet
					UpdateLowerOpenSet( y );				
				}
			}
		} //-- End for every neighbor

		
	} //-- End of while
	node = NULL;
	printf("Finished finding shortest path \n");
	return path;
}

/**
 * @function FindPath
 * @brief This is in the whole free space
 */
std::vector<Eigen::Vector3i> LJM2::FindPath( int _n1, int _n2 ) {

	std::vector<Eigen::Vector3i> path;
	printf( "--> Start FindPath in whole free space \n" );

	int nodeStart = _n1;
	int nodeTarget = _n2;


	//-- Fill start space
	Node3D *node;
	node = &mNodes[nodeStart];
	node->s.costG = 0;
	node->s.costH = CostHeuristic( nodeStart, nodeTarget )*( 0 + mAlpha*1 );
	node->s.costF = node->s.costG + node->s.costH;
	node->s.parent = -1;
	node->s.status = IN_NO_SET;

	//-- Push it into the open set
	PushOpenSet( nodeStart );

	//-- Loop
	int x;
	int count = 0;

	while( count < sMaxIter ) {
		count++;
		//-- Remove top node in OpenSet
		try {
			x = PopOpenSet();
		} catch( int i ) {
			printf( "-- (%d) No more nodes to pop out \n", i );
			break;
		}

		//-- Check if it is goal
		if( x == nodeTarget ) {
			printf( "--> Found a path! : iters: %d  Cost: %.3f  \n", count, mNodes[x].s.costF ); TracePath( x, path ); break;
		}
		
		//-- Add node to closed set
		mNodes[x].s.status = IN_CLOSED_SET;
		std::vector<int> neighbors = mGeometricNeighbors[x];

		//-- 
		for( int i = 0; i < neighbors.size(); i++ ) {
			if( mNodes[ neighbors[i] ].s.status == IN_CLOSED_SET ) {
				continue;	
			}
			
			int y = mNodes[ neighbors[i] ].index; // Same as neighbors[i] actually
			float tentative_G_score = mNodes[x].s.costG + EdgeCost( x, y, sNominalValue )*( mNodes[y].s.value + mAlpha*1 );
			
			if( mNodes[y].s.status != IN_OPEN_SET ) {
				node = &mNodes[y];
				node->s.parent = x;
				node->s.costG = tentative_G_score;
				node->s.costH = CostHeuristic( y, nodeTarget )*( 0 + mAlpha*1 );
				node->s.costF = node->s.costG + node->s.costH;
				PushOpenSet(y);
			}
			else {
				if( tentative_G_score < mNodes[y].s.costG ) {
					node = &mNodes[y];
					node->s.parent = x;
					node->s.costG = tentative_G_score;
					node->s.costH = CostHeuristic( y, nodeTarget )*( 0 + mAlpha*1 );
					node->s.costF = node->s.costG + node->s.costH;
					//-- Reorder your OpenSet
					UpdateLowerOpenSet( y );				
				}
			}
		} //-- End for every neighbor

		
	} //-- End of while
	node = NULL;
	printf("Finished FindPath whole free space \n");
	return path;
}




/**
 * @function CalculateDistanceFromPathSet
 */
void LJM2::CalculateDistanceFromPathSet( std::vector<int> _path ) {

	printf("--> Calculating Distance From Path Set \n");
	std::vector<int> queue = _path;

	//-- 1. Initialize all to infinity
	for( int i = 0; i < mNumNodes; ++i ) {
		
		if( GetState( i ) == OBSTACLE_STATE || GetState( i ) == INFLATED_STATE ) {
			mNodes[i].s.brushDist = 0;
		}

		else {
			mNodes[i].s.brushDist = LJM2_INF;
		}
	}


	// Path nodes with zero initial value
	for( int i = 0; i < _path.size(); ++i ) {
		mNodes[ _path[i] ].s.brushDist = 0;
	}
	

	//-- 2. Loop until no more new elements in Queue
	std::vector<int> newQueue(0);

	while( queue.size() > 0 ) {

		for( int i = 0; i < queue.size(); ++i ) {
			std::vector<int> n = mGeometricNeighbors[ queue[i] ];
			double dist = mNodes[ queue[i] ].s.brushDist;

			for( int j = 0; j < n.size(); ++j ) {
				double new_dist = dist + EdgeCost( queue[i], n[j], sNominalValue );
				if( new_dist < mNodes[ n[j] ].s.brushDist ) {
					mNodes[ n[j] ].s.brushDist = new_dist;
					newQueue.push_back( n[j] );
				}
			}
		}

		queue.clear();
		queue = newQueue;
		newQueue.clear();

	}
	printf("--) Finished calculating DT Paths \n");
}


/**
 * @function UpdateNodeValues
 */
void LJM2::UpdateNodeValues( std::vector<int> _path ) {

	CalculateDistanceFromPathSet(_path );	

	//-- Our DT is ready. Let's check the minimum and maximum guys
	float minBrushDist = LJM2_INF;
	float maxBrushDist = -LJM2_INF;

	for( int i = 0; i < mNumNodes; ++i ) {
		if( mNodes[i].s.brushDist < minBrushDist ) {
			minBrushDist = mNodes[ i ].s.brushDist;
		}

		if( mNodes[ i ].s.brushDist > maxBrushDist ) {
			maxBrushDist = mNodes[ i ].s.brushDist;
		}
		
	}

	mMaxBrushDist = maxBrushDist;

	printf("--(i) Min and max: %f , %f \n", minBrushDist, maxBrushDist );


	//-- Update accordingly
	for( int i = 0; i < mNumNodes; ++i ) {
		mNodes[i].s.value = ( maxBrushDist - mNodes[i].s.brushDist );
        //mNodes[ i ].s.value = ( maxBrushDist - mNodes[ i ].s.brushDist )/mMaxBrushDist;
	}

}


/**
 * @function JoinPaths
 * @brief FIXME CUT OFF REPEATED GUYS	
 */
std::vector<int> LJM2::JoinPaths( std::vector< std::vector<int> >  _allPaths ) {

	std::vector<int> bunchPaths;

	for( int i = 0; i < _allPaths.size(); ++i ) {
		for( int j = 0; j < _allPaths[i].size(); ++j ) {
			bunchPaths.push_back( _allPaths[i][j] );			
		}
	}

	return bunchPaths;
}


/**
 * @function InitSearch
 */
void LJM2::InitSearch() {

	printf("Start Init Search \n");

	//-- Reset the Hash Table for OpenSet and OpenSet
	mHT = new int[mNumNodes];
	std::fill( mHT, mHT + mNumNodes, -1 );

    ResetSearch();
	printf("Finishing Init Search \n");
}

/**
 * @function ResetSearch 
 */
void LJM2::ResetSearch() {

	printf("-- Start Reset  Search \n");
	Node3D *node;
	for( int i = 0; i < mNumNodes; ++i ) {

		node = &mNodes[i];
		node->s.costF = LJM2_INF;
		node->s.costG = LJM2_INF;
		node->s.costH = LJM2_INF; 
		node->s.status = IN_NO_SET;
		node->s.value = 0; 
		node->s.parent = -1;
		node->s.brushDist = LJM2_INF;
	}

	mOpenSet.resize(0);
	printf("-- End Reset Search \n");
}

/**
 * @function TracePath
 * @brief Trace path, literally
 */
bool LJM2::TracePath( const int &_key, std::vector<Eigen::Vector3i> & _path  ) {

  printf( "--> Start Trace path \n" );

  std::vector<int> backPath(0);

  mPath.resize(0);
  _path.resize(0);

  int n = _key;

  while( n != -1 ) 
  {
    backPath.push_back( n );
    n = mNodes[n].s.parent;
  }

  int b = backPath.size();

  Node3D *node;
  for( int i = 0; i < b; i++ )
     { 
	   node = &mNodes[ backPath[ b - 1- i] ];
	   Eigen::Vector3i p; p << node->x, node->y, node->z; 
	   _path.push_back( p );
	   mPath.push_back( backPath[b-1-i] );
	 }
    node = NULL;
  if( _path.size() > 0 )
    { return true; }

  return false;  
}  


/**
 * @function CostHeuristic
 */
float LJM2::CostHeuristic( int _start, int _target ) {

	Node3D nStart= mNodes[_start];
	Node3D nTarget = mNodes[_target];

	float dx = nStart.x - nTarget.x;
	float dy = nStart.y - nTarget.y;
	float dz = nStart.z - nTarget.z;

	return sqrt( dx*dx + dy*dy + dz*dz );
}

/**
 * @function PushOpenSet
 */
void LJM2::PushOpenSet( int _key ) {

	int n; 
    int node; int parent;
    int temp;

    //-- Sign the flag
    mNodes[_key].s.status = IN_OPEN_SET;

    mOpenSet.push_back( _key );
    n = mOpenSet.size() - 1;
 
    // If this is the first element added
    if( n == 0 ) { 
  	    mHT[_key] = n;
	    return; 
	}

    // If not, start on the bottom and go up
    node = n;

    int qp; int qn;

    while( node != 0 )
    {
    	parent = floor( (node - 1)/2 );
		qp = mOpenSet[parent];
    	qn = mOpenSet[node];
    	// Always try to put new nodes up
    	if( mNodes[qp].s.costF >= mNodes[qn].s.costF )
      	{
        	temp = mOpenSet[parent];
        	mOpenSet[parent] = qn; mHT[qn] = parent; 
        	mOpenSet[node] = temp; mHT[temp] = node; 
        	node = parent; 
      	}  
    	else
     	{ break; }
    }   

 	mHT[_key] = node;

}

/**
 * @function PopOpenSet
 */
int LJM2::PopOpenSet() { 

  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  int temp;

  if( mOpenSet.size() == 0 )
    { throw -1; }

  // Save the pop-out element
  first = mOpenSet[0];
  
  // Reorder your binary heap
  bottom = mOpenSet.size() - 1;

  mOpenSet[0] = mOpenSet[bottom]; mHT[ mOpenSet[bottom] ] = 0;
  mOpenSet.pop_back();
  n = mOpenSet.size();

  int u = 0;

  int qu;
  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( mNodes[ mOpenSet[node] ].s.costF >= mNodes[ mOpenSet[child_1] ].s.costF )
        { u = child_1;  }
       if( mNodes[ mOpenSet[u] ].s.costF  >= mNodes[ mOpenSet[child_2] ].s.costF )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( mNodes[ mOpenSet[node] ].s.costF >= mNodes[ mOpenSet[child_1] ].s.costF )
         { u = child_1; }
     }
    
	qu = mOpenSet[u];
    if( node != u )
     { temp = mOpenSet[node];
       mOpenSet[node] = qu; mHT[qu] = node;
       mOpenSet[u] = temp; mHT[temp] = u; 
     }

    else
     { break; } 
  }

  return first;

}

/**
 * @function UpdateLowerOpenSet
 * @brief Update a node with a lower value
 */
void LJM2::UpdateLowerOpenSet( int _key ) { 
  int n; 
  int node; int parent;
  int temp;

  //-- Find your guy
  n = mHT[_key];

  //-- If it happens to be the first element
  if( n == 0 )
    { return; } //-- mHT[ID] = 0; // the same

  //-- If not, start on the bottom and go up
  node = n;

  int qp; int qn;

  while( node != 0 )
  { 
    parent = floor( (node - 1)/2 );

	qp = mOpenSet[parent]; qn = mOpenSet[node];
    // Always try to put new nodes up
    if( mNodes[ qp ].s.costF > mNodes[ qn ].s.costF )
      {
        temp = mOpenSet[parent];
        mOpenSet[parent] = qn; mHT[qn] = parent;
        mOpenSet[node] = temp; mHT[temp] = node; 
        node = parent; 
      }  
    else
     {  break; }
  }   

}
