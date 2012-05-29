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
 * @function FindVarietyPaths1
 * @brief Using Surfaces
 */
std::vector< std::vector<Eigen::Vector3i> > LJM2::FindVarietyPaths1( int _x1, int _y1, int _z1,
									   			   	  		        int _x2, int _y2, int _z2, 
																    int _times, float _alpha ) {

	mAlpha = _alpha;
	std::vector< std::vector<Eigen::Vector3i> > paths;
    std::vector<Eigen::Vector3i> path;
	std::vector< std::vector<int> > nodePaths;
	std::vector<int> allNodePaths;


	//-- Find the nearest points
	int startIndex = ref( _x1, _y1, _z1 );
	int targetIndex = ref( _x2, _y2, _z2 );
	int startSurfaceIndex; 
	int targetSurfaceIndex;

	//-- Locate nearest points in Surface
	struct kdres* result;

	Eigen::VectorXd eStart(3);
	eStart << (double)_x1, (double)_y1, (double)_z1; 

    result = kd_nearest( mKdTree, eStart.data() );
	startSurfaceIndex = (int) kd_res_item_data(result);

	Eigen::VectorXd eTarget(3);
	eTarget << (double)_x2, (double)_y2, (double)_z2; 

    result = kd_nearest( mKdTree, eTarget.data() );
	targetSurfaceIndex = (int) kd_res_item_data(result);

	kd_res_free( result );


	printf("--> Start: %d (%d %d %d) Target: %d (%d %d %d)\n", startIndex, mNodes[startIndex].x, mNodes[startIndex].y, mNodes[startIndex].z, targetIndex, mNodes[targetIndex].x, mNodes[targetIndex].y, mNodes[targetIndex].z );	
	printf(" Start Surface: %d (%d %d %d) Target Surface: %d (%d %d %d) \n", startSurfaceIndex, mNodes[startSurfaceIndex].x, mNodes[startSurfaceIndex].y, mNodes[startSurfaceIndex].z, targetSurfaceIndex, mNodes[targetSurfaceIndex].x, mNodes[targetSurfaceIndex].y, mNodes[targetSurfaceIndex].z );

	printf("Start surface on Surface: %d \n", mInSurface[startSurfaceIndex] );
	printf("Target surface on Surface: %d \n", mInSurface[targetSurfaceIndex] );

	//-- Now find short paths connecting the ending nodes with Surface
	InitSearch();
	std::vector<Eigen::Vector3i> startSegment = ShortestPath( startIndex, startSurfaceIndex );
	ResetSearch();
	std::vector<Eigen::Vector3i> targetSegment = ShortestPath( targetIndex, targetSurfaceIndex );

    paths.push_back( startSegment );
	paths.push_back( targetSegment );


	//-- Now search in the Surface
	ResetSearch();

	for( size_t i = 0; i < _times; ++i ) {

		path.resize(0);
		path = FindSurfacePath( startSurfaceIndex, targetSurfaceIndex );
		paths.push_back( path );
		nodePaths.push_back( mPath );

		//-- Update the values
		ResetSearch();
		allNodePaths = JoinPaths( nodePaths );
		UpdateSurfaceNodeValues( allNodePaths );
	}

	return paths;
}



/**
 * @function ShortestSurfacePath
 * @brief This is in the whole free space
 */
std::vector<Eigen::Vector3i> LJM2::ShortestSurfacePath( int _n1, int _n2 ) {

	std::vector<Eigen::Vector3i> path;
	printf( "--> Start Shortest Surface Path search \n" );

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
		std::vector<int> neighbors = GetSurfaceNeighbors(x);

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
 * @function FindSurfacePath
 */
std::vector<Eigen::Vector3i> LJM2::FindSurfacePath( int _n1, int _n2 ) {

	std::vector<Eigen::Vector3i> path;
	printf( "Start Surface Search \n" );

	int nodeStart = _n1;
	int nodeTarget = _n2;

	//-- Fill start space
	Node3D *node;
	node = &mNodes[nodeStart];
	node->s.costG = 0;
	node->s.costH = CostHeuristic( nodeStart, nodeTarget )*( 0 + mAlpha );
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
			printf( "--> Found a path! \n" ); TracePath( x, path );break;
		}
		
		//-- Add node to closed set
		mNodes[x].s.status = IN_CLOSED_SET;
		std::vector<int> neighbors = GetSurfaceNeighbors(x);

		//-- Send node to closed list
		for( int i = 0; i < neighbors.size(); i++ ) {
			if( mNodes[ neighbors[i] ].s.status == IN_CLOSED_SET ) {
				continue;	
			}
			
			int y = mNodes[ neighbors[i] ].index; // Same as neighbors[i] actually
			float tentative_G_score = mNodes[x].s.costG + EdgeCost( x, y, ( mNodes[y].s.value + mAlpha) );
			
			if( mNodes[y].s.status != IN_OPEN_SET ) {
				node = &mNodes[y];
				node->s.parent = x;
				node->s.costG = tentative_G_score;
				node->s.costH = CostHeuristic( y, nodeTarget )*( 0 + mAlpha );
				node->s.costF = node->s.costG + node->s.costH;
				PushOpenSet(y);
			}
			else {
				if( tentative_G_score < mNodes[y].s.costG ) {
					node = &mNodes[y];
					node->s.parent = x;
					node->s.costG = tentative_G_score;
					node->s.costH = CostHeuristic( y, nodeTarget )*( 0 + mAlpha );
					node->s.costF = node->s.costG + node->s.costH;
					//-- Reorder your OpenSet
					UpdateLowerOpenSet( y );				
				}
			}
		} //-- End for every neighbor

		
	} //-- End of while
	node = NULL;
	printf("-- Surface Search ended \n");
	return path;
}


/**
 * @function CalculateDistanceFromPathSet
 */
void LJM2::CalculateSurfaceDistanceFromPathSet( std::vector<int> _path ) {

	printf("--> Calculating Surface Distance From Path Set \n");
	std::vector<int> queue = _path;

	//-- 1. Initialize queue with _path nodes and set distances to zero for them
	for( int i = 0; i < mNumSurfaceNodes; ++i ) {
		mNodes[mSurfaceIndices[i]].s.brushDist = LJM2_INF;
	}

	// Path nodes with zero initial value
	for( int i = 0; i < _path.size(); ++i ) {
		mNodes[ _path[i] ].s.brushDist = 0;
	}
	

	//-- 2. Loop until no more new elements in Queue
	std::vector<int> newQueue(0);

	while( queue.size() > 0 ) {

		for( int i = 0; i < queue.size(); ++i ) {
			std::vector<int> n = GetSurfaceNeighbors( queue[i] );
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
	printf("--) Finished calculating Surface DT Paths \n");
}


/**
 * @function UpdateSurfaceNodeValues
 */
void LJM2::UpdateSurfaceNodeValues( std::vector<int> _path ) {

	CalculateSurfaceDistanceFromPathSet(_path );	

	//-- Our DT is ready. Let's check the minimum and maximum guys
	float minBrushDist = LJM2_INF;
	float maxBrushDist = -LJM2_INF;


	int countInf = 0;
	std::vector<Eigen::Vector3i> infPath;

	for( int i = 0; i < mNumSurfaceNodes; ++i ) {
		float dist = mNodes[ mSurfaceIndices[i] ].s.brushDist;


		if( dist == LJM2_INF ) {
			countInf++;
			Eigen::Vector3i p; p << mNodes[ mSurfaceIndices[i] ].x, mNodes[ mSurfaceIndices[i] ].y, mNodes[ mSurfaceIndices[i] ].z;
			std::vector<int> a; a = GetSurfaceNeighbors(mSurfaceIndices[i]);
			printf("Surface Neighbors INf : %d \n", a.size()); 
			infPath.push_back( p );	
			mNodes[ mSurfaceIndices[i] ].s.brushDist = 0;
		}

		else {

			if( dist < minBrushDist ) {
				minBrushDist = dist;
			}

			if( dist > maxBrushDist ) {
				maxBrushDist = dist;
			}

		}
	}

	mMaxBrushDist = maxBrushDist;

	printf("--(i) Min and max: %f , %f count inf: %d \n", minBrushDist, maxBrushDist, countInf );


	//-- Update accordingly
	for( int i = 0; i < mNumSurfaceNodes; ++i ) {
		int ind = mSurfaceIndices[i];
        //mNodes[ ind ].s.value = ( mMaxBrushDist - mNodes[ ind ].s.brushDist )/mMaxBrushDist;
		mNodes[ ind ].s.value = ( mMaxBrushDist -mNodes[ ind ].s.brushDist );
	}

}


