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
#include <cfloat>
#include <cmath>

const float LJM2::LJM2_INF = FLT_MAX;
const float LJM2::LJM2_1 = 1.0;
const float LJM2::LJM2_SQRT2 = 1.414;
const float LJM2::LJM2_SQRT3 = 1.732;	

const float LJM2::sNominalValue = 1.0;
const float LJM2::sDefaultBrushDist = 1.0;
const float LJM2::sNodeNeighborRange = 1.8;

const int LJM2::sMaxIter = 500000;

const int LJM2::NX[26] = {-1,-1,1, 1,-1,-1, 1, 1,-1,-1,1, 1, 0,-1,0,1, 0,-1, 0, 1,-1,0,1, 0,0, 0};
const int LJM2::NY[26] = {-1, 1,1,-1,-1, 1, 1,-1,-1, 1,1,-1,-1, 0,1,0,-1, 0, 1, 0, 0,1,0,-1,0, 0};
const int LJM2::NZ[26] = { 1, 1,1, 1,-1,-1,-1,-1, 0, 0,0, 0, 1, 1,1,1,-1,-1,-1,-1, 0,0,0, 0,1,-1};



/**
 * @function LJM2
 * @brief Constructor
 */
LJM2::LJM2( double _worldSizeX, double _worldSizeY, double _worldSizeZ, double _originX, double _originY, double _originZ, double _resolution ) {

	printf("-------(...) Creating object LJM2 (...)-------\n");

	//-- Geometry initialization
	mWorldSizeX = _worldSizeX;
	mWorldSizeY = _worldSizeY;
	mWorldSizeZ = _worldSizeZ;
	mResolution = _resolution;

	mOriginX = _originX;
	mOriginY = _originY;
	mOriginZ = _originZ;

	mSizeX = (int) ((double) mWorldSizeX/mResolution);
	mSizeY = (int) ((double) mWorldSizeY/mResolution);
	mSizeZ = (int) ((double) mWorldSizeZ/mResolution);

	printf("-------(i) Size XYZ: %d %d %d \n", mSizeX, mSizeY, mSizeZ );
	mStride1 = mSizeY*mSizeZ;
	mStride2 = mSizeZ;

	mNumNodes = mSizeX*mSizeY*mSizeZ;

    printf("-------(i) Num nodes : %d \n", mNumNodes );

	mNodes = new Node3D[mNumNodes];
	mGeometricNeighbors = new std::vector<int>[mNumNodes];

	//-- Added for search in Skeleton
	mInSurface = new bool[mNumNodes];
	std::fill( mInSurface, mInSurface + mNumNodes, false );

	//-- Initialize count of paths to zero
	mCountPaths = 0;

	//-- Create nodes
	printf("-- Creating nodes for all the grid (%d %d %d) \n", mSizeX, mSizeY, mSizeZ);
	int index;
	for( size_t i = 0; i < mSizeX; ++i ) {
		for( size_t j = 0; j < mSizeY; ++j ) {
			for( size_t k = 0; k < mSizeZ; ++k ) {
				Node3D node;
				node.x = i; node.y = j; node.z = k;
				node.obsDist = LJM2_INF;
				node.state = FREE_STATE;
				index = ref(i,j,k);
				node.index = index;
				
				mNodes[index] = node;
			}
		}
	}
	printf("--> End of creation of nodes \n");
}
		

/**
 * @function ~LJM2
 * @brief Destructor
 */		
LJM2::~LJM2() {

	kd_free( mKdTree );

	if( mNodes != NULL ) {
		delete [] mNodes;
	}

	if( mGeometricNeighbors != NULL ) {
		delete [] mGeometricNeighbors;
	}

	if( mInSurface != NULL ) {
		delete [] mInSurface;
	}

	if( mSurfaceIndices != NULL ) {
		delete [] mSurfaceIndices;
	}

	if( mSurfaceNeighbors != NULL ) {
		delete [] mSurfaceNeighbors;
	}


	if( mHT != NULL ) {
		delete [] mHT;
	}

	if( mHTSurface != NULL ) {
		delete [] mHTSurface;
	}

}



/**
 * @function ProcessGeometry
 */
void LJM2::ProcessGeometry() {
    CalculateGeometricNeighbors();
	CalculateDT();
	//CalculateDTSurface();
}

/**
 * @function CalculateGeometricNeighbors
 */
void LJM2::CalculateGeometricNeighbors() {

	printf( "------ (...) Getting Geometric Neighbors: (...)------ \n" );

	for( size_t i = 0; i < mNumNodes; ++i ) {
		mGeometricNeighbors[i] = GetGeometricNeighbors(i);
	}
	printf("-------(i) Gotten all of Geometric Neighbors (i) ------- \n");


}

/////////////////// DT Functions ///////////////////////////

/**
 * @function CalculateDT
 */
void LJM2::CalculateDT() {


	printf("-------(...) Calculating DT (...) -------\n");
	std::vector<int> queue;

	//-- 1. Initialize queue with obstacle grids and set distances to zero for them
	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {
			for( int k = 0; k < mSizeZ; ++k ) {

				if( GetState( i, j, k ) == OBSTACLE_STATE ) {
					int ind = ref( i, j, k );
					queue.push_back( ind );
					mNodes[ ind ].obsDist = 0;
				}
			}
		}
	}

	//-- 2. Loop until no more new elements in Queue
	std::vector<int> newQueue(0);

	while( queue.size() > 0 ) {

		for( int i = 0; i < queue.size(); ++i ) {
			std::vector<int> n = mGeometricNeighbors[ queue[i] ];
			double dist = mNodes[ queue[i] ].obsDist;

			for( int j = 0; j < n.size(); ++j ) {
				double new_dist = dist + EdgeCost( queue[i], n[j], sNominalValue ); // standard distance
				if( new_dist < mNodes[ n[j] ].obsDist ) {
					mNodes[ n[j] ].obsDist = new_dist;
					newQueue.push_back( n[j] );
				}
			}
		}

		queue.clear();
		queue = newQueue;
		newQueue.clear();

	}
	printf("--> Finished calculating DT \n" );

}

/**
 * @function CalculateDTSurface
 */
void LJM2::CalculateDTSurface() {

	mHTSurface = new int[mNumNodes];
	std::fill( mHTSurface, mHTSurface + mNumNodes, -1 );

	std::vector<int> surfaceIndices;	
    
	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {
			for( int k = 0; k < mSizeZ; ++k ) {
				int x = ref( i, j, k );
				if( GetState( x ) == FREE_STATE && IsLocalMaxima( x ) == true ) {							
					surfaceIndices.push_back( x );
				}
			}		
		}
	}	

	mNumSurfaceNodes = surfaceIndices.size();
    printf("Num surface nodes : %d \n", mNumSurfaceNodes);
	mSurfaceIndices = new int[mNumSurfaceNodes];
    mSurfaceNeighbors = new std::vector<int>[mNumSurfaceNodes];
	mKdTree = kd_create( 3 ); 


	//-- Put everything inside the k-tree
	for( size_t i = 0; i < mNumSurfaceNodes; ++i ) {
		int x = surfaceIndices[i];
		mSurfaceIndices[i] = x;
        mHTSurface[x] = i; 
        mInSurface[x] = true;
		
		Eigen::VectorXd entity(3);
        entity << (double)mNodes[x].x, (double)mNodes[x].y, (double)mNodes[x].z; 
		kd_insert( mKdTree, entity.data(), (void*)x ); 
	}
	
}


/**
 * @function EdgeCost
 */
float LJM2::EdgeCost( int _index1, int _index2, float _value ) {

	Node3D n1 = mNodes[_index1];
	Node3D n2 = mNodes[_index2];

    int d = abs( n1.x - n2.x ) + abs( n1.y - n2.y ) + abs( n1.z - n2.z );

	if( d == 3 ) {
		return LJM2_SQRT3*_value; // sqrt(3)
	}

	else if( d == 2 ) {
		return LJM2_SQRT2*_value; // sqrt(2)
	}
	else if( d == 1 ) {
		return 1.0*_value; // 1
	}
	else {
		printf( "--> WTH. There is an error here! d: %d \n", d );
		return LJM2_INF;
	}
}

/**
 * @function IsLocalMaxima
 */
bool LJM2::IsLocalMaxima( int _index ) {

	Node3D node = mNodes[ _index ];
    float dist = node.obsDist;		
	std::vector<int> nx = GetGeometricNeighbors( _index );

	for( int i = 0; i < nx.size(); ++i ) {

		Node3D neigh = mNodes[ nx[i] ];

		if( EdgeCost( nx[i], _index ) == LJM2_1 && neigh.obsDist >= node.obsDist + LJM2_1  ) {
			return false;		
		}
		else if( EdgeCost( nx[i], _index ) == LJM2_SQRT2 && neigh.obsDist >= node.obsDist  + LJM2_SQRT2 ) { // LJM2_SQRT2 // 0.95		
			return false;
		}
		else if( EdgeCost( nx[i], _index ) == LJM2_SQRT3 && neigh.obsDist >= node.obsDist  + LJM2_SQRT3 ) {		
			return false;
		}
	}

	return true;

}

/**
 * @function GetGeometricNeighbors
 */
std::vector<int> LJM2::GetGeometricNeighbors( int _x, int _y, int _z ) const {
	return GetGeometricNeighbors( ref( _x, _y, _z ) );
}

/**
 * @function GetGeometricNeighbors
 */
std::vector<int> LJM2::GetGeometricNeighbors( int _ref ) const {
	std::vector<int> neighbors;	

	Node3D *node = &mNodes[_ref];
	int x = node->x;
	int y = node->y;
	int z = node->z;
	int nx; int ny; int nz;

	for( int i = 0; i < 26; ++i ) {
		nx = x + NX[i];
		ny = y + NY[i];
		nz = z + NZ[i];

		if( IsValid( nx, ny, nz ) == true && GetState( nx, ny, nz) == FREE_STATE ) {
			neighbors.push_back( ref(nx, ny, nz) );
		}
	}		
	return neighbors;
}

/**
 * @function GetSurfaceNeighbors
 */
std::vector<int> LJM2::GetSurfaceNeighbors( int _ref ) const {

		std::vector<int> surfaceNeighbors;

		Eigen::VectorXd entity(3); 

		Node3D *n = &mNodes[_ref];
		entity<< (double)n->x, (double)n->y, (double)n->z;

		struct kdres* neighset = kd_nearest_range( mKdTree, entity.data(), (double)sNodeNeighborRange );

		int size = kd_res_size( neighset );

		for( int j = 0; j < size; ++j ) {
		    uintptr_t near = (uintptr_t) kd_res_item_data( neighset ); 
			if( mNodes[_ref].index != near ) { 
				surfaceNeighbors.push_back( near );
			}
			kd_res_next( neighset );			
		}

		size = surfaceNeighbors.size();
		if( size == 0 || size > 26 ) {
			printf("--(!) Error filling neighbors. Either too few or too much \n");
		}

		//-- Free the structure
		kd_res_free( neighset );

	return surfaceNeighbors;
}


////////////////// Visualization Functions //////////////////

/**
 * @function ViewObstacles
 */
void LJM2::ViewObstacles( pcl::visualization::PCLVisualizer *_viewer,
				  	     int _r, int _g, int _b ) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud( new pcl::PointCloud<pcl::PointXYZ> );

	obstaclesCloud->height = 1;
	obstaclesCloud->is_dense = false;
	obstaclesCloud->points.resize( 0 );

	pcl::PointXYZ q;
		
	for( size_t i = 0; i < mSizeX; ++i ) {	
		for( size_t j = 0; j < mSizeY; ++j ) {
			for( size_t k = 0; k < mSizeZ; ++k ) {
				if( GetState( i, j, k ) == OBSTACLE_STATE ) { // Obstacle
					q.x = (double) i; q.y = (double) j; q.z = (double)k;				
					obstaclesCloud->points.push_back(q);       
				}
			}
		}
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> obstaclesColor( obstaclesCloud, _r, _g, _b );

	_viewer->addPointCloud<pcl::PointXYZ>( obstaclesCloud, obstaclesColor, "Obstacles Cloud" );
	_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Obstacles Cloud");
}

/**
 * @function ViewDTSurface
 */
void LJM2::ViewDTSurface( pcl::visualization::PCLVisualizer *_viewer,
	   	  		         int _r, int _g, int _b ) {


	pcl::PointCloud<pcl::PointXYZ>::Ptr DTSurfaceCloud( new pcl::PointCloud<pcl::PointXYZ> );

	DTSurfaceCloud->height = 1;
	DTSurfaceCloud->is_dense = false;
	DTSurfaceCloud->points.resize( 0 );
		
	for( size_t i = 0; i < mNumSurfaceNodes ; ++i ) {

		pcl::PointXYZ q;
		q.x = mNodes[ mSurfaceIndices[i] ].x; 
		q.y = mNodes[ mSurfaceIndices[i] ].y; 
		q.z = mNodes[ mSurfaceIndices[i] ].z;					
		DTSurfaceCloud->points.push_back(q);       
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> DTSurfaceColor( DTSurfaceCloud, _r, _g, _b );

	_viewer->addPointCloud<pcl::PointXYZ>( DTSurfaceCloud, DTSurfaceColor, "DT Surface Cloud" );

}

/**
 * @function ViewBlameDTPoints
 * @brief Visualize the points with DistBrush max (something wrong there)
 */
void LJM2::ViewBlameDTPoints( pcl::visualization::PCLVisualizer *_viewer,
	   	  		         int _r, int _g, int _b ) {


	pcl::PointCloud<pcl::PointXYZ>::Ptr BlameCloud( new pcl::PointCloud<pcl::PointXYZ> );

	BlameCloud->height = 1;
	BlameCloud->is_dense = false;
	BlameCloud->points.resize( 0 );
		
	int cont = 0;
	for( int i = 0; i < mNumNodes; ++i ) {
		pcl::PointXYZ q;
		if( mNodes[i].s.brushDist == LJM2_INF ) {
			q.x = mNodes[i].x; 
			q.y = mNodes[i].y; 
			q.z = mNodes[i].z;					
			BlameCloud->points.push_back(q);  
			cont++;
		}     
	}

	printf("--(i) Num Points to blame ( DistBrush = MAX ) = %d \n", cont );
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> BlameColor( BlameCloud, _r, _g, _b );

	_viewer->addPointCloud<pcl::PointXYZ>( BlameCloud, BlameColor, "Blame Cloud" );

}


/**
 * @function ViewPath
 */
void LJM2::ViewPath( std::vector<Eigen::Vector3i> _path, pcl::visualization::PCLVisualizer *_viewer,
int _r, int _g, int _b ) {

	printf( "Plotting a path of %d points \n", _path.size() );

	pcl::PointXYZ q;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pathCloud( new pcl::PointCloud<pcl::PointXYZ> );
	for( int j = 0; j < _path.size(); ++j ) {
		q.x = _path[j](0);
		q.y = _path[j](1);
		q.z = _path[j](2);
		pathCloud->points.push_back(q);
	}

	double r; double g; double b;
	r = ( _r % 256 )/255.0;
	g = ( _g % 256 )/255.0;
	b = ( _b % 256 )/255.0;

	for( int j = 0; j < pathCloud->points.size() - 1; ++j ) {
		char linename[15];
		sprintf( linename, "path%d-%d", mCountPaths, j );
		std::string id(linename);
		_viewer->addLine<pcl::PointXYZ>( pathCloud->points[j], pathCloud->points[j + 1], r, g, b, id );
		_viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id );
	}

	mCountPaths++;
}

/**
* @function ViewPaths
*/
void LJM2::ViewPaths( std::vector< std::vector<Eigen::Vector3i> > _paths, pcl::visualization::PCLVisualizer *_viewer ) {

	srand( time(NULL) );

	int numPaths = _paths.size();
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pathCloud;

	pcl::PointXYZ q;
	for( size_t i = 0; i < numPaths; ++i ) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr pc( new pcl::PointCloud<pcl::PointXYZ> );

		for( int j = 0; j < _paths[i].size(); ++j ) {
			q.x = _paths[i][j](0);
			q.y = _paths[i][j](1);
			q.z = _paths[i][j](2);
			pc->points.push_back(q);
		}
		pathCloud.push_back( pc );
	}


	for( size_t i = 0; i < numPaths; ++i ) {
		double r; double g; double b;
		r = ( rand() % 256 )/255.0;
		g = ( rand() % 256 )/255.0;
		b = ( rand() % 256 )/255.0;

		for( int j = 0; j < pathCloud[i]->points.size() - 1; ++j ) {
			char linename[16];
			sprintf( linename, "paths%d-%d", i,j );
			std::string id(linename);
			_viewer->addLine<pcl::PointXYZ>( pathCloud[i]->points[j], pathCloud[i]->points[j + 1], r, g, b, id );
			_viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, id );
		}
	}
}

/**
 * @function ViewBall
 * @brief Draw a ball in the location specified
 */
void LJM2::ViewBall( pcl::visualization::PCLVisualizer *_viewer, int _x, int _y, int _z, std::string _name, double _radius, double _r, double _g, double _b ) {

	pcl::PointXYZ pos;
    pos.x = _x;
    pos.y = _y;
    pos.z = _z;
	_viewer->addSphere ( pos, _radius, _r, _g, _b, _name );
    _viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, _name );
}


///////////// Geometry Helper Functions ////////////////////

/**
 * @function CreateExternalBoundary
 * @brief Create a wall around the available space
 */
void LJM2::CreateExternalBoundary() {

	//-- Add the faces
	for( size_t j = 0; j < mSizeY; ++j ) {
		for( size_t k = 0; k < mSizeZ; ++k ) {
			SetState( 0, j, k, OBSTACLE_STATE );			// Face 2
			SetState( mSizeX - 1, j, k, OBSTACLE_STATE );	// Face 4
		}	
	}

	for( size_t i = 0; i < mSizeX; ++i ) {
		for( size_t k = 0; k < mSizeZ; ++k ) {
			SetState( i, 0, k, OBSTACLE_STATE );			// Face 1
			SetState( i, mSizeY - 1, k, OBSTACLE_STATE );	// Face 3
		}	
	}

	for( size_t i = 0; i < mSizeX; ++i ) {
		for( size_t j = 0; j < mSizeY; ++j ) {
			SetState( i, j, 0, OBSTACLE_STATE );			// Face 6
			SetState( i, j, mSizeZ - 1, OBSTACLE_STATE );	// Face 5
		}	
	}
}

/**
 * @function CreateBox
 */
void LJM2::CreateBox( int _x, int _y, int _z, int _dimX, int _dimY, int _dimZ ) {

	for( size_t i = _x; i < _x + _dimX; ++i ) {
		for( size_t j = _y; j < _y + _dimY; ++j ) {
			for( size_t k = _z; k < _z + _dimZ; ++k ) {
				SetState( i, j, k, OBSTACLE_STATE );
			}
		}
	}
}


/////////////////////// GRID FUNCTIONS ///////////////////////////////////

/**
 * @function IsValid
 */
bool LJM2::IsValid( int _x, int _y, int _z ) const {
	if( _x < 0 || _x >= mSizeX || _y < 0 || _y >= mSizeY || _z < 0 || _z >= mSizeZ ) {
		return false;
	}
	return true;
}

/**
 * @function SetState
 */
void LJM2::SetState( int _x, int _y, int _z, int _state ) {

	mNodes[ref(_x, _y, _z)].state = _state;
}

/**
 * @function GetState
 */
int LJM2::GetState( int _x, int _y, int _z ) const{
	return mNodes[ref(_x, _y, _z)].state;
}

/**
 * @function GetState
 */
int LJM2::GetState( int _ref ) const{
	return mNodes[_ref].state;
}

/**
 * @function CheckCollision
 */
bool LJM2::CheckCollision( int _x, int _y, int _z ) {
	if( GetState(_x, _y, _z) == FREE_STATE ) {
		return false; 
	} else {
		return true;
	}
}


