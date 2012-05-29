/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved	
 * Author(s): Ana C. Huaman Quispe <ahuaman3@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "LJM2D-style" License:
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
 *   LIMITED TO, PROCUREMENT OF SULJM2TITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _LJM22_PLANNER_
#define _LJM22_PLANNER_

//-- PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <vector>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <time.h>

//-- Kd-tree stuff.
#include "kdtree/kdtree.h" 

/** State Enum */
enum {
	FREE_STATE = 1,
	OBSTACLE_STATE = 2,
    INFLATED_STATE = 3
};

/** Status Enum */
enum {
	IN_CLOSED_SET = 2,
	IN_OPEN_SET = 3,
	IN_NO_SET = 4
};

/**
 * @struct SearchData
 */
struct SearchData {
	float costF;
	float costG;
	float costH;
	
	float value;
	float brushDist;
	int status;
	int parent;
	
};

/**
 * @struct Node
 */
struct Node3D {
	int x; 
	int y; 
	int z;
	float obsDist;
	int index;
	int state;

	SearchData s;
};


/**
 * @class Workspace
 */
class LJM2 {

	public:
		//-- Constructor
		LJM2( double _worldSizeX, double worldSizeY, double _worldSizeZ, double _originX, double _originY, double _originZ, double _resolution );
		//-- Destructor
		~LJM2();
		
		//-- Processing functions
		void ProcessGeometry();

		//-- DT Functions
		void CalculateGeometricNeighbors();
		void CalculateDT();
		void CalculateDTSurface();
		bool IsLocalMaxima( int _index );

		float EdgeCost( int _index1, int _index2, float _value = sNominalValue );
		std::vector<int> GetGeometricNeighbors( int _x, int _y, int _z ) const;
		std::vector<int> GetGeometricNeighbors( int _ref ) const;
		std::vector<int> GetSurfaceNeighbors( int _ref ) const;

		//-- Search especific functions
		
		std::vector< std::vector<Eigen::Vector3i> > FindVarietyPaths1( int _x1, int _y1, int _z1,
										   				  		       int _x2, int _y2, int _z2, int _times, float _alpha = 1.0 );

		std::vector< std::vector<Eigen::Vector3i> > FindVarietyPaths2( int _x1, int _y1, int _z1,
										   				  		       int _x2, int _y2, int _z2, int _times, float _alpha = 1.0 );
		std::vector<Eigen::Vector3i> ShortestPath( int _n1, int _n2 ); 
		std::vector<Eigen::Vector3i> ShortestSurfacePath( int _n1, int _n2 ); 
		std::vector<Eigen::Vector3i> FindPath( int _n1, int _n2 );
		std::vector<Eigen::Vector3i> FindSurfacePath( int _n1, int _n2 );

		void CalculateSurfaceDistanceFromPathSet( std::vector<int> _path ); 
		void CalculateDistanceFromPathSet( std::vector<int> _path ); 
	    void UpdateNodeValues( std::vector<int> _path );
	    void UpdateSurfaceNodeValues( std::vector<int> _path );
		std::vector<int> JoinPaths( std::vector< std::vector<int> >  _allPaths );

		void InitSearch( );

    	void ResetSearch();
		bool TracePath( const int &key, std::vector<Eigen::Vector3i> & _path );
		float CostHeuristic( int _start, int _target  );

		void PushOpenSet( int _key );
		int PopOpenSet();
		void UpdateLowerOpenSet( int key );

		//-- Visualization Functions
		void ViewObstacles( pcl::visualization::PCLVisualizer *_viewer,
							int _r = 255, int _g = 0, int _b = 0 );

		void ViewDTSurface( pcl::visualization::PCLVisualizer *_viewer,
					        int _r = 255, int _g = 0, int _b = 0 );

		void ViewBlameDTPoints( pcl::visualization::PCLVisualizer *_viewer,
	   	 		    		    int _r, int _g, int _b );

		void ViewPath( std::vector<Eigen::Vector3i> _path, pcl::visualization::PCLVisualizer *_viewer,
					   int _r = 255, int _g = 255, int _b = 0 );
		void ViewPaths( std::vector< std::vector<Eigen::Vector3i> > _paths, pcl::visualization::PCLVisualizer *_viewer );

		void ViewBall( pcl::visualization::PCLVisualizer *_viewer, 
					   int _x, int _y, int _z, std::string _name, 
				       double _radius = 1.0, double _r = 1.0, double _g = 0.0, double _b = 0.0 );

		//-- Geometry Helper  functions
		void CreateExternalBoundary();
		void CreateBox( int _x, int _y, int _z, int _dimX, int _dimY, int _dimZ );

		//-- Grid functions
		int GetSizeX() const;
		int GetSizeY() const;
		int GetSizeZ() const;
		int ref( int _x, int _y, int _z ) const;

		bool IsValid( int _x, int _y, int _z ) const;
		void SetState( int _x, int _y, int _z, int _state );
		int GetState( int _x, int _y, int _z ) const;
		int GetState( int _ref ) const;
		bool CheckCollision( int _x, int _y, int _z );
		bool WorldToGrid( double _worldX, double _worldY, double _worldZ, int& _x, int& _y, int& _z ) const;
		bool GridToWorld(int _x, int _y, int _z, double& _worldX, double& _worldY, double& _worldZ ) const;
		std::vector< std::vector<Eigen::VectorXd> > NodePathToWorkspacePath( std::vector< std::vector<Eigen::Vector3i> > _nodePath );

		//-- Stuff!
		static const float LJM2_INF;
		static const float LJM2_1;
		static const float LJM2_SQRT2;
		static const float LJM2_SQRT3;	

		static const float sNominalValue;
		static const float sDefaultBrushDist;
		static const float sNodeNeighborRange;

		static const int sMaxIter; 

		static const int NX[];
		static const int NY[];
		static const int NZ[];


	private:
		double mWorldSizeX;
		double mWorldSizeY;
		double mWorldSizeZ;
		double mResolution;
		double mOriginX;
		double mOriginY;
		double mOriginZ;

		int mSizeX;
		int mSizeY;
		int mSizeZ;

		int mStride1;
		int mStride2;

		int mNumNodes;

		Node3D *mNodes;
		std::vector<int> *mGeometricNeighbors;
		std::vector<int> *mSurfaceNeighbors;

		// Auxiliar stuff for search
		int *mHT;

		bool *mInSurface;
		int *mSurfaceIndices;
		int mNumSurfaceNodes;
        int *mHTSurface;

		int mCountPaths;
		std::vector< int > mOpenSet;
		std::vector<int> mPath;

		float mAlpha;
		float mMaxBrushDist;

		struct kdtree *mKdTree;
	
};

/////////// INLINE FUNCTIONS ////////////////////////

inline int LJM2::GetSizeX() const {
	return mSizeX;
}

inline int LJM2::GetSizeY() const {
	return mSizeY;
}

inline int LJM2::GetSizeZ() const {
	return mSizeZ;
}

inline int LJM2::ref( int _x, int _y, int _z ) const {
	return  _x*mStride1 + _y*mStride2 + _z;
}

inline bool LJM2::WorldToGrid( double _worldX, double _worldY, double _worldZ, int& _x, int& _y, int& _z ) const
{
  _x = int(round(( _worldX - mOriginX )/mResolution));
  _y = int(round(( _worldY - mOriginY )/mResolution));
  _z = int(round(( _worldZ - mOriginZ )/mResolution));

	return IsValid( _x, _y, _z );
}

inline bool LJM2::GridToWorld(int _x, int _y, int _z, double& _worldX, double& _worldY, double& _worldZ ) const
{
  _worldX = mOriginX + mResolution*(double(_x));
  _worldY = mOriginY + mResolution*(double(_y));
  _worldZ = mOriginZ + mResolution*(double(_z));
  return true;
}


#endif /** _LJM2_ */
