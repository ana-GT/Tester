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

/**
 * @function CheckProcess.cpp
 * @dat 2012-03-07
 */
#include "CheckProcess.h"

/**
 * @function CheckProcess
 * @brief Constructor
 */
CheckProcess::CheckProcess( double _sizeX, double _sizeY, double _sizeZ,
	    		    		double _originX, double _originY, double _originZ, 
			    			double _resolution )
{

  //-- Save data
  mSizeX = _sizeX;
  mSizeY = _sizeY;
  mSizeZ = _sizeZ; 
  mOriginX = _originX;
  mOriginY = _originY;
  mOriginZ = _originZ;
  mResolution = _resolution;

  //-- Create our slideBox
  build_slideBox();
}

/**
 * @function initialize
 * @brief Initialize the slideBox, basically
 */
void CheckProcess::build_slideBox()
{ 

  //-- Initialize the slideBox 
  mSlideBox = new RAPID_model;

  static double p0[3] = { 0, 0, 0 };
  static double p1[3] = { 0, mResolution, 0 };
  static double p2[3] = { mResolution, mResolution, 0 };
  static double p3[3] = { mResolution, 0, 0 };
  static double p4[3] = { 0, 0, mResolution };
  static double p5[3] = { 0, mResolution, mResolution };
  static double p6[3] = { mResolution, mResolution, mResolution };
  static double p7[3] = { mResolution, 0, mResolution };

  mSlideBox->BeginModel();
  mSlideBox->AddTri( p0, p2, p1, 0 );
  mSlideBox->AddTri( p0, p2, p3, 1 );
  mSlideBox->AddTri( p4, p6, p5, 2 );
  mSlideBox->AddTri( p4, p6, p7, 3 );
  mSlideBox->AddTri( p0, p5, p4, 4 );
  mSlideBox->AddTri( p0, p5, p1, 5 );
  mSlideBox->AddTri( p3, p6, p7, 6 );
  mSlideBox->AddTri( p3, p6, p2, 7 );
  mSlideBox->AddTri( p0, p7, p4, 8 );
  mSlideBox->AddTri( p0, p7, p3, 9 );
  mSlideBox->AddTri( p1, p6, p5, 10 );
  mSlideBox->AddTri( p1, p6, p2, 11 );
  //-- Should we add more inner triangles here?
  //-- Added : 2012-03-07
  mSlideBox->AddTri( p0, p1, p6, 12 );  // Diagonal 
  mSlideBox->AddTri( p0, p6, p7, 13 );  // Diagonal
  mSlideBox->AddTri( p3, p4, p5, 14 );  // Diagonal 
  mSlideBox->AddTri( p2, p4, p5, 15 );  // Diagonal

  mSlideBox->EndModel();

}

/**
 * @function getObjectsData
 * @brief Copy the information of the world objects in our class
 */
void CheckProcess::getObjectsData( std::vector<planning::Object*> _objects, std::string _objectNoIncludedName )
{
  mObjectNoIncludedName = _objectNoIncludedName;
  
  mObjs.resize( 0 );
  mObjNames.resize( 0 );

  mNumObjs = 0;

  //-- NOTICE! Each object has only one model (my ASSUMPTION)
  for( int i = 0; i < _objects.size(); i++ )
  { 
	if( _objects[i]->getName() != mObjectNoIncludedName ) {
		printf("-------(i) Getting info from %s \n", _objects[i]->getName().c_str() );
		CheckObject obj;
		obj.getModelData( _objects[i]->mModels[0] ); 
        mObjs.push_back(obj);
	    mObjNames.push_back( _objects[i]->getName() );
		mNumObjs++;
	}
  }
}  

/**
 * @function getLinksData
 * @brief Get data from the robot links
 */
void CheckProcess::getLinksData( planning::Robot* _robot, Eigen::VectorXi _linksID )
{
  mNumLinks = _linksID.size();
  mLinksID = _linksID;
  mLinks.resize( mNumLinks );


  Eigen::VectorXi ind(mNumLinks);

  // Only use the models indicated
  for( int i = 0; i < mNumLinks; i++ ) {
     for( int j = 0; j < _robot->mModels.size(); j++ ) {
	   if( _robot->mModelIndices[j] == _linksID[i] ) {
	     ind[i] = j;
       }
     }

  }

  
  for( int i = 0; i < mNumLinks; i++ ) 
  { 
    mLinks[i].getModelData( _robot->mModels[ind[i]] );
    mLinks[i].updateObjectData( _robot->getNode( mLinksID[i] ) );
  }
}


/**
 * @function build_voxel
 * @brief Collision quick checking
 */
void CheckProcess::build_voxel( std::vector<planning::Object*> _objects, LJM2 &_ljm2, int _inflated )
{
   //time_t ts; time_t tf; double dt;
   mObjsVoxels.clear();

   //ts = clock();
   //-- For each object
   for( int i = 0; i < mObjs.size(); ++i ) {
   	   for( int k = 0; k < _objects.size(); k++ )
   	   {
   	   		if( mObjNames[i] == _objects[k]->getName() ) {
      	   		// Each object has only one node, so get node 0 do it
      			mObjs[i].updateObjectData( _objects[k]->getNode(0) );

	      		//-- Convert to voxel grid
    	  		_ljm2.WorldToGrid( mObjs[i].min_x, mObjs[i].min_y, mObjs[i].min_z, mObjs[i].minCell_x, mObjs[i].minCell_y, mObjs[i].minCell_z );
    	  		_ljm2.WorldToGrid( mObjs[i].max_x, mObjs[i].max_y, mObjs[i].max_z, mObjs[i].maxCell_x, mObjs[i].maxCell_y, mObjs[i].maxCell_z );

    	  		printf(" --(i) Building voxel (%s): \n", mObjNames[i].c_str() );
    	  		printf( "* Min XYZ World: %f %f %f Min XYZ Cell: %d %d %d \n", mObjs[i].min_x, mObjs[i].min_y, mObjs[i].min_z, mObjs[i].minCell_x, mObjs[i].minCell_y, mObjs[i].minCell_z );
    	  	printf( "* Max XYZ World: %f %f %f Max XYZ Cell: %d %d %d \n", mObjs[i].max_x, mObjs[i].max_y, mObjs[i].max_z, mObjs[i].maxCell_x, mObjs[i].maxCell_y, mObjs[i].minCell_z );

    	  		//-- 
    	  		double idt[3][3];
    	  		idt[0][0] = 1; idt[0][1] = 0; idt[0][2] = 0;
    	 		idt[1][0] = 0; idt[1][1] = 1; idt[1][2] = 0;
    	  		idt[2][0] = 0; idt[2][1] = 0; idt[2][2] = 1;
  
		      	double tt[3]; 

    		  	//-- Building the voxel grid, sliding the basic box
    		  	for( int m = mObjs[i].minCell_x; m <= mObjs[i].maxCell_x; m++ )
    		   	{ for( int n = mObjs[i].minCell_y; n <= mObjs[i].maxCell_y; n++ )
    		     	{ for( int p = mObjs[i].minCell_z; p <= mObjs[i].maxCell_z; p++ )
    		       		{ 
    		         		_ljm2.GridToWorld( m, n, p, tt[0], tt[1], tt[2] ); 
    		         		RAPID_Collide( mObjs[i].R, mObjs[i].T, mObjs[i].rapidObject, idt, tt, mSlideBox, RAPID_FIRST_CONTACT );
    		         		if( RAPID_num_contacts != 0)
    		          		{ 
						 		if( _ljm2.IsValid(m, n, p) == false ) {
									printf("--(!) Error here, no valid obstacle voxel (%d %d %d)! \n", m, n, p );
						 		}
								_ljm2.SetState( m, n, p, OBSTACLE_STATE );
								//-- Set Inflated, if indicated
								if( _inflated > 0 ) {
									for( int xi = m - _inflated; xi <=  m + _inflated; ++xi ) {
										for( int yi = n - _inflated; yi <=  n + _inflated; ++yi ) {
											for( int zi = p - _inflated; zi <=  p + _inflated; ++zi ) {
												if ( _ljm2.IsValid( xi, yi, zi ) == false ) { continue; }
												if( _ljm2.GetState( xi, yi, zi ) == FREE_STATE ) {
													_ljm2.SetState( xi, yi, zi, INFLATED_STATE );
												}
											}
										}
									}
								}
    	            			mObjsVoxels.push_back( Eigen::Vector3i( m, n, p ) );
    	          			}
    	       			} 
    	     		}
    	   		}

				break; // Got the corresponding object, go to the following one
			} // end of if

		} // end of for _objects

   } //-- End of mObjs

  //tf = clock();
  //dt = (double) ( tf - ts )/CLOCKS_PER_SEC; 
  //printf("--** Voxel construction time: %.3f \n", dt );

}


/**
 * @function reportObjects
 * @brief print Stuff
 */
void CheckProcess::reportObjects()
{
  for( int i = 0; i < mNumObjs; i++ )
  { printf( " Object [%d] %s : center: ( %.3f,%.3f, %.3f ) -- radius: ( %.3f, %.3f, %.3f ) \n", i, mObjNames[i].c_str(), mObjs[i].center[0], mObjs[i].center[1], mObjs[i].center[2], mObjs[i].radius[0], mObjs[i].radius[1], mObjs[i].radius[2] );
  }
}


