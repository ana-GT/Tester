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
 * @function CheckProcess.h
 */
#ifndef CHECK_PROCESS_H_
#define CHECK_PROCESS_H_

#include <Eigen/Core>
#include <time.h>
#include <stdio.h>
#include <string>
#include <planning/Robot.h>
#include <vector>
#include "CheckObject.h"
#include "../LJM2/LJM2.h"

/**
 * @class CheckProcess
 */
class CheckProcess
{
  public:

  RAPID_model *mSlideBox;
  std::vector< CheckObject > mObjs;
  std::vector< std::string> mObjNames;
  std::vector< CheckObject > mLinks;
  int mNumObjs;
  int mNumLinks;

  std::string mObjectNoIncludedName;

  Eigen::VectorXi mLinksID;

  double mSizeX;
  double mSizeY;
  double mSizeZ; 
  double mOriginX;
  double mOriginY;
  double mOriginZ;
  double mResolution;

  CheckProcess( double _sizeX, double _sizeY, double _sizeZ,
 	    	    double _originX, double _originY, double _originZ, 
	            double _resolution );
  void build_slideBox();
  void getObjectsData( std::vector<planning::Object*> _objects, std::string _objectNoIncludedName = "default" );
  void getLinksData( planning::Robot* _robot, Eigen::VectorXi _linksID );
  void reportObjects();

  //-- Voxel construction
  void build_voxel( std::vector<planning::Object*> _objects, LJM2 &_ljm2, int _inflated = 0 );

  //-- Distance transform 
  std::vector< Eigen::Vector3i >mObjsVoxels;
};

#endif /** CHECK_PROCESS_H */
