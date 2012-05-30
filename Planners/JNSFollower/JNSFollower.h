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

#ifndef _JNS_FOLLOWER_H_
#define _JNS_FOLLOWER_H_

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <planning/World.h>
#include <Tools/Collision.h>

/**
 * @class JNSFollower
 * @brief Simple Follower with Jacobian PseudoInverse
 */
class JNSFollower {

public:

    /// Member variables
  double mConfigStep;
  planning::World *mWorld;
  Collision *mCollision;
  int mRobotId;
  Eigen::VectorXi mLinks;
  double mWorkspaceThresh;
  
  kinematics::BodyNode *mEENode;
  int mEEId;
  int mMaxIter;
  
  /// Constructor
  JNSFollower();
  JNSFollower( planning::World &_world,
	       Collision *_collision,
	       bool _copyWorld = false,
	       double _configStep = 1.05 ); // 0.35 = sqrt(7)*1_degree
    
  /// Destructor
  ~JNSFollower();
  
  /// Planner itself
  std::vector< Eigen::VectorXd > PlanPath( int _robotId,
					   const Eigen::VectorXi &_links,
					   const Eigen::VectorXd &_start,  // Configuration,
					   std::string _EEName,
					   int _EEId,
					   double _res,
					   const std::vector<Eigen::VectorXd> &_workspacePath ); // Pose    
    
  Eigen::MatrixXd GetPseudoInvJac( Eigen::VectorXd _q ) ;
  bool GoToEEPos( Eigen::VectorXd &_q, 
		  Eigen::VectorXd _targetPos, 
		  std::vector<Eigen::VectorXd> &_workspacePath );
  Eigen::VectorXd GetEEPos( Eigen::VectorXd _q );
    
 private:
  /// Member variables
  bool mCopyWorld;
  
};

#endif /** _JNS_FOLLOWER_H_ */

