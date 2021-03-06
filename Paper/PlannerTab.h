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

#ifndef _PAPER_PLANNER_TAB_
#define _PAPER_PLANNER_TAB_

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>

#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <kinematics/BodyNode.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>

#include <iostream>
#include <list>

#include "globalStuff.h"
#include "Planners/LJM2_Suite/myFunctions/CheckProcess.h"


/**
 * @class Planner
 * @brief Tab with Tester Planners
 */
class PlannerTab : public GRIPTab
{
 public:
  
  /// Functions related to Tab
  PlannerTab(){};
  PlannerTab( wxWindow * parent, wxWindowID id = -1,
	      const wxPoint & pos = wxDefaultPosition,
	      const wxSize & size = wxDefaultSize,
	      long style = wxTAB_TRAVERSAL);
  virtual ~PlannerTab();
  
  
  /// Public vars to capture external selection stuff 
  robotics::Object* selectedObject;
  robotics::Robot* selectedRobot;
  kinematics::BodyNode* selectedNode;

  void OnSlider(wxCommandEvent &evt);
  void OnButton(wxCommandEvent &evt);
  

  // ***** Workspace Plan A *****
  CheckProcess *mCp_A;
  LJM2 *mLjm2_A;

  Eigen::VectorXi mStartNode_A;
  Eigen::VectorXi mTargetNode_A;
  std::vector< std::vector<Eigen::VectorXd> > mWorkspacePaths_A;
  std::vector< std::vector<Eigen::Vector3i> > mNodePaths_A;
  std::vector< std::vector<Eigen::VectorXd> > mConfigPaths_A;

  wxTextCtrl *mPathIndex_A;

  // ***** Workspace Plan B *****
  CheckProcess *mCp_B;
  LJM2 *mLjm2_B;

  Eigen::VectorXi mStartNode_B;
  Eigen::VectorXi mTargetNode_B;
  std::vector< std::vector<Eigen::VectorXd> > mWorkspacePaths_B;
  std::vector< std::vector<Eigen::Vector3i> > mNodePaths_B;
  std::vector< std::vector<Eigen::VectorXd> > mConfigPaths_B;

  wxTextCtrl *mPathIndex_B;


  // ****  3D Info ****
  wxTextCtrl *mSizeXText;
  wxTextCtrl *mSizeYText;
  wxTextCtrl *mSizeZText;

  wxTextCtrl *mOriginXText;
  wxTextCtrl *mOriginYText;
  wxTextCtrl *mOriginZText;

  wxTextCtrl *mPaddingText;
  wxTextCtrl *mResolText;

  wxTextCtrl *mAlphaText;
  wxTextCtrl *mNumPathsText;

  double mAlpha;
  int mNumPaths;


  void WorkspacePlan_A(); 
  void WorkspacePlan_B(); 

  // ****************************

    void GRIPStateChange();
    DECLARE_DYNAMIC_CLASS( PlannerTab )
    DECLARE_EVENT_TABLE()
};

#endif /** _PAPER_PLANNER_PLANNER_TAB_ */

