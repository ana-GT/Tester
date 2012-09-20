/**
 * @file TrackerTab.h
 * @author A. Huaman
 * @date July 1, 2012
 */

#ifndef _PAPER_TRACKER_TAB_H_
#define _PAPER_TRACKER_TAB_H_

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

#include "Planners/IK/IK.h"
#include "Planners/IK/IKGradient.h"
#include "Planners/IK/IKSearch.h"
#include "globalStuff.h"


/**
 * @class TrackerTab
 */
class TrackerTab : public GRIPTab 
{
 public:
  TrackerTab(){};
  TrackerTab( wxWindow *parent, wxWindowID id = -1,
	      const wxPoint &pos = wxDefaultPosition,
	      const wxSize & size = wxDefaultSize,
	      long style = wxTAB_TRAVERSAL );
  virtual ~TrackerTab();

  //Functions
  void OnButton( wxCommandEvent &evt );

  void ExecuteBoth();
  void ExecuteBothWithObject(); 

  // Key variables
  IK *mIk;
  std::vector<Eigen::VectorXd> mExecutePath;
  std::vector<Eigen::VectorXd> mExecutePath_A;
  std::vector<Eigen::VectorXd> mExecutePath_B;
  std::vector<Eigen::VectorXd> mTrackPath_A;
  std::vector<Eigen::VectorXd> mTrackPath_B;
  std::vector<Eigen::VectorXd> mTrack_BT_Path_A;
  std::vector<Eigen::VectorXd> mTrack_BT_Path_B;
  std::vector<Eigen::VectorXd> mTrack_LA_Path;

  ////
  std::vector<Eigen::VectorXd> mTrack_Test_Path;

  Eigen::VectorXd mNSConf;

  wxTextCtrl *mBT_Window;
  wxTextCtrl *mLA_Window;

  wxTextCtrl *mNumCoeff;
  wxTextCtrl *mMinCoeff;
  wxTextCtrl *mMaxCoeff;

  // ***************************************
  // NEVER FORGET THIS!
  DECLARE_DYNAMIC_CLASS( TrackerTab )
  DECLARE_EVENT_TABLE()
};

#endif /** _PAPER_TRACKER_TAB_ */

