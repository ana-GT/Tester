/**
 * @file TrackerTab.h
 * @author A. Huaman
 * @date June 8, 2012
 */

#ifndef _TRACKER_TAB_H_
#define _TRACKER_TAB_H_

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
#include "Planners/IK/IKSearch.h"
#include "Planners/IK/IKGradient.h"
#include "globalStuff.h"

using namespace std;

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

  // Key variables
  IK *mIk;
  std::vector<Eigen::VectorXd> mExecutePath;
  std::vector<Eigen::VectorXd> mTrackPath;
  std::vector<Eigen::VectorXd> mTrack_BT_Path;
  std::vector<Eigen::VectorXd> mTrack_LA_Path;

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

#endif /** _TRACKER_TAB_ */

