/**
 * @file TrackerTab.h
 * @author A. Huaman
 * @date June 8, 2012
 */

#ifndef _TRACKER_TAB_H_
#define _TRACKER_TAB_H_

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>

#include <planning/Robot.h>
#include <planning/Object.h>
#include <kinematics/BodyNode.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>

#include <iostream>
#include <list>

#include "Planners/IK.h"
#include "Planners/IKSearch.h"
#include "Planners/IKGradient.h"
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

  wxTextCtrl *mW_ManipG;
  wxTextCtrl *mW_JraG;
  wxTextCtrl *mW_DexterityS;
  wxTextCtrl *mW_JraS;
  wxTextCtrl *mW_JvmS;


  // ***************************************
  // NEVER FORGET THIS!
  DECLARE_DYNAMIC_CLASS( TrackerTab )
  DECLARE_EVENT_TABLE()
};

#endif /** _TRACKER_TAB_ */

