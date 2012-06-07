 /*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

/**
 * @author A. Huaman
 * @date 2012-05-28 -- modified Jun 7, 2012
 */
#include "GRIPApp.h"
#include "ConfigTab.h"
#include "TrackerTab.h"
#include "PlannerTab.h"


extern wxNotebook* tabView;

class TesterTabApp : public GRIPApp {
	virtual void AddTabs() {
	  tabView->AddPage(new ConfigTab(tabView), wxT("Config Tab"));
	  tabView->AddPage( new PlannerTab(tabView), wxT("Planner Tab")); 
	  tabView->AddPage( new TrackerTab(tabView), wxT("Tracker Tab"));
	}
};

IMPLEMENT_APP(TesterTabApp)
