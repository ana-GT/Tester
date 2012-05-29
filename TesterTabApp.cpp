 /*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

/**
 * @author A. Huaman
 * @date 2012-05-28
 */
#include "GRIPApp.h"
#include "ConfigTab.h"
#include "PlannerTab.h"

extern wxNotebook* tabView;

class TesterTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new ConfigTab(tabView), wxT("Config Tab"));
		tabView->AddPage( new PlannerTab(tabView), wxT("Planner Tab")); 
	}
};

IMPLEMENT_APP(TesterTabApp)
