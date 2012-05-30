/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>
#include <string>
#include <iostream>

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>

#include "PlannerTab.h"

using namespace std;

// Control IDs (used for event handling - be sure to start with a non-conflicted id)
enum PlannerTabEvents {
  button_RRT = 100,
  button_RRTExecute,
  button_Get3DInfo,
  button_Plot3DInfo,
  button_RunLJM2,
  button_Plot3DPaths,
  button_Plot3DPath,
  button_Follow3DPath_NS,
  button_Follow3DPath_PI,

  button_Dummy,
  slider_Alpha,
  slider_NumPaths
};


// sizer for whole tab
wxBoxSizer* sizerFull_Planner;

//Add a handler for any events that can be generated by the widgets you add here (sliders, radio, checkbox, etc)
BEGIN_EVENT_TABLE( PlannerTab, wxPanel )
EVT_COMMAND ( wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, PlannerTab::OnSlider )
EVT_COMMAND ( wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, PlannerTab::OnButton )
EVT_COMMAND( wxID_ANY, wxEVT_COMMAND_RADIOBOX_SELECTED, PlannerTab::OnRadio )
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS( PlannerTab, GRIPTab )

/**
 * @function PlannerTab
 * @brief Constructor
 */
PlannerTab::PlannerTab( wxWindow *parent, const wxWindowID id,
			const wxPoint& pos, const wxSize& size, long style) :
GRIPTab(parent, id, pos, size, style ) {

    sizerFull_Planner = new wxBoxSizer( wxHORIZONTAL );
 
    // ** Random Planners **

    // Create StaticBox container for all items
    wxStaticBox* RandomBox = new wxStaticBox(this, -1, wxT("Random Planners"));

    // Create sizer for this box with horizontal layout
    wxStaticBoxSizer* RandomBoxSizer = new wxStaticBoxSizer(RandomBox, wxHORIZONTAL);

    // Create sizer for start buttons in 1st column
    wxBoxSizer *RandomButtonSizer = new wxBoxSizer(wxVERTICAL);
    RandomButtonSizer->Add( new wxButton(this, button_RRT, wxT("RRT")), 0, wxALL, 1 );
    RandomButtonSizer->Add( new wxButton( this, button_RRTExecute, wxT("Execute")), 0, wxALL, 1 );
    RandomBoxSizer->Add( RandomButtonSizer, 1, wxALIGN_NOT ); 

    sizerFull_Planner->Add( RandomBoxSizer, 1, wxEXPAND | wxALL, 4 );

    // ** Discrete Planners **

    // Create StaticBox container for all items
    wxStaticBox* DiscreteBox = new wxStaticBox(this, -1, wxT("Discrete Planners"));

    // Create sizer for this box with horizontal layout
    wxStaticBoxSizer* DiscreteBoxSizer = new wxStaticBoxSizer(DiscreteBox, wxVERTICAL);

    // 3D Info
    wxBoxSizer *Info3DSizer = new wxBoxSizer(wxHORIZONTAL);
    Info3DSizer->Add( new wxButton(this, button_Get3DInfo, wxT("Get 3D Info")), 0, wxALL, 1 );
    Info3DSizer->Add( new wxButton(this, button_Plot3DInfo, wxT("Plot 3D Info")), 0, wxALL, 1 );

    DiscreteBoxSizer->Add( Info3DSizer, 1, wxALIGN_NOT ); 

  // LJM2 Parameters
    wxBoxSizer *LJM2ParamSizer = new wxBoxSizer(wxHORIZONTAL);

    mSlider_Alpha = new GRIPSlider( "alpha", 0.0, 1.0, 50, 0.01, 10, 50, this, slider_Alpha ); 
    LJM2ParamSizer->Add( mSlider_Alpha, 0, wxEXPAND | wxALL, 1 );
  
    mSlider_NumPaths = new GRIPSlider( "num paths:", 0.0, 5.0, 5, 1, 10, 50, this, slider_NumPaths ); 
    LJM2ParamSizer->Add( mSlider_NumPaths, 0, wxEXPAND | wxALL, 1 );

    DiscreteBoxSizer->Add( LJM2ParamSizer, 1, wxALIGN_NOT ); 

    // LJM2 Run Info
    wxBoxSizer *LJM2RunSizer = new wxBoxSizer(wxHORIZONTAL);
    LJM2RunSizer->Add( new wxButton(this, button_RunLJM2, wxT("Run LJM2")), 0, wxALL, 1 );
    LJM2RunSizer->Add( new wxButton(this, button_Plot3DPaths, wxT("Plot 3D Paths")), 0, wxALL, 1 );

    DiscreteBoxSizer->Add( LJM2RunSizer, 1, wxALIGN_NOT ); 

    // Execute LJM2Path
    wxBoxSizer *LJM2ExecuteSizer = new wxBoxSizer(wxHORIZONTAL);

    wxBoxSizer *LJM2ExecuteButtonSizer = new wxBoxSizer(wxHORIZONTAL);
    LJM2ExecuteButtonSizer->Add( new wxButton(this, button_Plot3DPath, wxT("Plot path")), 0, wxALL, 1 );
    LJM2ExecuteButtonSizer->Add( new wxButton(this, button_Follow3DPath_NS, wxT("Follow NS")), 0, wxALL, 1 );
    LJM2ExecuteButtonSizer->Add( new wxButton(this, button_Follow3DPath_PI, wxT("Follow PI")), 0, wxALL, 1 );

    LJM2ExecuteSizer->Add( LJM2ExecuteButtonSizer, 1, wxALL, 2 );

    wxBoxSizer *pathIndexSizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *pathIndexLabel = new wxStaticText( this, 1013, wxT("Path:") );
    mPathIndex = new wxTextCtrl(this,1014,wxT("0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

    pathIndexSizer->Add( pathIndexLabel, 0, wxALL, 1 );
    pathIndexSizer->Add( mPathIndex, 0, wxALL, 1 );
    LJM2ExecuteSizer->Add( pathIndexSizer, 0, wxALL, 1 );


    DiscreteBoxSizer->Add( LJM2ExecuteSizer, 1, wxALIGN_NOT ); 

    // Add DiscreteBoxSizer to sizerFull_Planner
    sizerFull_Planner->Add( DiscreteBoxSizer, 2, wxEXPAND | wxALL, 4 );

    // ** Dummy **

    // Create StaticBox container for all items
    wxStaticBox* DummyBox = new wxStaticBox(this, -1, wxT("Dummy"));

    // Create sizer for this box with horizontal layout
    wxStaticBoxSizer* DummyBoxSizer = new wxStaticBoxSizer(DummyBox, wxVERTICAL);

    // Create sizer for start buttons in 1st column
    wxBoxSizer *DummyButtonSizer = new wxBoxSizer(wxVERTICAL);
    DummyButtonSizer->Add( new wxButton(this, button_Dummy, wxT("Dummy")), 0, wxALL, 1 );

    DummyBoxSizer->Add( DummyButtonSizer, 1, wxALIGN_NOT ); 

    wxBoxSizer *timeSizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *timeLabel = new wxStaticText( this, 1007, wxT("T: ") );
    timeText = new wxTextCtrl(this,1008,wxT("5.0") ); //,wxDefaultPosition,wxSize(40,20),wxTE_RIGHT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

    timeSizer->Add(timeLabel,2,wxALL,1);
    timeSizer->Add(timeText,2,wxALL,1);
    DummyBoxSizer->Add( timeSizer, 1, wxALIGN_NOT );

    sizerFull_Planner->Add( DummyBoxSizer, 1, wxEXPAND | wxALL, 4 );



    SetSizer(sizerFull_Planner);

}

/**
 * @function ~PlannerTab
 * @brief Destructor
 */ 
PlannerTab::~PlannerTab(){

}


/**
 * @function OnButton
 * @brief Handle Button Events
 */
void PlannerTab::OnButton(wxCommandEvent &evt) {

    int button_num = evt.GetId();

    switch (button_num) {
      
        /** RRT */
    case button_RRT: {
      RRTPlan();
    }
      break;
      
      /** RRT Execute */
    case button_RRTExecute: {
      
    }
      break;


      /** Get 3D Info */
    case button_Get3DInfo:
      {
	printf("**********************\n");
	printf("      Get 3D Info     \n");
	printf("**********************\n");
	printf("--(!) Be sure to set start conf and target object or I will output rubbish \n");
	mLjm2 = new LJM2( gSizeX, gSizeY, gSizeZ, gOriginX, gOriginY, gOriginZ, gResolution );
	mCp = new CheckProcess( gSizeX, gSizeY, gSizeZ, gOriginX, gOriginY, gOriginZ, gResolution );
	mCp->getObjectsData( mWorld->mObjects, gTargetObjectName );  
	mCp->build_voxel( mWorld->mObjects, *mLjm2, gPadding ); // Here your LJM2 is built
	mCp->reportObjects(); 
	printf(" (i) Process Geometry (i) \n");
	mLjm2->ProcessGeometry();
	printf(" (i) End process geometry (i)  \n");
      }		
      break;
      
      /** Plot 3D Configuration */
    case button_Plot3DInfo:
      {
	printf("*************************************\n");
	printf( "     Plotting 3D Configuration      \n");
	printf("*************************************\n");
	pcl::visualization::PCLVisualizer *viewer;
	viewer = new pcl::visualization::PCLVisualizer( "Discretized Workspace" );

	mLjm2->ViewObstacles( viewer, 0, 0, 255 );
	
	while( !viewer->wasStopped() ) {
	  viewer->spin();
	}
	delete viewer;
      }		
      break;
      
      /** Execute Plan */
    case button_RunLJM2: {
      WorkspacePlan();
    }
      break;

      /** Plot 3D Paths*/
    case button_Plot3DPaths: {
      printf("******************************\n");
      printf( "     Plotting 3D Paths       \n");
      printf("******************************\n");
      pcl::visualization::PCLVisualizer *viewer;
      viewer = new pcl::visualization::PCLVisualizer( "Workspace Paths" );
      mLjm2->ViewObstacles( viewer, 0, 0, 255 );
      mLjm2->ViewPaths( mNodePaths, viewer );
      mLjm2->ViewBall( viewer, mStartNode(0), mStartNode(1), mStartNode(2), "Start" );
      mLjm2->ViewBall( viewer, mTargetNode(0), mTargetNode(1), mTargetNode(2), "Target" );
      
      while( !viewer->wasStopped() ) {
	viewer->spin();
      }
      delete viewer; 
    }
      break;

      /** Show Path */
    case button_Plot3DPath: {     

      double temp;
      mPathIndex->GetValue().ToDouble(&temp);
      int n = (int) temp;
    
      printf("*******************************\n");
      printf( "     Plotting 3D Path %d       \n", n);
      printf("*******************************\n");
      pcl::visualization::PCLVisualizer *viewer;
      viewer = new pcl::visualization::PCLVisualizer( "Workspace Path" );
      mLjm2->ViewObstacles( viewer, 0, 0, 255 );
      mLjm2->ViewPath( mNodePaths[n], viewer );
      mLjm2->ViewBall( viewer, mStartNode(0), mStartNode(1), mStartNode(2), "Start" );
      mLjm2->ViewBall( viewer, mTargetNode(0), mTargetNode(1), mTargetNode(2), "Target" );
      
      while( !viewer->wasStopped() ) {
	viewer->spin();
      }
      delete viewer; 
    }    
      break;

      /** Follow NS - Nullspace search */
    case button_Follow3DPath_NS: {
      double temp;
      mPathIndex->GetValue().ToDouble(&temp);
      int n = (int) temp;
	  printf("Executing workspace %d -- Follow NS \n", n);
	  WorkspaceExecute( mWorkspacePaths[n], 0 );
    }

      /** Follow PI - Pseudo Inverse alone */
    case button_Follow3DPath_PI: {
      double temp;
      mPathIndex->GetValue().ToDouble(&temp);
      int n = (int) temp;
	  printf("Executing workspace %d -- Follow JT \n", n);
	  WorkspaceExecute( mWorkspacePaths[n], 1 );
    }

      /** Dummy */
    case button_Dummy: { 
    }
      break;

    } // end of switch
}


/**
 * @function RRTPlan
 */
void PlannerTab::RRTPlan() {

  double stepSize = 0.02;	  
  PathPlanner *planner = new PathPlanner( *mWorld, mCollision, false, stepSize );
  
  int maxNodes = 5000;
  bool result = planner->planPath( gRobotId, 
				   gLinks, 
				   gStartConf, 
				   gTargetConf, 
				   true, // bidirectional  
				   true, // connect
				   true, // greedy 
				   false, // smooth 
				   maxNodes );
  
  RRTExecute( planner->path );
  
}

/**
 * @function RRTExecute
 * @brief 
 */
void PlannerTab::RRTExecute( std::list<Eigen::VectorXd> _path ) {

    if( mWorld == NULL || _path.size() == 0 ) {
        cout << "--(!) Must create a valid plan before updating its duration (!)--" << endl;
	return;
    }

    double T;
    timeText->GetValue().ToDouble(&T);

    int numsteps = _path.size();
    double increment = T/(double)numsteps;

    cout << "-->(+) Updating Timeline - Increment: " << increment << " Total T: " << T << " Steps: " << numsteps << endl;

    frame->InitTimer( string("Plan"),increment );

    Eigen::VectorXd vals( gLinks.size() );

    for( std::list<Eigen::VectorXd>::iterator it = _path.begin(); it != _path.end(); it++ ) {

        mWorld->mRobots[gRobotId]->setDofs( *it, gLinks );
	mWorld->mRobots[gRobotId]->update();

        frame->AddWorld( mWorld );
    }
}

/**
 * @function WorkspacePlan
 */
void PlannerTab::WorkspacePlan() {
 
    /// Check start position is not in collision
    mWorld->mRobots[gRobotId]->setDofs( gStartConf, gLinks );

    if( mCollision->CheckCollisions() ) {   
      printf(" --(!) Initial status is in collision. I am NOT proceeding. Exiting \n");
      return; 
    }

   //-- Setting start cell
   gStartPos = GetEE_Pos( gStartConf );

   mStartNode.resize(3);
   mTargetNode.resize(3);

   if( mLjm2->WorldToGrid( gStartPos(0), gStartPos(1), gStartPos(2), mStartNode(0), mStartNode(1), mStartNode(2) ) == false )
   {  printf("(x) Error: Start Position no valid  (x)\n"); return; } 
   if( mLjm2->WorldToGrid( gTargetPos(0), gTargetPos(1), gTargetPos(2), mTargetNode(0), mTargetNode(1), mTargetNode(2) ) == false )
   {  printf("(x) Error: Target Position no valid  (x)\n"); return; }    


   //-- Plan now workspace guys
   printf("(o) Planning from (%d %d %d) to (%d %d %d) (o)\n", mStartNode(0), mStartNode(1), mStartNode(2), mTargetNode(0), mTargetNode(1), mTargetNode(2) );
   printf("(o) Start State: %d  Target state: %d (FREE: 1 OBSTACLE: 2) (o)\n", mLjm2->GetState(mStartNode(0), mStartNode(1), mStartNode(2)), mLjm2->GetState(mTargetNode(0), mTargetNode(1), mTargetNode(2)));

   //-- Check alpha and numPaths
   printf("---(i) Search Parameters: Alpha: %.3f - Num Paths: %d \n", mAlpha, mNumPaths );
   
   mNodePaths = mLjm2->FindVarietyPaths2( mStartNode(0), mStartNode(1), mStartNode(2), mTargetNode(0), mTargetNode(1), mTargetNode(2), mNumPaths, mAlpha );
   mWorkspacePaths = mLjm2->NodePathToWorkspacePath( mNodePaths );
   printf("-------(i) Finished Workpace Planning (i)------- \n");
   
}

/**
 * @function WorkspaceExecute
 * @brief 
 */
void PlannerTab::WorkspaceExecute( std::vector<Eigen::VectorXd> _path, int _type ) {

    if( mWorld == NULL || _path.size() == 0 ) {
        std::cout << "--(!) Must create a valid plan before updating its duration (!)--" << std::endl;
	      return;
    }
    
    std::vector< Eigen::VectorXd > configPath;

    if( _type == 0 ) { // Nullspace
      JNSFollower jns( *mWorld, mCollision, false );	
      configPath = jns.PlanPath( gRobotId, gLinks, gStartConf, gEEName, gEEId, gResolution, _path );
    }
    else if( _type == 1 ) { // Pseudo Inverse
      JTFollower jt( *mWorld, mCollision, false );	
      configPath = jt.PlanPath( gRobotId, gLinks, gStartConf, gEEName, gEEId, gResolution, _path );
    }

    double T;
    timeText->GetValue().ToDouble(&T);
    
    int numsteps = configPath.size();
    double increment = T/(double)numsteps;

    cout << "-->(+) Updating Timeline - Increment: " << increment << " Total T: " << T << " Steps: " << numsteps << endl;

    frame->InitTimer( string("Plan"),increment );


    Eigen::VectorXd vals( gLinks.size() );

    for( size_t i = 0; i < numsteps; ++i ) {
      mWorld->mRobots[gRobotId]->setDofs( configPath[i], gLinks );
      mWorld->mRobots[gRobotId]->update();
      
      frame->AddWorld( mWorld );
    }
    
}


/**
 * @function OnSlider
 * @brief Handle slider changes
 */
void PlannerTab::OnSlider(wxCommandEvent &evt) {

    /*
    if ( selectedTreeNode == NULL ) {
        return;
    } */ /// Do I need this now? - AHQ: Dec 6th, 2012

    int slnum = evt.GetId();
    double pos = *(double*) evt.GetClientData();
    char numBuf[1000];

    switch (slnum) {

    case slider_Alpha:
      mAlpha = pos;
      break;

    case slider_NumPaths:
      mNumPaths = (int) pos;
      break;
      
    default:
      return;
    }

    //world->updateCollision(o);
    //viewer->UpdateCamera();

    if (frame != NULL)
        frame->SetStatusText(wxString(numBuf, wxConvUTF8));
}

/**
 * @function OnRadio
 */
void PlannerTab::OnRadio( wxCommandEvent &evt ) {
   
}

/**
 * @function GRIPStateChange -- Keep using this name as it is a virtual function
 * @brief This function is called when an object is selected in the Tree View or other
 *        global changes to the RST world. Use this to capture events from outside the tab.
 */
void PlannerTab::GRIPStateChange() {
    if ( selectedTreeNode == NULL ) {

        return;
    }

    string statusBuf;
    string buf, buf2;

    switch (selectedTreeNode->dType) {

        case Return_Type_Object:
	          selectedObject = (planning::Object*) ( selectedTreeNode->data );
	          statusBuf = " Selected Object: " + selectedObject->getName();
	          buf = "You clicked on object: " + selectedObject->getName();
	          // Enter action for object select events here:
	          break;

	      case Return_Type_Robot:
	          selectedRobot = (planning::Robot*) ( selectedTreeNode->data );
	          statusBuf = " Selected Robot: " + selectedRobot->getName();
	          buf = " You clicked on robot: " + selectedRobot->getName();
      	    // Enter action for Robot select events here:
	          break;
	      case Return_Type_Node:
	          selectedNode = (kinematics::BodyNode*) ( selectedTreeNode->data );
	          statusBuf = " Selected Body Node: " + string(selectedNode->getName()) + " of Robot: "
			      + ( (planning::Robot*) selectedNode->getSkel() )->getName();
	          buf = " Node: " + string(selectedNode->getName()) + " of Robot: " + ( (planning::Robot*) selectedNode->getSkel() )->getName();
	          // Enter action for link select events here:
      	    break;
        default:
            fprintf(stderr, "--( :D ) Someone else's problem!\n");
            assert(0);
            exit(1);
    }

    //cout << buf << endl;
    frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
    sizerFull_Planner->Layout();
}
