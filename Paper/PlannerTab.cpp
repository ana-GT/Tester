/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
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
  button_Get3DInfo_A = 100,
  button_Plot3DInfo_A,
  button_Run_LJM2_A,
  button_Plot3DPaths_A,
  button_Plot3DPath_A,
  button_Save3DPath_A,
  button_Get3DInfo_B,
  button_Plot3DInfo_B,
  button_Run_LJM2_B,
  button_Plot3DPaths_B,
  button_Plot3DPath_B,
  button_Save3DPath_B
};


// sizer for whole tab
wxBoxSizer* sizerFull_Planner;

//Add a handler for any events that can be generated by the widgets you add here (sliders, radio, checkbox, etc)
BEGIN_EVENT_TABLE( PlannerTab, wxPanel )
EVT_COMMAND ( wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, PlannerTab::OnSlider )
EVT_COMMAND ( wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, PlannerTab::OnButton )
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
  
  // ** SET SIZER FULL FIRST THING **
  sizerFull_Planner = new wxBoxSizer( wxHORIZONTAL );
  
  // ** SET 3D PARAMETERS **
  wxStaticBox* Param3D_Box = new wxStaticBox(this, -1, wxT("3D Parameters"));
  wxStaticBoxSizer* Param3D_BoxSizer = new wxStaticBoxSizer( Param3D_Box, wxVERTICAL );

  // ** Size Sizer **
  wxBoxSizer *sizeSizer = new wxBoxSizer(wxHORIZONTAL);

  // -- x --
  wxBoxSizer *sizeXSizer = new wxBoxSizer( wxHORIZONTAL );
  wxStaticText *sizeXLabel = new wxStaticText( this, 1007, wxT(" Size x: ") );
  mSizeXText = new wxTextCtrl(this,1008,wxT("1.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
  
  sizeXSizer->Add( sizeXLabel, 0, wxALL, 1 );
  sizeXSizer->Add( mSizeXText, 0, wxALL, 1 );
  
  sizeSizer->Add( sizeXSizer, 0, wxALL, 1 );

  // -- y --
  wxBoxSizer *sizeYSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *sizeYLabel = new wxStaticText( this, 1009, wxT(" y: ") );
  mSizeYText = new wxTextCtrl(this,1010,wxT("1.0"), wxDefaultPosition,wxSize(40,20),wxTE_RIGHT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
  
  sizeYSizer->Add( sizeYLabel, 0, wxALL, 1 );
  sizeYSizer->Add( mSizeYText, 0, wxALL, 1 );
  
  sizeSizer->Add( sizeYSizer, 0, wxALL, 1 );

  // -- z --
  wxBoxSizer *sizeZSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *sizeZLabel = new wxStaticText( this, 1011, wxT(" z: ") );
  mSizeZText = new wxTextCtrl(this,1012,wxT("1.0"), wxDefaultPosition,wxSize(40,20),wxTE_RIGHT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
  
  sizeZSizer->Add( sizeZLabel, 0, wxALL, 1 );
  sizeZSizer->Add( mSizeZText, 0, wxALL, 1 );
  
  sizeSizer->Add( sizeZSizer, 0, wxALL, 1 );
  
  // -- * --
  Param3D_BoxSizer->Add( sizeSizer, 0, wxALIGN_NOT, 1 );
  
  /** Origin Sizer */
  wxBoxSizer *originSizer = new wxBoxSizer(wxHORIZONTAL);
  
  // -- x --
  wxBoxSizer *originXSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *originXLabel = new wxStaticText( this, 1013, wxT(" Orig x: ") );
  mOriginXText = new wxTextCtrl(this,1014,wxT("-0.1"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
  
  originXSizer->Add( originXLabel, 0, wxALL, 1 );
  originXSizer->Add( mOriginXText, 0, wxALL, 1 );
  
  originSizer->Add( originXSizer, 0, wxALL, 1 );
  
  // -- y --
  wxBoxSizer *originYSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *originYLabel = new wxStaticText( this, 1015, wxT(" y: ") );
  mOriginYText = new wxTextCtrl(this,1016,wxT("-0.2"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

  originYSizer->Add( originYLabel, 0, wxALL, 1 );
  originYSizer->Add( mOriginYText, 0, wxALL, 1 );
  
  originSizer->Add( originYSizer, 0, wxALL, 1 );
  
  // -- z --
  wxBoxSizer *originZSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *originZLabel = new wxStaticText( this, 1017, wxT(" z: ") );
  mOriginZText = new wxTextCtrl(this,1018,wxT("0.05"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
  
  originZSizer->Add( originZLabel, 0, wxALL, 1 );
  originZSizer->Add( mOriginZText, 0, wxALL, 1 );
  
  originSizer->Add( originZSizer, 0, wxALL, 1 );
    
  // -- * --
  Param3D_BoxSizer->Add( originSizer, 0, wxALIGN_NOT, 1 );


  /** Detail Sizer */
  wxBoxSizer *detailSizer = new wxBoxSizer(wxHORIZONTAL);
  
    // resolution
  wxBoxSizer *resolSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *resolLabel = new wxStaticText( this, 1019, wxT(" Res:    ") );
  mResolText = new wxTextCtrl(this,1020,wxT("0.02"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
  
  resolSizer->Add( resolLabel, 0, wxALL, 1 );
  resolSizer->Add( mResolText, 0, wxALL, 1 );

  detailSizer->Add( resolSizer, 0, wxALL, 1 );
  
  // inflated
  wxBoxSizer *paddingSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *paddingLabel = new wxStaticText( this, 1021, wxT("          Pad: ") );
  mPaddingText = new wxTextCtrl(this,1022,wxT("1"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
  
  paddingSizer->Add( paddingLabel, 0, wxALL, 1 );
  paddingSizer->Add( mPaddingText, 0, wxALL, 1 );
  
  detailSizer->Add( paddingSizer, 0, wxALL, 1 );

  // -- * --
  Param3D_BoxSizer->Add( detailSizer, 0, wxALIGN_NOT, 1 );

  // 3D Info 
  wxBoxSizer *Info3D_Sizer = new wxBoxSizer(wxVERTICAL);

  wxBoxSizer *Info3D_A_Sizer = new wxBoxSizer(wxHORIZONTAL);
  Info3D_A_Sizer->Add( new wxButton(this, button_Get3DInfo_A, wxT("Get 3D Info A")), 0, wxALL, 1 );
  Info3D_A_Sizer->Add( new wxButton(this, button_Plot3DInfo_A, wxT("Plot 3D Info A")), 0, wxALL, 1 );
  Info3D_Sizer->Add( Info3D_A_Sizer, 0, wxALL, 1 );

  wxBoxSizer *Info3D_B_Sizer = new wxBoxSizer(wxHORIZONTAL);
  Info3D_B_Sizer->Add( new wxButton(this, button_Get3DInfo_B, wxT("Get 3D Info B")), 0, wxALL, 1 );
  Info3D_B_Sizer->Add( new wxButton(this, button_Plot3DInfo_B, wxT("Plot 3D Info B")), 0, wxALL, 1 );
  Info3D_Sizer->Add( Info3D_B_Sizer, 0, wxALL, 1 );


  Param3D_BoxSizer->Add( Info3D_Sizer, 1, wxALIGN_NOT ); 

  // **----- Add to sizerFull -----**
  sizerFull_Planner->Add( Param3D_BoxSizer, 2, wxEXPAND | wxALL, 4 ); 

  // ** WORKSPACE PLANNER **
 
  // Create StaticBox container for all items
  wxStaticBox* WSPlanner_Box = new wxStaticBox(this, -1, wxT("WS Planner"));
  
  // Create sizer for this box with horizontal layout
  wxStaticBoxSizer* WSPlanner_BoxSizer = new wxStaticBoxSizer( WSPlanner_Box, wxVERTICAL );

  // LJM2 Parameters
  wxBoxSizer *LJM2Param_Sizer = new wxBoxSizer(wxHORIZONTAL);
  
  // -- Alpha --
  wxBoxSizer *alpha_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *alpha_Label = new wxStaticText( this, 1013, wxT(" Alpha: ") );
  mAlphaText = new wxTextCtrl(this,1014,wxT("0.1"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
  
  alpha_Sizer->Add( alpha_Label, 0, wxALL, 1 );
  alpha_Sizer->Add( mAlphaText, 0, wxALL, 1 );
  
  LJM2Param_Sizer->Add( alpha_Sizer, 0, wxALL, 1 );
  
  // -- Num Paths --
  wxBoxSizer *numPaths_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *numPaths_Label = new wxStaticText( this, 1015, wxT(" Num Paths: ") );
  mNumPathsText = new wxTextCtrl(this,1016,wxT("3"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

  numPaths_Sizer->Add( numPaths_Label, 0, wxALL, 1 );
  numPaths_Sizer->Add( mNumPathsText, 0, wxALL, 1 );
  
  LJM2Param_Sizer->Add( numPaths_Sizer, 0, wxALL, 1 );

  // -- * --
  WSPlanner_BoxSizer->Add( LJM2Param_Sizer, 0, wxALIGN_NOT, 1 );

  // LJM2 Run Info 
  wxBoxSizer *LJM2Run_Sizer = new wxBoxSizer( wxVERTICAL );

  wxBoxSizer *LJM2Run_A_Sizer = new wxBoxSizer( wxHORIZONTAL );
  LJM2Run_A_Sizer->Add( new wxButton(this, button_Run_LJM2_A, wxT("Run LJM2 A")), 0, wxALL, 1 );
  LJM2Run_A_Sizer->Add( new wxButton(this, button_Plot3DPaths_A, wxT("Plot 3D Paths A")), 0, wxALL, 1 );

  LJM2Run_Sizer->Add( LJM2Run_A_Sizer, 0, wxALL, 1 );

  wxBoxSizer *LJM2Run_B_Sizer = new wxBoxSizer( wxHORIZONTAL );
  LJM2Run_B_Sizer->Add( new wxButton(this, button_Run_LJM2_B, wxT("Run LJM2 B")), 0, wxALL, 1 );
  LJM2Run_B_Sizer->Add( new wxButton(this, button_Plot3DPaths_B, wxT("Plot 3D Paths B")), 0, wxALL, 1 );

  LJM2Run_Sizer->Add( LJM2Run_B_Sizer, 0, wxALL, 1 );

  // -- * --
  WSPlanner_BoxSizer->Add( LJM2Run_Sizer, 1, wxALIGN_NOT ); 
   
  // **----- Add to sizerFull -----**
  sizerFull_Planner->Add( WSPlanner_BoxSizer, 2, wxEXPAND | wxALL, 4 );
 
  // ** SAVE PATHS FOR IK **

  // Create StaticBox 
  wxStaticBox* WSSave_Box = new wxStaticBox(this, -1, wxT("Save a Path"));
  
  // Create sizer for this box with horizontal layout
  wxStaticBoxSizer* WSSave_BoxSizer = new wxStaticBoxSizer( WSSave_Box, wxVERTICAL ); 
  

  
  // Save LJM2Path
    wxBoxSizer *LJM2Save_Sizer = new wxBoxSizer( wxVERTICAL );

    //-- ** A **
    wxBoxSizer *LJM2Save_A_Sizer = new wxBoxSizer(wxHORIZONTAL);

    // -- Path Index A--
    wxBoxSizer *pathIndex_A_Sizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *pathIndex_A_Label = new wxStaticText( this, 1013, wxT("Path A:") );
    mPathIndex_A = new wxTextCtrl(this,1014,wxT("0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

    pathIndex_A_Sizer->Add( pathIndex_A_Label, 0, wxALL, 1 );
    pathIndex_A_Sizer->Add( mPathIndex_A, 0, wxALL, 1 );

    LJM2Save_A_Sizer->Add( pathIndex_A_Sizer, 0, wxALL, 1 );

    // -- Buttons A-- 
    LJM2Save_A_Sizer->Add( new wxButton(this, button_Plot3DPath_A, wxT("Plot")), 0, wxALL, 1 );
    LJM2Save_A_Sizer->Add( new wxButton(this, button_Save3DPath_A, wxT("Save")), 0, wxALL, 1 );
    
    LJM2Save_Sizer->Add( LJM2Save_A_Sizer, 0, wxALL, 1 );    

    //-- ** B  **
    wxBoxSizer *LJM2Save_B_Sizer = new wxBoxSizer(wxHORIZONTAL);

    // -- Path Index B --
    wxBoxSizer *pathIndex_B_Sizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *pathIndex_B_Label = new wxStaticText( this, 1013, wxT("Path B:") );
    mPathIndex_B = new wxTextCtrl(this,1014,wxT("0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

    pathIndex_B_Sizer->Add( pathIndex_B_Label, 0, wxALL, 1 );
    pathIndex_B_Sizer->Add( mPathIndex_B, 0, wxALL, 1 );

    LJM2Save_B_Sizer->Add( pathIndex_B_Sizer, 0, wxALL, 1 );

    // -- Buttons A-- 
    LJM2Save_B_Sizer->Add( new wxButton(this, button_Plot3DPath_B, wxT("Plot")), 0, wxALL, 1 );
    LJM2Save_B_Sizer->Add( new wxButton(this, button_Save3DPath_B, wxT("Save")), 0, wxALL, 1 );
    
    LJM2Save_Sizer->Add( LJM2Save_B_Sizer, 0, wxALL, 1 );    

    // --* -- 
    WSSave_BoxSizer->Add( LJM2Save_Sizer, 1, wxALIGN_NOT ); 

    // **----- Add to sizerFull -----**
    sizerFull_Planner->Add( WSSave_BoxSizer, 2, wxEXPAND | wxALL, 4 );
    
  // ** SET SIZER FULL **
  SetSizer(sizerFull_Planner);

}

/**
 * @function ~PlannerTab
 * @brief Destructor
 */ 
PlannerTab::~PlannerTab(){

	if( mCp_A != NULL ) {
		delete mCp_A;
	}

    if( mLjm2_A != NULL ) {
		delete mLjm2_A;
	}

	if( mCp_B != NULL ) {
		delete mCp_B;
	}

    if( mLjm2_B != NULL ) {
		delete mLjm2_B;
	}

}


/**
 * @function OnButton
 * @brief Handle Button Events
 */
void PlannerTab::OnButton(wxCommandEvent &_evt ) {

  int button_num = _evt.GetId();
  
  switch ( button_num ) {
    
    /** Get 3D Info A */
  case button_Get3DInfo_A:
    {
      printf("**********************\n");
      printf("      Get 3D Info     \n");
      printf("**********************\n");
      printf("--(!) Be sure to set start conf and target object or I will output rubbish \n");
      
      //-- Get data from Text	
      mSizeXText->GetValue().ToDouble( &gSizeX );
      mSizeYText->GetValue().ToDouble( &gSizeY );
      mSizeZText->GetValue().ToDouble( &gSizeZ );

      mOriginXText->GetValue().ToDouble( &gOriginX );
      mOriginYText->GetValue().ToDouble( &gOriginY );
      mOriginZText->GetValue().ToDouble( &gOriginZ );
      
      double temp;
      mPaddingText->GetValue().ToDouble( &temp );
      gPadding = (int) temp;
      mResolText->GetValue().ToDouble( &gResolution );
      
      //-- Create LJM2 Object
      mLjm2_A = new LJM2( gSizeX, gSizeY, gSizeZ, gOriginX, gOriginY, gOriginZ, gResolution );
      mCp_A = new CheckProcess( gSizeX, gSizeY, gSizeZ, gOriginX, gOriginY, gOriginZ, gResolution );
      mCp_A->getObjectsData( mWorld->mObjects, gTargetObjectName_A );  
      mCp_A->build_voxel( mWorld->mObjects, *mLjm2_A, gPadding ); // Here your LJM2 is built
      mCp_A->reportObjects(); 
      printf(" (i) Process Geometry (i) \n");
      mLjm2_A->ProcessGeometry();
      printf(" (i) End process geometry (i)  \n");
    }		
    break;
    
    /** Plot 3D Configuration A */
  case button_Plot3DInfo_A:
    {
      printf("*************************************\n");
      printf( "     Plotting 3D Configuration      \n");
      printf("*************************************\n");
      pcl::visualization::PCLVisualizer *viewer;
      viewer = new pcl::visualization::PCLVisualizer( "Discretized Workspace" );
      
      mLjm2_A->ViewObstacles( viewer, 0, 0, 255 );
      
      while( !viewer->wasStopped() ) {
	viewer->spin();
      }
      delete viewer;
    }		
    break;

    
    /** Get 3D Info B */
  case button_Get3DInfo_B:
    {
      printf("***********************\n");
      printf("      Get 3D Info B    \n");
      printf("***********************\n");
      printf("--(!) Be sure to set start conf and target object or I will output rubbish \n");
      
      //-- Get data from Text	
      mSizeXText->GetValue().ToDouble( &gSizeX );
      mSizeYText->GetValue().ToDouble( &gSizeY );
      mSizeZText->GetValue().ToDouble( &gSizeZ );
      
      mOriginXText->GetValue().ToDouble( &gOriginX );
      mOriginYText->GetValue().ToDouble( &gOriginY );
      mOriginZText->GetValue().ToDouble( &gOriginZ );

      double temp;
      mPaddingText->GetValue().ToDouble( &temp );
      gPadding = (int) temp;
      mResolText->GetValue().ToDouble( &gResolution );
      
      //-- Create LJM2 Object
      mLjm2_B = new LJM2( gSizeX, gSizeY, gSizeZ, gOriginX, gOriginY, gOriginZ, gResolution );
      mCp_B = new CheckProcess( gSizeX, gSizeY, gSizeZ, gOriginX, gOriginY, gOriginZ, gResolution );
      mCp_B->getObjectsData( mWorld->mObjects, gTargetObjectName_B );  
      mCp_B->build_voxel( mWorld->mObjects, *mLjm2_B, gPadding ); // Here your LJM2 is built
      mCp_B->reportObjects(); 
      printf(" (i) Process Geometry (i) \n");
      mLjm2_B->ProcessGeometry();
      printf(" (i) End process geometry (i)  \n");
    }		
    break;
    
    /** Plot 3D Configuration B */
  case button_Plot3DInfo_B:
    {
      printf("**************************************\n");
      printf( "     Plotting 3D Configuration B     \n");
      printf("**************************************\n");
      pcl::visualization::PCLVisualizer *viewer;
      viewer = new pcl::visualization::PCLVisualizer( "Discretized Workspace" );
      
      mLjm2_B->ViewObstacles( viewer, 0, 0, 255 );
      
      while( !viewer->wasStopped() ) {
	viewer->spin();
      }
      delete viewer;
    }		
    break;
    
    
    /** Run LJM2 A */
  case button_Run_LJM2_A: {

    //-- Update the parameters
    mAlphaText->GetValue().ToDouble( &mAlpha );
    double temp;
    mNumPathsText->GetValue().ToDouble( &temp );
    mNumPaths = (int)temp;
    //-- Execute
    WorkspacePlan_A();
  }
    break;
    
    /** Plot 3D Paths_A */
  case button_Plot3DPaths_A: {
    printf("*******************************\n");
    printf( "     Plotting 3D Paths A      \n");
    printf("*******************************\n");
    pcl::visualization::PCLVisualizer *viewer;
    viewer = new pcl::visualization::PCLVisualizer( "Workspace Paths A" );
    mLjm2_A->ViewObstacles( viewer, 0, 0, 255 );
    mLjm2_A->ViewPaths( mNodePaths_A, viewer );
    mLjm2_A->ViewBall( viewer, mStartNode_A(0), mStartNode_A(1), mStartNode_A(2), "Start" );
    mLjm2_A->ViewBall( viewer, mTargetNode_A(0), mTargetNode_A(1), mTargetNode_A(2), "Target" );
    
    while( !viewer->wasStopped() ) {
      viewer->spin();
    }
    delete viewer; 
  }
    break;
    
    /** Show Path A */
  case button_Plot3DPath_A: {     
    
    double temp;
    mPathIndex_A->GetValue().ToDouble(&temp);
    int n = (int) temp;
    
    printf("**********************************\n");
    printf( "     Plotting 3D Path A - %d       \n", n);
    printf("**********************************\n");
    pcl::visualization::PCLVisualizer *viewer;
    viewer = new pcl::visualization::PCLVisualizer( "Workspace Path A" );
    mLjm2_A->ViewObstacles( viewer, 0, 0, 255 );
    mLjm2_A->ViewPath( mNodePaths_A[n], viewer );
    mLjm2_A->ViewBall( viewer, mStartNode_A(0), mStartNode_A(1), mStartNode_A(2), "Start" );
    mLjm2_A->ViewBall( viewer, mTargetNode_A(0), mTargetNode_A(1), mTargetNode_A(2), "Target" );
    
    while( !viewer->wasStopped() ) {
      viewer->spin();
    }
    delete viewer; 
  }    
    break;
    
    /** Save3DPath: Save the WSPath into a global variable */
  case button_Save3DPath_A: {
    double temp;
    mPathIndex_A->GetValue().ToDouble(&temp);
    int n = (int) temp;
    gPosePath_A = mWorkspacePaths_A[n];  
    
    printf("--Saving path A - %d \n", n);
    for( size_t i = 0; i < gPosePath_A.size(); ++i ) {
      std::cout << gPosePath_A[i].transpose() << std::endl;
    }    
  }
    break;
    
    
    /** Plot 3D Paths_B */
  case button_Plot3DPaths_B: {
    printf("*******************************\n");
    printf( "     Plotting 3D Paths B      \n");
    printf("*******************************\n");
    pcl::visualization::PCLVisualizer *viewer;
    viewer = new pcl::visualization::PCLVisualizer( "Workspace Paths B" );
    mLjm2_B->ViewObstacles( viewer, 0, 0, 255 );
    mLjm2_B->ViewPaths( mNodePaths_A, viewer );
    mLjm2_B->ViewBall( viewer, mStartNode_B(0), mStartNode_B(1), mStartNode_B(2), "Start" );
    mLjm2_B->ViewBall( viewer, mTargetNode_B(0), mTargetNode_B(1), mTargetNode_B(2), "Target" );
    
    while( !viewer->wasStopped() ) {
      viewer->spin();
    }
    delete viewer; 
  }
    break;
    
    /** Show Path B */
  case button_Plot3DPath_B: {     
    
    double temp;
    mPathIndex_B->GetValue().ToDouble(&temp);
    int n = (int) temp;
    
    printf("**********************************\n");
    printf( "     Plotting 3D Path B - %d       \n", n);
    printf("**********************************\n");
    pcl::visualization::PCLVisualizer *viewer;
    viewer = new pcl::visualization::PCLVisualizer( "Workspace Path B" );
    mLjm2_B->ViewObstacles( viewer, 0, 0, 255 );
    mLjm2_B->ViewPath( mNodePaths_B[n], viewer );
    mLjm2_B->ViewBall( viewer, mStartNode_B(0), mStartNode_B(1), mStartNode_B(2), "Start" );
    mLjm2_B->ViewBall( viewer, mTargetNode_B(0), mTargetNode_B(1), mTargetNode_B(2), "Target" );
    
    while( !viewer->wasStopped() ) {
      viewer->spin();
    }
    delete viewer; 
  }    
    break;
    
    /** Run LJM2 B */
  case button_Run_LJM2_B: {

    //-- Update the parameters
    mAlphaText->GetValue().ToDouble( &mAlpha );
    double temp;
    mNumPathsText->GetValue().ToDouble( &temp );
    mNumPaths = (int)temp;
    //-- Execute
    WorkspacePlan_B();
  }
    break;
    
    /** Save3DPath: Save the WSPath into a global variable */
  case button_Save3DPath_B: {
    double temp;
    mPathIndex_B->GetValue().ToDouble(&temp);
    int n = (int) temp;
    gPosePath_B = mWorkspacePaths_B[n];  
    
    printf("--Saving path B - %d \n", n);
    for( size_t i = 0; i < gPosePath_B.size(); ++i ) {
      std::cout << gPosePath_B[i].transpose() << std::endl;
    }    
    }
    break;
    

  } // end of switch
  
}

/**
 * @function WorkspacePlan_A
 */ 
void PlannerTab::WorkspacePlan_A() {
 
    /// Check start position is not in collision
    mWorld->mRobots[gRobotId]->setDofs( gStartConf_A, gLinks_A );

    if( mCollision->CheckCollisions() ) {   
      printf(" --(!) Initial status is in collision. I am NOT proceeding. Exiting \n");
      return; 
    }

   //-- Setting start cell
   gStartPos_A = GetEE_Pos( gStartConf_A );

   mStartNode_A.resize(3);
   mTargetNode_A.resize(3);

   if( mLjm2_A->WorldToGrid( gStartPos_A(0), gStartPos_A(1), gStartPos_A(2), mStartNode_A(0), mStartNode_A(1), mStartNode_A(2) ) == false )
   {  printf("(x) Error: Start Position no valid  (x)\n"); return; } 
   if( mLjm2_A->WorldToGrid( gTargetPos_A(0), gTargetPos_A(1), gTargetPos_A(2), mTargetNode_A(0), mTargetNode_A(1), mTargetNode_A(2) ) == false )
   {  printf("(x) Error: Target Position no valid  (x)\n"); return; }    


   //-- Plan now workspace guys
   printf("(o) Planning from (%d %d %d) to (%d %d %d) (o)\n", mStartNode_A(0), mStartNode_A(1), mStartNode_A(2), mTargetNode_A(0), mTargetNode_A(1), mTargetNode_A(2) );
   printf("(o) Start State: %d  Target state: %d (FREE: 1 OBSTACLE: 2) (o)\n", mLjm2_A->GetState(mStartNode_A(0), mStartNode_A(1), mStartNode_A(2)), mLjm2_A->GetState(mTargetNode_A(0), mTargetNode_A(1), mTargetNode_A(2)));

   //-- Check alpha and numPaths
   printf("---(i) Search Parameters: Alpha: %.3f - Num Paths: %d \n", mAlpha, mNumPaths );
   
   mNodePaths_A = mLjm2_A->FindVarietyPaths2( mStartNode_A(0), mStartNode_A(1), mStartNode_A(2), mTargetNode_A(0), mTargetNode_A(1), mTargetNode_A(2), mNumPaths, mAlpha );
   mWorkspacePaths_A = mLjm2_A->NodePathToWorkspacePath( mNodePaths_A );
   printf("-------(i) Finished Workpace Planning A (i)------- \n");
    
} 

/**
 * @function WorkspacePlan_B
 */ 
void PlannerTab::WorkspacePlan_B() {
 
    /// Check start position is not in collision
    mWorld->mRobots[gRobotId]->setDofs( gStartConf_B, gLinks_B );

    if( mCollision->CheckCollisions() ) {   
      printf(" --(!) Initial status is in collision. I am NOT proceeding. Exiting \n");
      return; 
    }

   //-- Setting start cell
   gStartPos_B = GetEE_Pos( gStartConf_B, ARM_B );

   mStartNode_B.resize(3);
   mTargetNode_B.resize(3);

   if( mLjm2_B->WorldToGrid( gStartPos_B(0), gStartPos_B(1), gStartPos_B(2), mStartNode_B(0), mStartNode_B(1), mStartNode_B(2) ) == false )
   {  printf("(x) Error: Start Position no valid  (x)\n"); return; } 
   if( mLjm2_B->WorldToGrid( gTargetPos_B(0), gTargetPos_B(1), gTargetPos_B(2), mTargetNode_B(0), mTargetNode_B(1), mTargetNode_B(2) ) == false )
   {  printf("(x) Error: Target Position no valid  (x)\n"); return; }    


   //-- Plan now workspace guys
   printf("(o) Planning from (%d %d %d) to (%d %d %d) (o)\n", mStartNode_B(0), mStartNode_B(1), mStartNode_B(2), mTargetNode_B(0), mTargetNode_B(1), mTargetNode_B(2) );
   printf("(o) Start State: %d  Target state: %d (FREE: 1 OBSTACLE: 2) (o)\n", mLjm2_B->GetState(mStartNode_B(0), mStartNode_B(1), mStartNode_B(2)), mLjm2_B->GetState(mTargetNode_B(0), mTargetNode_B(1), mTargetNode_B(2)));

   //-- Check alpha and numPaths
   printf("---(i) Search Parameters: Alpha: %.3f - Num Paths: %d \n", mAlpha, mNumPaths );
   
   mNodePaths_B = mLjm2_B->FindVarietyPaths2( mStartNode_B(0), mStartNode_B(1), mStartNode_B(2), mTargetNode_B(0), mTargetNode_B(1), mTargetNode_B(2), mNumPaths, mAlpha );
   mWorkspacePaths_A = mLjm2_A->NodePathToWorkspacePath( mNodePaths_A );
   printf("-------(i) Finished Workpace Planning B (i)------- \n");
    
} 


/**
 * @function OnSlider
 * @brief Handle slider changes
 */
void PlannerTab::OnSlider(wxCommandEvent &evt) {

    int slnum = evt.GetId();
    double pos = *(double*) evt.GetClientData();
 
    return;
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
