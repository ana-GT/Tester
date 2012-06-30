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

#include "ConfigTab.h"

using namespace std;

// Control IDs (used for event handling - be sure to start with a non-conflicted id)

enum ConfigTabEvents {
  button_SetStart_A = 50,
  button_ShowStart_A,
  button_SetTarget_A,
  button_ShowTarget_A,

  button_SetStart_B,
  button_ShowStart_B,
  button_SetTarget_B,
  button_ShowTarget_B
};



//-- sizer for ConfigTab
wxBoxSizer* SuperSizer;

//-- Add a handler for any events that can be generated by the widgets you add here (sliders, radio, checkbox, etc)
BEGIN_EVENT_TABLE( ConfigTab, wxPanel )
EVT_COMMAND ( wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, ConfigTab::OnSlider ) // ALWAYS PUT OR THERE WILL BE A LINKER ERROR
EVT_COMMAND ( wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, ConfigTab::OnButton )
EVT_COMMAND ( wxID_ANY, wxEVT_COMMAND_RADIOBOX_SELECTED, ConfigTab::OnRadio )
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS( ConfigTab, GRIPTab )

/**
 * @function OnSlider
 * @brief Function that MUST be to avoid linker errors 
 */
void ConfigTab::OnSlider(wxCommandEvent &evt) {
}

/**
 * @function ConfigTab
 * @brief Constructor
 */
ConfigTab::ConfigTab( wxWindow *_parent, const wxWindowID _id,
		      const wxPoint& _pos, const wxSize& _size, long _style) :
GRIPTab( _parent, _id, _pos, _size, _style ) {

  // **** SET SUPERSIZER ****
  SuperSizer = new wxBoxSizer( wxHORIZONTAL );

  // ****  RESET VALUES ****
  gStartConf_A.resize(0);
  gStartPos_A.resize(0);

  gTargetPos_A.resize(3);
  gTargetPose_A.resize(6);
  
  gRobotId = 0;
  gLinks_A.resize(0);
 
 
  // **** SET ROBOT TYPE ****
  wxStaticBox* RobotType_Box = new wxStaticBox( this, -1, wxT("Robot Type") );
  wxStaticBoxSizer* RobotType_BoxSizer = new wxStaticBoxSizer( RobotType_Box, wxVERTICAL );
   
  RobotType_BoxSizer->Add( new wxRadioBox( this, wxID_ANY, wxT("Robot:"),
					   wxDefaultPosition, wxDefaultSize,
					   gNumRobotTypes, gWxRobotNames, 1,
					   wxRA_SPECIFY_COLS),
			   1,
			   wxALIGN_NOT,
			   0 );

  SuperSizer->Add( RobotType_BoxSizer, 2, wxEXPAND | wxALL, 5 );


  // **** SET ARMS A AND B ****
  wxStaticBox* setArm_AB_Box = new wxStaticBox( this, -1, wxT("Set Arms") );
  wxStaticBoxSizer* setArm_AB_BoxSizer = new wxStaticBoxSizer( setArm_AB_Box, wxHORIZONTAL );

  // ** Start and Target A  **
  wxBoxSizer *setArm_A_Sizer = new wxBoxSizer( wxVERTICAL );

  // Put buttons in
  setArm_A_Sizer->Add( new wxButton(this, button_SetStart_A, wxT("Set Start A")),
			  0, wxALL, 1 ); 
  setArm_A_Sizer->Add( new wxButton(this, button_ShowStart_A, wxT("Show Start A")),
			  0, wxALL, 1 ); 
  setArm_A_Sizer->Add( new wxButton(this, button_SetTarget_A, wxT("Set Target A")),
			  0, wxALL, 1 ); 
  setArm_A_Sizer->Add( new wxButton(this, button_ShowTarget_A, wxT("Show Target A")),
			  0, wxALL, 1 ); 
  
  setArm_AB_BoxSizer->Add( setArm_A_Sizer, 0, wxALL, 1 );

  // ** Start and Target B  **
  wxBoxSizer *setArm_B_Sizer = new wxBoxSizer( wxVERTICAL );

  // Put buttons in
  setArm_B_Sizer->Add( new wxButton(this, button_SetStart_B, wxT("Set Start B")),
			  0, wxALL, 1 ); 
  setArm_B_Sizer->Add( new wxButton(this, button_ShowStart_B, wxT("Show Start B")),
			  0, wxALL, 1 ); 
  setArm_B_Sizer->Add( new wxButton(this, button_SetTarget_B, wxT("Set Target B")),
			  0, wxALL, 1 ); 
  setArm_B_Sizer->Add( new wxButton(this, button_ShowTarget_B, wxT("Show Target B")),
			  0, wxALL, 1 ); 
  
  setArm_AB_BoxSizer->Add( setArm_B_Sizer, 0, wxALL, 1 );

  // **----- Add to sizerFull -----**
  SuperSizer->Add( setArm_AB_BoxSizer, 2, wxEXPAND | wxALL, 5 ); 

  /*
  // ** SET TARGET POSE **
  wxStaticBox* setPoseBox = new wxStaticBox(this, -1, wxT("Set Pose"));

  // Create sizer for StaticBox above
  wxStaticBoxSizer* setPoseBoxSizer = new wxStaticBoxSizer( setPoseBox, wxHORIZONTAL );
  wxBoxSizer *PoseBoxSizer = new wxBoxSizer( wxHORIZONTAL );	
  
  // ** Create sizer for Target Position inputs **
  wxBoxSizer *PosInputSizer = new wxBoxSizer(wxVERTICAL);
  
  // x
  wxBoxSizer *posX_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *posX_Label = new wxStaticText( this, 1007, wxT("x: ") );
  mTargetX_Text = new wxTextCtrl(this,1008,wxT("0.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);
  
  posX_Sizer->Add( posX_Label, 0, wxALL, 1 );
  posX_Sizer->Add( mTargetX_Text, 0, wxALL, 1 );
  
  PosInputSizer->Add( posX_Sizer, 0, wxALL, 1 );

  // y
  wxBoxSizer *posY_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *posY_Label = new wxStaticText( this, 1007, wxT("y: ") );
  mTargetY_Text = new wxTextCtrl(this,1008,wxT("0.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);
  
  posY_Sizer->Add( posY_Label, 0, wxALL, 1 );
  posY_Sizer->Add( mTargetY_Text, 0, wxALL, 1 );
  
  PosInputSizer->Add( posY_Sizer, 0, wxALL, 1 );

  // z
  wxBoxSizer *posZ_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *posZ_Label = new wxStaticText( this, 1007, wxT("z: ") );
  mTargetZ_Text = new wxTextCtrl(this,1008,wxT("0.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);
  
  posZ_Sizer->Add( posZ_Label, 0, wxALL, 1 );
  posZ_Sizer->Add( mTargetZ_Text, 0, wxALL, 1 );
  
  PosInputSizer->Add( posZ_Sizer, 0, wxALL, 1 );

  // Create sizer for start buttons in 1st column
  wxBoxSizer *OrientInputSizer = new wxBoxSizer(wxVERTICAL);

  // roll
  wxBoxSizer *oriRoll_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *oriRoll_Label = new wxStaticText( this, 1007, wxT("roll: ") );
  mTargetRoll_Text = new wxTextCtrl( this,1008,wxT("0.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT );
  
  oriRoll_Sizer->Add( oriRoll_Label, 0, wxALL, 1 );
  oriRoll_Sizer->Add( mTargetRoll_Text, 0, wxALL, 1 );
  
  OrientInputSizer->Add( oriRoll_Sizer, 0, wxALL, 1 );

  // pitch
  wxBoxSizer *oriPitch_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *oriPitch_Label = new wxStaticText( this, 1007, wxT("pitch: ") );
  mTargetPitch_Text = new wxTextCtrl(this,1008,wxT("0.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);
  
  oriPitch_Sizer->Add( oriPitch_Label, 0, wxALL, 1 );
  oriPitch_Sizer->Add( mTargetPitch_Text, 0, wxALL, 1 );
  
  OrientInputSizer->Add( oriPitch_Sizer, 0, wxALL, 1 );

  // yaw
  wxBoxSizer *oriYaw_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *oriYaw_Label = new wxStaticText( this, 1007, wxT("yaw: ") );
  mTargetYaw_Text = new wxTextCtrl(this,1008,wxT("0.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);
  
  oriYaw_Sizer->Add( oriYaw_Label, 0, wxALL, 1 );
  oriYaw_Sizer->Add( mTargetYaw_Text, 0, wxALL, 1 );
  
  OrientInputSizer->Add( oriYaw_Sizer, 0, wxALL, 1 );


  // ******    

  PoseBoxSizer->Add( PosInputSizer, 1, wxALL, 2 ); 
  PoseBoxSizer->Add( OrientInputSizer, 1, wxALL, 2 );

  setPoseBoxSizer->Add( PoseBoxSizer, 2, wxALL, 3 ); 

   wxBoxSizer *PoseButtonsSizer = new wxBoxSizer( wxVERTICAL );
   
  PoseButtonsSizer->Add( new wxButton(this, button_SetTargetPose, wxT("Set Goal Pose")),
			0, wxALL, 1 ); 
  PoseButtonsSizer->Add( new wxButton(this, button_SetTarget2ObjectPose, wxT("Set Target to Object")),
			0, wxALL, 1 ); 

  PoseButtonsSizer->Add( new wxButton(this, button_SetEE, wxT("Set EE")),
		       0, wxALL, 1 ); 
  
  setPoseBoxSizer->Add( PoseButtonsSizer, 1, wxALL, 3 );

  // ** Add to sizerFull **
  sizerFull->Add( setPoseBoxSizer, 2, wxEXPAND | wxALL, 5 );


  // ** Create StaticBox to set Configurations **
  wxStaticBox* setAddBox = new wxStaticBox(this, -1, wxT("Additional"));


	/** Size Sizer */
  /*
    wxBoxSizer *sizeSizer = new wxBoxSizer(wxHORIZONTAL);

    // x
    wxBoxSizer *sizeXSizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *sizeXLabel = new wxStaticText( this, 1007, wxT("Size x:") );
    mSizeXText = new wxTextCtrl(this,1008,wxT("1.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

    sizeXSizer->Add( sizeXLabel, 0, wxALL, 1 );
    sizeXSizer->Add( mSizeXText, 0, wxALL, 1 );

    sizeSizer->Add( sizeXSizer, 0, wxALL, 1 );
    // y
    wxBoxSizer *sizeYSizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *sizeYLabel = new wxStaticText( this, 1009, wxT("y:") );
    mSizeYText = new wxTextCtrl(this,1010,wxT("1.0"), wxDefaultPosition,wxSize(40,20),wxTE_RIGHT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

    sizeYSizer->Add( sizeYLabel, 0, wxALL, 1 );
    sizeYSizer->Add( mSizeYText, 0, wxALL, 1 );

    sizeSizer->Add( sizeYSizer, 0, wxALL, 1 );

    // z
    wxBoxSizer *sizeZSizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *sizeZLabel = new wxStaticText( this, 1011, wxT("z:") );
    mSizeZText = new wxTextCtrl(this,1012,wxT("1.0"), wxDefaultPosition,wxSize(40,20),wxTE_RIGHT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

    sizeZSizer->Add( sizeZLabel, 0, wxALL, 1 );
    sizeZSizer->Add( mSizeZText, 0, wxALL, 1 );

    sizeSizer->Add( sizeZSizer, 0, wxALL, 1 );

    setAddBoxSizer->Add( sizeSizer, 0, wxALIGN_NOT, 1 );

  */
	/** Origin Sizer */
  // wxBoxSizer *originSizer = new wxBoxSizer(wxHORIZONTAL);
    /*
    // x
    wxBoxSizer *originXSizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *originXLabel = new wxStaticText( this, 1013, wxT("Orig x:") );
    mOriginXText = new wxTextCtrl(this,1014,wxT("-0.1"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

    originXSizer->Add( originXLabel, 0, wxALL, 1 );
    originXSizer->Add( mOriginXText, 0, wxALL, 1 );

    originSizer->Add( originXSizer, 0, wxALL, 1 );

    // y
    wxBoxSizer *originYSizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *originYLabel = new wxStaticText( this, 1015, wxT(" y:") );
    mOriginYText = new wxTextCtrl(this,1016,wxT("-0.2"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

    originYSizer->Add( originYLabel, 0, wxALL, 1 );
    originYSizer->Add( mOriginYText, 0, wxALL, 1 );

    originSizer->Add( originYSizer, 0, wxALL, 1 );

    // z
    wxBoxSizer *originZSizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *originZLabel = new wxStaticText( this, 1017, wxT(" z:") );
    mOriginZText = new wxTextCtrl(this,1018,wxT("0.05"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

    originZSizer->Add( originZLabel, 0, wxALL, 1 );
    originZSizer->Add( mOriginZText, 0, wxALL, 1 );

    originSizer->Add( originZSizer, 0, wxALL, 1 );


    setAddBoxSizer->Add( originSizer, 0, wxALIGN_NOT, 1 );


	/** Detail Sizer */
  //   wxBoxSizer *detailSizer = new wxBoxSizer(wxHORIZONTAL);

    // resolution
  /*    wxBoxSizer *resolSizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *resolLabel = new wxStaticText( this, 1019, wxT("Res:") );
    mResolText = new wxTextCtrl(this,1020,wxT("0.02"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

    resolSizer->Add( resolLabel, 0, wxALL, 1 );
    resolSizer->Add( mResolText, 0, wxALL, 1 );

    detailSizer->Add( resolSizer, 0, wxALL, 1 );

    // inflated
    wxBoxSizer *paddingSizer = new wxBoxSizer(wxHORIZONTAL);
    wxStaticText *paddingLabel = new wxStaticText( this, 1021, wxT(" Pad:") );
    mPaddingText = new wxTextCtrl(this,1022,wxT("1"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

    paddingSizer->Add( paddingLabel, 0, wxALL, 1 );
    paddingSizer->Add( mPaddingText, 0, wxALL, 1 );

    detailSizer->Add( paddingSizer, 0, wxALL, 1 );

    detailSizer->Add( new wxButton(this, button_SetAdd, wxT("Add")),
		       0, wxALL, 1 ); 

    setAddBoxSizer->Add( detailSizer, 0, wxALIGN_NOT, 1 );


  */

  SetSizer( SuperSizer);

}

/**
 * @function ~ConfigTab
 * @brief Destructor
 */ 
ConfigTab::~ConfigTab(){
    
}

/**
 * @function GetLinksId
 * @brief Get DOF's IDs for Manipulator
 */
void ConfigTab::GetLinksId() {

  Eigen::VectorXi linksAll = mWorld->mRobots[gRobotId]->getQuickDofsIndices();  

  //-- **** LWA4 ****
  if( gRobotName.compare( gRobotNames[0] ) == 0 ) {
    printf(" LWA4!\n");
  }
  //-- **** LWA3 ****
  else if( gRobotName.compare( gRobotNames[1] ) == 0 ) {
    gNumLinks_A = sNum_LA_Links_LWA3; 
    gLinks_A.resize( gNumLinks_A );

    for( unsigned int i = 0; i < gNumLinks_A; i++ ) {
      for( unsigned int j = 0; j < linksAll.size(); j++ ) {      
	if( strcmp( mWorld->mRobots[gRobotId]->getDof( linksAll[j] )->getJoint()->getChildNode()->getName(),
		    sLA_Ids_LWA3[i] ) == 0 ) {
	  gLinks_A[i] = linksAll[j]; 
	  break;   
	}
      }
    } // end for
   
  }

  //-- **** BARRET ARM ****
  else if( gRobotName.compare( gRobotNames[2] ) == 0 ) {
    gNumLinks_A = sNum_LA_Links_Barret;
    gLinks_A.resize( gNumLinks_A );

    for( unsigned int i = 0; i < gNumLinks_A; i++ ) {
      for( unsigned int j = 0; j < linksAll.size(); j++ ) {      
	if( strcmp( mWorld->mRobots[gRobotId]->getDof( linksAll[j] )->getJoint()->getChildNode()->getName(), 
		    sLA_Ids_Barret[i] ) == 0 ) {
	  gLinks_A[i] = linksAll[j]; 
	  break;   
	}
      }
    } // end for
  }

  std::cout << "gLinks_A: " << gLinks_A.transpose() << std::endl;
}


/**
 * @function OnButton
 * @brief Handle Button Events
 */
void ConfigTab::OnButton( wxCommandEvent &_evt ) {

  int button_num = _evt.GetId();
  GetLinksId();

  switch (button_num) {

    // **** Set Start A ****
  case button_SetStart_A: {
    
    if ( mWorld != NULL ) {
      if( mWorld->mRobots.size() < 1) {
	cout << "--(!) Must have a world with a robot to set a Start state (!)--" << endl;
	break;
      }
      std::cout << "--(i) Setting Start state for " << gRobotName << ":" << std::endl;
      
      gStartConf_A = mWorld->mRobots[gRobotId]->getDofs( gLinks_A );
      
      for( unsigned int i = 0; i < gStartConf_A.size(); i++ )
	{  std::cout << gStartConf_A(i) << " ";  } 
      std::cout << endl;
    } else {
      std::cout << "--(!) Must have a world loaded to set a Start state.(!)--" << std::endl;
    }
  }
    break;
    
    // **** Show Start_A ****
  case button_ShowStart_A: {
    
    if( gStartConf_A.size() < 1 ) {
      std::cout << "--(!) First, set a start config A (!)--" << std::endl;
      break;
    } 
    
    mWorld->mRobots[gRobotId]->setDofs( gStartConf_A, gLinks_A );
    
    for( unsigned int i = 0; i< gStartConf_A.size(); i++ )
      {  std::cout << gStartConf_A(i) << " "; }
    std::cout << std::endl;
    
    mWorld->mRobots[gRobotId]->update();
    viewer->UpdateCamera();
  }
    break;

    // **** Set Target A ****
  case button_SetTarget_A: {
    
    if( mWorld != NULL ) {  
      if( mWorld->mRobots.size() < 1 )
	{  printf("---------(xx) No robot in the loaded world, you idiot, I need one! (xx)---------- \n"); break; }
      
      if( selectedObject != NULL )
	{ 
	  gTargetObjectName_A = selectedObject->getName();
	  double x; double y; double z;
	  selectedObject->getPositionXYZ( x, y, z );
	  gTargetPos_A << x, y, z;
	  gTargetPose_A << x, y, z, 0, 0, 0;
	  std::cout<<"** Target object A: "<< gTargetObjectName_A << ": " << gTargetPos_A.transpose() << std::endl;
	}
      else
	{ std::cout<< "------xx (!) Please, select an object in the Viewer Tree and try again xx------"<< std::endl; }                    
          }
    else
      { std::cout<< "------xx (!) No world loaded, I cannot set a goal xx------"<< std::endl; }
  }
    break;


    // **** Set Start B ****
  case button_SetStart_B: {
    
    if ( mWorld != NULL ) {
      if( mWorld->mRobots.size() < 1) {
	cout << "--(!) Must have a world with a robot to set a Start state (!)--" << endl;
	break;
      }
      std::cout << "--(i) Setting Start state for " << gRobotName << ":" << std::endl;
      
      gStartConf_B = mWorld->mRobots[gRobotId]->getDofs( gLinks_B );
      
      for( unsigned int i = 0; i < gStartConf_B.size(); i++ )
	{  std::cout << gStartConf_B(i) << " ";  } 
      std::cout << std::endl;
    } else {
      std::cout << "--(!) Must have a world loaded to set a Start state.(!)--" << std::endl;
    }
  }
    break;
    
    // **** Show Start_B ****
  case button_ShowStart_B: {
    
    if( gStartConf_B.size() < 1 ) {
      std::cout << "--(!) First, set a start config B (!)--" << std::endl;
      break;
    } 
    
    mWorld->mRobots[gRobotId]->setDofs( gStartConf_B, gLinks_B );
    
    for( unsigned int i = 0; i< gStartConf_B.size(); i++ )
      {  std::cout << gStartConf_B(i) << " "; }
    std::cout << std::endl;
    
    mWorld->mRobots[gRobotId]->update();
    viewer->UpdateCamera();
  }
    break;

    // **** Set Target B ****
  case button_SetTarget_B: {
    
    if( mWorld != NULL ) {  
      if( mWorld->mRobots.size() < 1 )
	{  printf("---------(xx) No robot in the loaded world, you idiot, I need one! (xx)---------- \n"); break; }
      
      if( selectedObject != NULL )
	{ 
	  gTargetObjectName_B = selectedObject->getName();
	  double x; double y; double z;
	  selectedObject->getPositionXYZ( x, y, z );
	  gTargetPos_B << x, y, z;
	  gTargetPose_B << x, y, z, 0, 0, 0;
	  std::cout<<"** Target object B: "<< gTargetObjectName_B << ": " << gTargetPos_B.transpose() << std::endl;
	}
      else
	{ cout<<"------xx (!) Please, select an object in the Viewer Tree and try again xx------"<<endl; }                    
          }
    else
      { cout<<"------xx (!) No world loaded, I cannot set a goal xx------"<<endl; }
  }
    break;

    

  }  // End of switch
}


/**
 * @function OnRadio
 * @brief Choose name of Robot
 */
void ConfigTab::OnRadio( wxCommandEvent &_evt ) {
  
  int num = _evt.GetSelection();
  
  if( num < gNumRobotTypes ) { 
    gRobotName = gRobotNames[num];
    std::cout << "--> Robot name: " << gRobotName << std::endl; 
  } 
  else {
    std::cout << "--(!) No robot name set (!)--" << std::endl;
  }

}

/**
 * @function GRIPStateChange -- Keep using this name as it is a virtual function
 * @brief This function is called when an object is selected in the Tree View or other
 *        global changes to the RST world. Use this to capture events from outside the tab.
 */
void ConfigTab::GRIPStateChange() {
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
  SuperSizer->Layout();
}
