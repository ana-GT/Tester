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
  button_ShowTarget_B,
  button_SetVideoParam
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

  gStartConf_B.resize(0);
  gStartPos_B.resize(0);

  gTargetPos_B.resize(3);
  gTargetPose_B.resize(6);

  
  gRobotId = 0;
  gLinks_A.resize(0);
  gLinks_B.resize(0);
 
 
  // **** SET ROBOT TYPE ****
  wxStaticBox* RobotType_Box = new wxStaticBox( this, -1, wxT("Robot Type") );
  wxStaticBoxSizer* RobotType_BoxSizer = new wxStaticBoxSizer( RobotType_Box, wxVERTICAL );
   
  RobotType_BoxSizer->Add( new wxRadioBox( this, wxID_ANY, wxT("Robot:"),
					   wxDefaultPosition, wxDefaultSize,
					   gNumRobotTypes, gWxRobotNames, 1,
					   wxRA_SPECIFY_ROWS), // COLS / ROWS
			   1,
			   wxALIGN_NOT,
			   0 );

  SuperSizer->Add( RobotType_BoxSizer, 3, wxEXPAND | wxALL, 6 );


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
  SuperSizer->Add( setArm_AB_BoxSizer, 2, wxEXPAND | wxALL, 6 ); 


  // ** VIDEO CONFIGURATION**
 
  // Create StaticBox container for all items
  wxStaticBox* Video_Box = new wxStaticBox(this, -1, wxT("Video Conf"));
  
  // Create sizer for this box with horizontal layout
  wxStaticBoxSizer* Video_BoxSizer = new wxStaticBoxSizer( Video_Box, wxVERTICAL );

  // Video Parameters
  wxBoxSizer *VideoParam_Sizer = new wxBoxSizer(wxVERTICAL);
  
  // -- FPS --
  wxBoxSizer *FPS_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *FPS_Label = new wxStaticText( this, 1013, wxT(" FPS: ") );
  mFPSText = new wxTextCtrl(this,1014,wxT("20"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
  
  FPS_Sizer->Add( FPS_Label, 0, wxALL, 1 );
  FPS_Sizer->Add( mFPSText, 0, wxALL, 1 );
  
  VideoParam_Sizer->Add( FPS_Sizer, 0, wxALL, 1 );
  
  // -- Video Time --
  wxBoxSizer *VideoTime_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *VideoTime_Label = new wxStaticText( this, 1015, wxT(" Time: ") );
  mVideoTimeText = new wxTextCtrl(this,1016,wxT("5"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

  VideoTime_Sizer->Add( VideoTime_Label, 0, wxALL, 1 );
  VideoTime_Sizer->Add( mVideoTimeText, 0, wxALL, 1 );
  
  VideoParam_Sizer->Add( VideoTime_Sizer, 0, wxALL, 1 );

  // -- * --
  Video_BoxSizer->Add( VideoParam_Sizer, 0, wxALIGN_NOT, 1 );

  // -- Video Button -- 
  wxBoxSizer *VideoButton_Sizer = new wxBoxSizer( wxHORIZONTAL );
  VideoButton_Sizer->Add( new wxButton(this, button_SetVideoParam, wxT("Set")), 0, wxALL, 1 );

  // -- * --
  Video_BoxSizer->Add( VideoButton_Sizer, 1, wxALIGN_NOT ); 


  // **----- Add to sizerFull -----**
  SuperSizer->Add( Video_BoxSizer, 1, wxEXPAND | wxALL, 6 );

  // **** Set SuperSizer ****
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

  //-- **** MITSUBISHI ****
  if( gRobotName.compare( gRobotNames[0] ) == 0 ) {
    gNumLinks_A = sNum_LA_Links_Mitsubishi; 
    gLinks_A.resize( gNumLinks_A );

    for( unsigned int i = 0; i < gNumLinks_A; i++ ) {
      for( unsigned int j = 0; j < linksAll.size(); j++ ) {      
	if( strcmp( mWorld->mRobots[gRobotId]->getDof( linksAll[j] )->getJoint()->getChildNode()->getName(),
		    sLA_Ids_Mitsubishi[i] ) == 0 ) {
	  gLinks_A[i] = linksAll[j]; 
	  break;   
	}
      }
    } // end for
   
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

  //-- **** KATANA ARM ****
  else if( gRobotName.compare( gRobotNames[3] ) == 0 ) {
    gNumLinks_A = sNum_LA_Links_Katana;
    gLinks_A.resize( gNumLinks_A );

    for( unsigned int i = 0; i < gNumLinks_A; i++ ) {
      for( unsigned int j = 0; j < linksAll.size(); j++ ) {      
	if( strcmp( mWorld->mRobots[gRobotId]->getDof( linksAll[j] )->getJoint()->getChildNode()->getName(), 
		    sLA_Ids_Katana[i] ) == 0 ) {
	  gLinks_A[i] = linksAll[j]; 
	  break;   
	}
      }
    } // end for
  }

  //-- **** Snake ARM ****
  else if( gRobotName.compare( gRobotNames[4] ) == 0 ) {
    gNumLinks_A = sNum_LA_Links_Snake;
    gLinks_A.resize( gNumLinks_A );

    for( unsigned int i = 0; i < gNumLinks_A; i++ ) {
      for( unsigned int j = 0; j < linksAll.size(); j++ ) {      
	if( strcmp( mWorld->mRobots[gRobotId]->getDof( linksAll[j] )->getJoint()->getChildNode()->getName(), 
		    sLA_Ids_Snake[i] ) == 0 ) {
	  gLinks_A[i] = linksAll[j]; 
	  break;   
	}
      }
    } // end for
  }


  //-- **** LWA4 ARM DUAL ****
  else if( gRobotName.compare( gRobotNames[5] ) == 0 ) {
    gNumLinks_A = sNum_LA_Links_LWA4;
    gLinks_A.resize( gNumLinks_A );

    for( unsigned int i = 0; i < gNumLinks_A; i++ ) {
      for( unsigned int j = 0; j < linksAll.size(); j++ ) {      
	if( strcmp( mWorld->mRobots[gRobotId]->getDof( linksAll[j] )->getJoint()->getChildNode()->getName(), 
		    sLA_Ids_LWA4[i] ) == 0 ) {
	  gLinks_A[i] = linksAll[j]; 
	  break;   
	}
      }
    } // end for

	gNumLinks_B = sNum_RA_Links_LWA4;
    gLinks_B.resize( gNumLinks_B );
    for( unsigned int i = 0; i < gNumLinks_B; i++ ) {
      for( unsigned int j = 0; j < linksAll.size(); j++ ) {      
	if( strcmp( mWorld->mRobots[gRobotId]->getDof( linksAll[j] )->getJoint()->getChildNode()->getName(), 
		    sRA_Ids_LWA4[i] ) == 0 ) {
	  gLinks_B[i] = linksAll[j]; 
	  break;   
	}
      }
    } // end for

  }

  std::cout << "gLinks_A: " << gLinks_A.transpose() << std::endl;
	if( gNumLinks_B > 0 ) {
  std::cout << "gLinks_B: " << gLinks_B.transpose() << std::endl;
	}
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

  /** Set Video parameters */
  case button_SetVideoParam: {
      //-- Get data from Text	
      mFPSText->GetValue().ToDouble( &gFPS );
      mVideoTimeText->GetValue().ToDouble( &gVideoTime );

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
  
  //-- Set Robot to use
  if( num < gNumRobotTypes ) { 

    // Set robot name
    gRobotName = gRobotNames[num];
    // Set EE for arm A
    gEEName_A = gEEId_A_Names[num];
    gEENode_A = mWorld->mRobots[gRobotId]->getNode( gEEName_A.c_str() );
    gEEId_A = gEENode_A->getSkelIndex();
    // Display Info
    std::cout << "--> Robot set: " << gRobotName << std::endl;
    std::cout << "--> [A] End Effector: " << gEEName_A << " ID: " << gEEId_A << std::endl;

	// If LWA4
	if( num == 5 ) {
    // Set EE for arm B
    gEEName_B = gEEId_B_Names[num];
    gEENode_B = mWorld->mRobots[gRobotId]->getNode( gEEName_B.c_str() );
    gEEId_B = gEENode_B->getSkelIndex();	
    std::cout << "--> [B] End Effector: " << gEEName_B << " ID: " << gEEId_B << std::endl;
	}
  } 
  else {
    std::cout << "--(!) No robot set (!)--" << std::endl;
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
