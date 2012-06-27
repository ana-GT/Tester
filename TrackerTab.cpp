/**
 * @file TrackerTab.cpp
 * @author A. Huaman
 */

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>
#include <string>
#include <iostream>
#include <GRIPApp.h>
#include "TrackerTab.h"
#include "Plot/Plot.h"

using namespace std;

enum TrackerTabEvents {
  button_Track_LA = 50,
  button_Execute_LA,
  button_Plot1S_LA,
  button_Track_BT,
  button_Execute_BT,
  button_Plot_1S_BT,
  button_SetPose_NS,
  button_Search_NS,
  button_Show_NS,
  button_Plot_1S_NS
};

//-- Sizer for TrackerTab
wxBoxSizer *sizerFullTab;

BEGIN_EVENT_TABLE( TrackerTab, wxPanel )
EVT_COMMAND( wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, TrackerTab::OnButton )
END_EVENT_TABLE()

IMPLEMENT_DYNAMIC_CLASS( TrackerTab, GRIPTab )

/**
 * @function TrackerTab
 * @brief Constructor
 */
TrackerTab::TrackerTab( wxWindow *parent, const wxWindowID id,
			const wxPoint &pos, const wxSize &size,
			long style ):
GRIPTab( parent, id, pos, size, style ) {
  sizerFullTab = new wxBoxSizer( wxHORIZONTAL );


  // ** Configuration BOX **
  wxStaticBox *Conf_Box = new wxStaticBox( this, -1, wxT("Configuration") );
  wxStaticBoxSizer *Conf_BoxSizer = new wxStaticBoxSizer( Conf_Box, wxVERTICAL );
  
  // ** Coeff **
  wxBoxSizer *CoeffSizer = new wxBoxSizer( wxVERTICAL );

  // Num coeff
  wxBoxSizer *NumCoeff_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *NumCoeff_Label = new wxStaticText( this, 1013, wxT("# coeff:") );
  mNumCoeff = new wxTextCtrl(this,1014,wxT("11"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  NumCoeff_Sizer->Add( NumCoeff_Label, 0, wxALL, 1 );  
  NumCoeff_Sizer->Add( mNumCoeff, 0, wxALL, 1 );

  CoeffSizer->Add( NumCoeff_Sizer, 1, wxALIGN_NOT );

  // Min Coeff
  wxBoxSizer *MinCoeff_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *MinCoeff_Label = new wxStaticText( this, 1013, wxT("Min:") );
  mMinCoeff = new wxTextCtrl(this,1014,wxT("-10.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  MinCoeff_Sizer->Add( MinCoeff_Label, 0, wxALL, 1 );  
  MinCoeff_Sizer->Add( mMinCoeff, 0, wxALL, 1 );

  CoeffSizer->Add( MinCoeff_Sizer, 1, wxALIGN_NOT );

  // Max coeff
  wxBoxSizer *MaxCoeff_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *MaxCoeff_Label = new wxStaticText( this, 1013, wxT("Max:") );
  mMaxCoeff = new wxTextCtrl(this,1014,wxT("10.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  MaxCoeff_Sizer->Add( MaxCoeff_Label, 0, wxALL, 1 );  
  MaxCoeff_Sizer->Add( mMaxCoeff, 0, wxALL, 1 );

  CoeffSizer->Add( MaxCoeff_Sizer, 1, wxALIGN_NOT );

 
  // Add sizers to NSBoxSizer 
  Conf_BoxSizer->Add( CoeffSizer, 1, wxALIGN_NOT, 0 );

  // Set the general sizer
  sizerFullTab->Add( Conf_BoxSizer, 1, wxEXPAND |wxALL, 4 );


  // ** NS BOX **
  wxStaticBox *NS_Box = new wxStaticBox( this, -1, wxT("Nullspace") );
  wxStaticBoxSizer *NS_BoxSizer = new wxStaticBoxSizer( NS_Box, wxVERTICAL );
  
  wxBoxSizer *NS_ButtonSizer = new wxBoxSizer( wxVERTICAL );
  NS_ButtonSizer->Add( new wxButton( this, button_SetPose_NS, _T("Set Pose") ),
		     1, wxALIGN_NOT );
  NS_ButtonSizer->Add( new wxButton( this, button_Search_NS, _T("Search NS") ),
		     1, wxALIGN_NOT );
  NS_ButtonSizer->Add( new wxButton( this, button_Show_NS, _T("Execute") ),
		     1, wxALIGN_NOT );
  NS_ButtonSizer->Add( new wxButton( this, button_Plot_1S_NS, _T("Plot") ),
		     1, wxALIGN_NOT );

 
  // Add sizers to NS_BoxSizer 
  NS_BoxSizer->Add( NS_ButtonSizer, 1, wxALIGN_NOT, 0 );

  // Set the general sizer
  sizerFullTab->Add( NS_BoxSizer, 1, wxEXPAND |wxALL, 4 );


  // ** Search BOX **
  wxStaticBox *Search_Box = new wxStaticBox( this, -1, wxT("Search") );
  wxStaticBoxSizer *Search_BoxSizer = new wxStaticBoxSizer( Search_Box, wxHORIZONTAL );


  // ** Button sizer BT (Backtrack) **
  wxBoxSizer *BT_ButtonSizer = new wxBoxSizer( wxVERTICAL );

  // BT Window
  wxBoxSizer *BT_Window_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *BT_Window_Label = new wxStaticText( this, 1013, wxT("BT Window:") );
  mBT_Window = new wxTextCtrl(this,1014,wxT("3"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  BT_Window_Sizer->Add( BT_Window_Label, 0, wxALL, 1 );  
  BT_Window_Sizer->Add( mBT_Window, 0, wxALL, 1 );

  BT_ButtonSizer->Add( BT_Window_Sizer, 1, wxALIGN_NOT );

  // BT Buttons
  BT_ButtonSizer->Add( new wxButton( this, button_Track_BT, _T("Track BT") ),
		     1, wxALIGN_NOT );
  BT_ButtonSizer->Add( new wxButton( this, button_Execute_BT, _T("Execute") ),
		     1, wxALIGN_NOT );
  BT_ButtonSizer->Add( new wxButton( this, button_Plot_1S_BT, _T("Plot") ),
		     1, wxALIGN_NOT );

  // ** Button sizer LA (Look Ahead) **
  wxBoxSizer *LA_ButtonSizer = new wxBoxSizer( wxVERTICAL );

  // LA Window
  wxBoxSizer *LA_Window_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *LA_Window_Label = new wxStaticText( this, 1013, wxT("LA Window:") );
  mLA_Window = new wxTextCtrl(this,1014,wxT("3"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  LA_Window_Sizer->Add( LA_Window_Label, 0, wxALL, 1 );  
  LA_Window_Sizer->Add( mLA_Window, 0, wxALL, 1 );

  LA_ButtonSizer->Add( LA_Window_Sizer, 1, wxALIGN_NOT );

  // LA Buttons
  LA_ButtonSizer->Add( new wxButton( this, button_Track_LA, _T("Track LA") ),
		      1, wxALIGN_NOT );
  LA_ButtonSizer->Add( new wxButton( this, button_Execute_LA, _T("Execute") ),
		      1, wxALIGN_NOT );
  LA_ButtonSizer->Add( new wxButton( this, button_Plot1S_LA, _T("Plot") ),
		      1, wxALIGN_NOT );


  // Add sizers to GBoxSizer 
  Search_BoxSizer->Add( BT_ButtonSizer, 1, wxALIGN_NOT, 0 );
  Search_BoxSizer->Add( LA_ButtonSizer, 1, wxALIGN_NOT, 0 );

  // Set the general sizer
  sizerFullTab->Add( Search_BoxSizer, 2, wxEXPAND |wxALL, 4 );


  SetSizer( sizerFullTab );
}

/**
 * @function ~TrackerTab
 * @brief Destructor
 */
TrackerTab::~TrackerTab() {
  if( mIk != NULL ){
    delete mIk;
  }
}


/**
 * @function OnButton
 */
void TrackerTab::OnButton( wxCommandEvent &evt ) {

  int button_num = evt.GetId();
  
  switch  (button_num) {
    
    
    /** Track Search Backtrack  IK */
  case button_Track_BT: {
    std::vector<int> constraints(6); 
    constraints[0] = 1;
    constraints[1] = 1;
    constraints[2] = 1;
    constraints[3] = 0;
    constraints[4] = 0;
    constraints[5] = 0;

    double numCoeff;
    double maxCoeff; 
    double minCoeff;
    double bt_window;
    mNumCoeff->GetValue().ToDouble( &numCoeff );
    mMaxCoeff->GetValue().ToDouble( &maxCoeff );
    mMinCoeff->GetValue().ToDouble( &minCoeff );
    mBT_Window->GetValue().ToDouble( &bt_window );

    IKSearch* ik = new IKSearch( *mWorld, mCollision );
    mTrack_BT_Path = ik->Track_BT( gRobotId,
				    gLinks,
				    gStartConf,
				    gEEName,
				    gEEId,
				    constraints,
				    gPosePath,
				    (int) bt_window,
				    10, // maxChain
				    numCoeff,
				    minCoeff,
				    maxCoeff );
    delete ik;
  }
    break;

    /** Track Search Look Ahead  IK */
  case button_Track_LA: {
    std::vector<int> constraints(6); 
    constraints[0] = 1;
    constraints[1] = 1;
    constraints[2] = 1;
    constraints[3] = 0;
    constraints[4] = 0;
    constraints[5] = 0;

    double numCoeff;
    double maxCoeff; double minCoeff;
    double la_window;

    mNumCoeff->GetValue().ToDouble( &numCoeff );
    mMaxCoeff->GetValue().ToDouble( &maxCoeff );
    mMinCoeff->GetValue().ToDouble( &minCoeff );
    mLA_Window->GetValue().ToDouble( &la_window );

    IKSearch* ik = new IKSearch( *mWorld, mCollision );
    mTrack_LA_Path = ik->Track_LA( gRobotId,
				   gLinks,
				   gStartConf,
				   gEEName,
				   gEEId,
				   constraints,
				   gPosePath,
				   (int) la_window,
				   10, // maxChain
				   numCoeff,
				   minCoeff,
				   maxCoeff );
    delete ik;
  }
    break;
    
    /** Execute Search BT3 IK */
  case button_Execute_BT: {
    SetTimeline( mTrack_BT_Path, 5.0 );
  }
    break;

    /** Execute Search LA IK */
  case button_Execute_LA: {
    SetTimeline( mTrack_LA_Path, 5.0 );
  }
    break;

    /** Plot BT3 (Backtrack 3) Joint evolution vs step */
  case button_Plot_1S_BT: {
    plotVariables( mTrack_BT_Path, "t", "joint", "Joint", "BT Joints vs t" );
  } break;

    /** Plot LA (Look Ahead) Joint evolution vs step */
  case button_Plot1S_LA: {
    plotVariables( mTrack_LA_Path, "t", "joint", "Joint", "LA Joints vs t" );
  } break;

    /** Set Pose for NS Evaluation */
  case button_SetPose_NS: {

      std::cout << "--(i) Setting NS Pose for " << mWorld->mRobots[gRobotId]->getName() << ":" << std::endl;
      
      mNSConf = mWorld->mRobots[gRobotId]->getDofs( gLinks );      
      std::cout << "NS Pose: " << mNSConf.transpose() << std::endl;
    
  } break;

    /** Search NS Evaluation */
  case button_Search_NS: {

    std::vector<int> constraints(6); 
    constraints[0] = 1;
    constraints[1] = 1;
    constraints[2] = 1;
    constraints[3] = 0;
    constraints[4] = 0;
    constraints[5] = 0;
  

    double numCoeff;
    double maxCoeff; double minCoeff;
    mNumCoeff->GetValue().ToDouble( &numCoeff );
    mMaxCoeff->GetValue().ToDouble( &maxCoeff );
    mMinCoeff->GetValue().ToDouble( &minCoeff );

    IKSearch* ik = new IKSearch( *mWorld, mCollision );
    mExecutePath = ik->NS_ChainSearch( gRobotId,
				       gLinks,
				       mNSConf,
				       gEEName,
				       gEEId,
				       constraints,
				       20,
				       (int) numCoeff, 
				       minCoeff, 
				       maxCoeff);
    delete ik;

  } break;

    /** Show all NS Solutions found */
  case button_Show_NS: {
    SetTimeline( mExecutePath, 5.0 );
  } break;

    /** Plot solution joints */
  case button_Plot_1S_NS: {
    plotVariables( mExecutePath, "t", "joint", "Joint", "Joint vs t" );
  } break;

  }
}
