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

using namespace std;

enum TrackerTabEvents {
  checkbox_ManipG = 50,
  checkbox_JointsG,
  checkbox_DexterityS,
  checkbox_JvmS,
  checkbox_JraS,
  button_TrackG,
  button_TrackS
};

//-- Sizer for TrackerTab
wxBoxSizer *sizerFullTab;

BEGIN_EVENT_TABLE( TrackerTab, wxPanel )
EVT_COMMAND( wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, TrackerTab::OnButton )
EVT_COMMAND ( wxID_ANY, wxEVT_COMMAND_CHECKBOX_CLICKED, TrackerTab::OnCheckbox )
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

  // GBox
  wxStaticBox *GBox = new wxStaticBox( this, -1, wxT("Gradient Method") );
  wxStaticBoxSizer *GBoxSizer = new wxStaticBoxSizer( GBox, wxVERTICAL );
  
  // Gradient Checkboxes
  wxBoxSizer *GCheckSizer = new wxBoxSizer( wxVERTICAL );
  GCheckSizer->Add( new wxCheckBox( this, checkbox_ManipG, _T("Manip") ), 
		    1, wxALIGN_NOT );
  GCheckSizer->Add( new wxCheckBox( this, checkbox_JointsG, _T("Joints lim") ),
		    1, wxALIGN_NOT );

  wxBoxSizer *GButtonSizer = new wxBoxSizer( wxVERTICAL );
  GButtonSizer->Add( new wxButton( this, button_TrackG, _T("Track") ),
		     1, wxALIGN_NOT );
 
  // Add sizers to GBoxSizer 
  GBoxSizer->Add( GCheckSizer, 1, wxALIGN_NOT, 0 );
  GBoxSizer->Add( GButtonSizer, 1, wxALIGN_NOT, 0 );

  // Set the general sizer
  sizerFullTab->Add( GBoxSizer, 1, wxEXPAND |wxALL, 2 );

  // SBox
  wxStaticBox *SBox = new wxStaticBox( this, -1, wxT("Search") );
  wxStaticBoxSizer *SBoxSizer = new wxStaticBoxSizer( SBox, wxVERTICAL );

  // Search Checkboxes
  wxBoxSizer *SCheckSizer = new wxBoxSizer( wxVERTICAL );
  SCheckSizer->Add( new wxCheckBox( this, checkbox_DexterityS, _T("Dexterity") ), 
		    1, wxALIGN_NOT );
  SCheckSizer->Add( new wxCheckBox( this, checkbox_JvmS, _T("JVM") ),
		    1, wxALIGN_NOT );
  SCheckSizer->Add( new wxCheckBox( this, checkbox_JraS, _T("JRA") ),
		    1, wxALIGN_NOT );

  wxBoxSizer *SButtonSizer = new wxBoxSizer( wxVERTICAL );
  SButtonSizer->Add( new wxButton( this, button_TrackS, _T("Track") ),
		     1, wxALIGN_NOT );
 
  // Add sizers to GBoxSizer 
  SBoxSizer->Add( SCheckSizer, 1, wxALIGN_NOT, 0 );
  SBoxSizer->Add( SButtonSizer, 1, wxALIGN_NOT, 0 );

  // Set the general sizer
  sizerFullTab->Add( SBoxSizer, 1, wxEXPAND |wxALL, 2 );

  SetSizer( sizerFullTab );
}

/**
 * 
 */
TrackerTab::~TrackerTab() {
}

/**
 * @function OnCheckBox
 */
void TrackerTab::OnCheckbox( wxCommandEvent &evt ) {

  int checkbox_num = evt.GetId();
  
  switch( checkbox_num ) {
  case checkbox_ManipG:
    oManipG = (bool) evt.GetSelection();
    break;
  case checkbox_JointsG:
    oJointsG = (bool) evt.GetSelection();
    break;
  case checkbox_DexterityS:
    oDexterityS = (bool) evt.GetSelection();
    break;
  case checkbox_JvmS:
    oJvmS = (bool) evt.GetSelection();
    break;
  case checkbox_JraS:
    oJraS = (bool) evt.GetSelection();
    break;
  }
}

/**
 * @function OnButton
 */
void TrackerTab::OnButton( wxCommandEvent &evt ) {

  int button_num = evt.GetId();
  
  switch  (button_num) {
  case button_TrackG: {

    std::vector<int> constraints(6); 
    constraints[0] = 1;
    constraints[1] = 1;
    constraints[2] = 1;
    constraints[3] = 0;
    constraints[4] = 0;
    constraints[5] = 0;

    std::vector<Eigen::VectorXd> jointPath;

    mIk = new IK( *mWorld, mCollision );
    jointPath = mIk->Track( gRobotId, 
		gLinks,
		gStartConf,
		gEEName,
		gEEId,
		constraints,
		gPosePath );
  }
    break;
  }
}
