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
  button_TrackB = 50,
  button_TrackG,
  button_TrackS,
  button_ExecuteB,
  button_ExecuteG,
  button_ExecuteS,
  button_Plot1S,
  button_SearchNS,
  button_ShowNS,
  button_PlotNS,
  button_SetPoseNS
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

  // ** BBOX **
  wxStaticBox *BBox = new wxStaticBox( this, -1, wxT("Basic IK Method") );
  wxStaticBoxSizer *BBoxSizer = new wxStaticBoxSizer( BBox, wxVERTICAL );
  
  wxBoxSizer *BButtonSizer = new wxBoxSizer( wxHORIZONTAL );
  BButtonSizer->Add( new wxButton( this, button_TrackB, _T("Track") ),
		     1, wxALIGN_NOT );
  BButtonSizer->Add( new wxButton( this, button_ExecuteB, _T("Execute") ),
		     1, wxALIGN_NOT );
 
  // Add sizers to BBoxSizer 
  BBoxSizer->Add( BButtonSizer, 1, wxALIGN_NOT, 0 );

  // Set the general sizer
  sizerFullTab->Add( BBoxSizer, 1, wxEXPAND |wxALL, 4 );

  // ** GBOX **
  wxStaticBox *GBox = new wxStaticBox( this, -1, wxT("Gradient Method") );
  wxStaticBoxSizer *GBoxSizer = new wxStaticBoxSizer( GBox, wxVERTICAL );
  
  // Gradient Checkboxes

  // Manip
  wxBoxSizer *GManip_CheckSizer = new wxBoxSizer( wxHORIZONTAL );

  wxBoxSizer *WManipGSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *WManipGLabel = new wxStaticText( this, 1013, wxT("Manipulatibility  weight:") );
  mW_ManipG = new wxTextCtrl(this,1014,wxT("0.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

  GManip_CheckSizer->Add( WManipGLabel, 0, wxALL, 1 );  
  GManip_CheckSizer->Add( mW_ManipG, 0, wxALL, 1 );

  // Joint lim
  wxBoxSizer *GJoint_CheckSizer = new wxBoxSizer( wxHORIZONTAL );

  wxBoxSizer *WJointGSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *WJointGLabel = new wxStaticText( this, 1013, wxT("JRA weight:") );
  mW_JraG = new wxTextCtrl(this,1014,wxT("0.2"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);

  GJoint_CheckSizer->Add( WJointGLabel, 0, wxALL, 1 );  
  GJoint_CheckSizer->Add( mW_JraG, 0, wxALL, 1 );

  wxBoxSizer *GButtonSizer = new wxBoxSizer( wxHORIZONTAL );
  GButtonSizer->Add( new wxButton( this, button_TrackG, _T("Track") ),
		     1, wxALIGN_NOT );
  GButtonSizer->Add( new wxButton( this, button_ExecuteG, _T("Execute") ),
		     1, wxALIGN_NOT );
 
  // Add sizers to GBoxSizer 
  GBoxSizer->Add( GManip_CheckSizer, 1, wxALIGN_NOT, 0 );
  GBoxSizer->Add( GJoint_CheckSizer, 1, wxALIGN_NOT, 0 );
  GBoxSizer->Add( GButtonSizer, 1, wxALIGN_NOT, 0 );

  // Set the general sizer
  sizerFullTab->Add( GBoxSizer, 1, wxEXPAND |wxALL, 4 );

  // ** SBOX **
  wxStaticBox *SBox = new wxStaticBox( this, -1, wxT("Search") );
  wxStaticBoxSizer *SBoxSizer = new wxStaticBoxSizer( SBox, wxVERTICAL );

  // Search Checkboxes
  wxBoxSizer *SCheckSizer = new wxBoxSizer( wxVERTICAL );

  // Dexterity
  wxBoxSizer *WDexterityS_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *WDexterityS_Label = new wxStaticText( this, 1013, wxT("Dexterity  weight:") );
  mW_DexterityS = new wxTextCtrl(this,1014,wxT("0.2"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  WDexterityS_Sizer->Add( WDexterityS_Label, 0, wxALL, 1 );  
  WDexterityS_Sizer->Add( mW_DexterityS, 0, wxALL, 1 );

  SCheckSizer->Add( WDexterityS_Sizer, 1, wxALIGN_NOT );

  // JRA
  wxBoxSizer *WJraS_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *WJraS_Label = new wxStaticText( this, 1013, wxT("Jra  weight:") );
  mW_JraS = new wxTextCtrl(this,1014,wxT("0.2"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  WJraS_Sizer->Add( WJraS_Label, 0, wxALL, 1 );  
  WJraS_Sizer->Add( mW_JraS, 0, wxALL, 1 );

  SCheckSizer->Add( WJraS_Sizer, 1, wxALIGN_NOT );

  // JVM
  wxBoxSizer *WJvmS_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *WJvmS_Label = new wxStaticText( this, 1013, wxT("Jvm  weight:") );
  mW_JvmS = new wxTextCtrl(this,1014,wxT("0.2"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  WJvmS_Sizer->Add( WJvmS_Label, 0, wxALL, 1 );  
  WJvmS_Sizer->Add( mW_JvmS, 0, wxALL, 1 );

  SCheckSizer->Add( WJvmS_Sizer, 1, wxALIGN_NOT );


  // ** Button sizer **
  wxBoxSizer *SButtonSizer = new wxBoxSizer( wxHORIZONTAL );
  SButtonSizer->Add( new wxButton( this, button_TrackS, _T("Track") ),
		     1, wxALIGN_NOT );
  SButtonSizer->Add( new wxButton( this, button_ExecuteS, _T("Execute") ),
		     1, wxALIGN_NOT );
  SButtonSizer->Add( new wxButton( this, button_Plot1S, _T("Plot") ),
		     1, wxALIGN_NOT );

  // Add sizers to GBoxSizer 
  SBoxSizer->Add( SCheckSizer, 1, wxALIGN_NOT, 0 );
  SBoxSizer->Add( SButtonSizer, 1, wxALIGN_NOT, 0 );

  // Set the general sizer
  sizerFullTab->Add( SBoxSizer, 1, wxEXPAND |wxALL, 4 );

  // ** NS Box **
  wxStaticBox *NSBox = new wxStaticBox( this, -1, wxT("Nullspace") );
  wxStaticBoxSizer *NSBoxSizer = new wxStaticBoxSizer( NSBox, wxVERTICAL );
  
  wxBoxSizer *NSConfigSizer = new wxBoxSizer( wxHORIZONTAL );
  wxBoxSizer *NSSearchSizer = new wxBoxSizer( wxHORIZONTAL );
  NSConfigSizer->Add( new wxButton( this, button_SetPoseNS, _T("Set Pose") ),
		      1, wxALIGN_NOT );

  NSSearchSizer->Add( new wxButton( this, button_SearchNS, _T("Search") ),
		      1, wxALIGN_NOT );
  NSSearchSizer->Add( new wxButton( this, button_ShowNS, _T("Show") ),
		      1, wxALIGN_NOT );
  NSSearchSizer->Add( new wxButton( this, button_PlotNS, _T("Plot") ),
		      1, wxALIGN_NOT );


  // ** Coeff **
  wxBoxSizer *NSCoeffSizer = new wxBoxSizer( wxHORIZONTAL );

  // Num coeff
  wxBoxSizer *NSNumCoeff_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *NSNumCoeff_Label = new wxStaticText( this, 1013, wxT("# coeff:") );
  mNS_NumCoeff = new wxTextCtrl(this,1014,wxT("11"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  NSNumCoeff_Sizer->Add( NSNumCoeff_Label, 0, wxALL, 1 );  
  NSNumCoeff_Sizer->Add( mNS_NumCoeff, 0, wxALL, 1 );

  NSCoeffSizer->Add( NSNumCoeff_Sizer, 1, wxALIGN_NOT );

  // Min Coeff
  wxBoxSizer *NSMinCoeff_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *NSMinCoeff_Label = new wxStaticText( this, 1013, wxT("Min:") );
  mNS_MinCoeff = new wxTextCtrl(this,1014,wxT("-10.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  NSMinCoeff_Sizer->Add( NSMinCoeff_Label, 0, wxALL, 1 );  
  NSMinCoeff_Sizer->Add( mNS_MinCoeff, 0, wxALL, 1 );

  NSCoeffSizer->Add( NSMinCoeff_Sizer, 1, wxALIGN_NOT );

  // Max coeff
  wxBoxSizer *NSMaxCoeff_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *NSMaxCoeff_Label = new wxStaticText( this, 1013, wxT("Max:") );
  mNS_MaxCoeff = new wxTextCtrl(this,1014,wxT("10.0"), wxDefaultPosition,wxSize(40,20),wxTE_LEFT);

  NSMaxCoeff_Sizer->Add( NSMaxCoeff_Label, 0, wxALL, 1 );  
  NSMaxCoeff_Sizer->Add( mNS_MaxCoeff, 0, wxALL, 1 );

  NSCoeffSizer->Add( NSMaxCoeff_Sizer, 1, wxALIGN_NOT );

 
  // Add sizers to NSBoxSizer 
  NSBoxSizer->Add( NSConfigSizer, 1, wxALIGN_NOT, 0 );
  NSBoxSizer->Add( NSCoeffSizer, 1, wxALIGN_NOT, 0 );
  NSBoxSizer->Add( NSSearchSizer, 1, wxALIGN_NOT, 0 );

  // Set the general sizer
  sizerFullTab->Add( NSBoxSizer, 1, wxEXPAND |wxALL, 4 );

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
    
    /** Track basic IK (no collision detection) */
  case button_TrackB: {
    std::vector<int> constraints(6); 
    constraints[0] = 1;
    constraints[1] = 1;
    constraints[2] = 1;
    constraints[3] = 0;
    constraints[4] = 0;
    constraints[5] = 0;
    mIk = new IK( *mWorld, mCollision );
    mExecutePath = mIk->Track( gRobotId,
			       gLinks,
			       gStartConf,
			       gEEName,
			       gEEId,
			       constraints,
			       gPosePath );
    
  } break;
    
    /** Track Simple Search IK */
  case button_TrackS: {
    std::vector<int> constraints(6); 
    constraints[0] = 1;
    constraints[1] = 1;
    constraints[2] = 1;
    constraints[3] = 0;
    constraints[4] = 0;
    constraints[5] = 0;

    double w_dexterity;
    double w_jra;
    double w_jvm;  
    mW_DexterityS->GetValue().ToDouble(&w_dexterity);
    mW_JraS->GetValue().ToDouble(&w_jra);
    mW_JvmS->GetValue().ToDouble(&w_jvm);

    double numCoeff;
    double maxCoeff; double minCoeff;
    mNS_NumCoeff->GetValue().ToDouble( &numCoeff );
    mNS_MaxCoeff->GetValue().ToDouble( &maxCoeff );
    mNS_MinCoeff->GetValue().ToDouble( &minCoeff );


    IKSearch* ik = new IKSearch( *mWorld, mCollision );
    mExecutePath = ik->Track_LJM( gRobotId,
				  gLinks,
				  gStartConf,
				  gEEName,
				  gEEId,
				  constraints,
				  gPosePath,
				  10, // maxChain
				  numCoeff,
				  minCoeff,
				  maxCoeff );
    delete ik;
  }
    break;
    
    /** Track Gradient-based IK */
  case button_TrackG: {
    std::vector<int> constraints(6); 
    constraints[0] = 1;
    constraints[1] = 1;
    constraints[2] = 1;
    constraints[3] = 0;
    constraints[4] = 0;
    constraints[5] = 0;

    double w_jra;
    double w_manip;  
    mW_ManipG->GetValue().ToDouble(&w_manip);
    mW_JraG->GetValue().ToDouble(&w_jra);
    
    IKGradient *ik = new IKGradient( *mWorld, mCollision );
    mExecutePath = ik->Track( gRobotId, 
			      gLinks,
			      gStartConf,
			      gEEName,
			      gEEId,
			      constraints,
			      gPosePath,
			      w_jra, w_manip );
    delete ik;
  }
    break;

    /** Execute Simple IK */
  case button_ExecuteB: {
    SetTimeline( mExecutePath, 5.0 );
  }
    break;

    /** Execute Gradient IK */
  case button_ExecuteG: {
    SetTimeline( mExecutePath, 5.0 );
  }
    break;    

    /** Execute Simple Search IK */
  case button_ExecuteS: {
    SetTimeline( mExecutePath, 5.0 );
  }
    break;

    /** Plot Joint evolution vs step */
  case button_Plot1S: {
    plotVariables( mExecutePath, "t", "joint", "Joint", "Joints vs t" );
  } break;

    /** Set Pose for NS Evaluation */
  case button_SetPoseNS: {

      std::cout << "--(i) Setting NS Pose for " << mWorld->mRobots[gRobotId]->getName() << ":" << std::endl;
      
      mNSConf = mWorld->mRobots[gRobotId]->getDofs( gLinks );      
      std::cout << "NS Pose: " << mNSConf.transpose() << std::endl;
    
  } break;

    /** Search NS Evaluation */
  case button_SearchNS: {

    std::vector<int> constraints(6); 
    constraints[0] = 1;
    constraints[1] = 1;
    constraints[2] = 1;
    constraints[3] = 0;
    constraints[4] = 0;
    constraints[5] = 0;
  

    double numCoeff;
    double maxCoeff; double minCoeff;
    mNS_NumCoeff->GetValue().ToDouble( &numCoeff );
    mNS_MaxCoeff->GetValue().ToDouble( &maxCoeff );
    mNS_MinCoeff->GetValue().ToDouble( &minCoeff );

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
  case button_ShowNS: {
    SetTimeline( mExecutePath, 5.0 );
  } break;

    /** Plot solution joints */
  case button_PlotNS: {
    plotVariables( mExecutePath, "t", "joint", "Joint", "Joint vs t" );
  } break;

  }
}
