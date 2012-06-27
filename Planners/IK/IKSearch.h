/**
 * @file IKSearch
 * @author A. Huaman
 * @date 2012-06-08 - modified 2012-06-24
 */

#ifndef _IK_SEARCH_
#define _IK_SEARCH_

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <planning/World.h>
#include <Tools/Collision.h>

#include "globalStuff.h"
#include "IK.h"

/**
 * @class IKSearch
 * @brief General Inverse Kinematics Class
 */
class IKSearch : public IK {
 public:
  IKSearch() {};
  IKSearch( planning::World &_world,
	    Collision *_collision );
  virtual ~IKSearch();
  virtual std::vector<Eigen::VectorXd> Track( int _robotId,
					      const Eigen::VectorXi &_links,
					      const Eigen::VectorXd &_start,
					      std::string _EEName,
					      int _EEId,
					      std::vector<int> _constraints,
					      const std::vector<Eigen::VectorXd> _WSPath,
					      int _maxChain = 10,
					      int _numCoeff = 11,
					      double _minCoeff = -10.0,
					      double _maxCoeff = 10.0 );

  virtual bool GoToPose( Eigen::VectorXd &_q, 
			 Eigen::VectorXd _targetPose, 
			 std::vector<Eigen::VectorXd> &_jointPath );
  virtual Eigen::VectorXd Getdq( Eigen::VectorXd _q, Eigen::VectorXd _s );
  
  // ** NS Specific functions **
  Eigen::MatrixXd GetNS_Basis( Eigen::MatrixXd _J );

  //------- Look-ahead function -------//
 std::vector< Eigen::VectorXd > Track_LA( int _robotId,
					  const Eigen::VectorXi &_links,
					  const Eigen::VectorXd &_start,  
					  std::string _EEName,
					  int _EEId,
					  std::vector<int> _constraints,
					  const std::vector<Eigen::VectorXd> _WSPath,
					  int _window = 3,
					  int _maxChain = 10,
					  int _numCoeff = 11,
					  double _minCoeff = -10.0,
					  double _maxCoeff = 10.0 );
 
 bool LookAhead( int _i,
		 int _window,
		 int _maxWindow,
		 std::vector< std::vector<Eigen::VectorXd> > &_qSet,
		 std::vector< std::vector<int> > &_heapSet,
		 std::vector< std::vector<double> > &_valSet,
		 std::vector< Eigen::VectorXd > &_qPath,
		 const std::vector<Eigen::VectorXd> &_WSPath );

 void ClearAhead_LA( int _i, 
		     int _w,
		     std::vector< std::vector<Eigen::VectorXd> > &_qSet,
		     std::vector< std::vector<int> > &_heapSet,
		     std::vector< std::vector<double> > &_valSet );
   
  //------ Backtrack function: Window 2 ------//
  std::vector<Eigen::VectorXd> Track_BT2( int _robotId,
					  const Eigen::VectorXi &_links,
					  const Eigen::VectorXd &_start,
					  std::string _EEName,
					  int _EEId,
					  std::vector<int> _constraints,
					  const std::vector<Eigen::VectorXd> _WSPath,
					  int _maxChain = 10,
					  int _numCoeff = 11,
					  double _minCoeff = -10.0,
					  double _maxCoeff = 10.0 );  

  bool GenerateNSSet( Eigen::VectorXd _q,
		      Eigen::VectorXd _s,
		      std::vector<Eigen::VectorXd> &_qSet,
		      std::vector<int> &_prioritySet,
		      std::vector<double> &_valSet );

  void SortNS( std::vector<Eigen::VectorXd> _configs, 
	       std::vector<double> &_vals,
	       std::vector<int> &_priority );


  //------ Backtrack function: Window 3 ------//
  std::vector<Eigen::VectorXd> Track_BT3( int _robotId,
					  const Eigen::VectorXi &_links,
					  const Eigen::VectorXd &_start,
					  std::string _EEName,
					  int _EEId,
					  std::vector<int> _constraints,
					  const std::vector<Eigen::VectorXd> _WSPath,
					  int _maxChain = 10,
					  int _numCoeff = 11,
					  double _minCoeff = -10.0,
					  double _maxCoeff = 10.0 );  


  void TrackReset();

  //------ Backtrack function ------//
  std::vector<Eigen::VectorXd> Track_BT( int _robotId,
					 const Eigen::VectorXi &_links,
					 const Eigen::VectorXd &_start,
					 std::string _EEName,
					 int _EEId,
					 std::vector<int> _constraints,
					 const std::vector<Eigen::VectorXd> _WSPath,
					 int _window = 3,
					 int _maxChain = 10,
					 int _numCoeff = 11,
					 double _minCoeff = -10.0,
					 double _maxCoeff = 10.0 );

  bool BackTrack( int _i, 
		  int _window, 
		  int _maxWindow, 
		  std::vector< std::vector<Eigen::VectorXd> > &_qSet,
		  std::vector< std::vector<int> > &_heapSet,
		  std::vector< std::vector<double> > &_valSet,
		  std::vector< Eigen::VectorXd > &_qPath,
		  const std::vector<Eigen::VectorXd> &_WSPath );

  bool ForwardSearch( int _i,
		      int _window,
		      int _maxWindow,
		      std::vector< std::vector<Eigen::VectorXd> > &_qSet,
		      std::vector< std::vector<int> > &_heapSet,
		      std::vector< std::vector<double> > &_valSet,
		      std::vector< Eigen::VectorXd > &_qPath,
		      const std::vector<Eigen::VectorXd> &_WSPath ); 
  

  void ClearAhead( int _i, 
		   int _w,
		   std::vector< std::vector<Eigen::VectorXd> > &_qSet,
		   std::vector< std::vector<int> > &_heapSet,
		   std::vector< std::vector<double> > &_valSet );
    
  void UpdateBackTrack( int _i,
			int _window,
			std::vector< std::vector<Eigen::VectorXd> > &_qSet,
			const std::vector< std::vector<int> > &_heapSet,
			std::vector< Eigen::VectorXd > &_qPath );

  //--------------------------------------------//
  std::vector<Eigen::VectorXd> NS_ChainSearch( int _robotId, 
					       const Eigen::VectorXi &_links,
					       const Eigen::VectorXd _NSConf,
					       std::string _EEName,
					       int _EEId,
					       std::vector<int> _constraints,
					       int _maxChain = 10,
					       int _numCoeff = 11,
					       double _minCoeff = -10.0,
					       double _maxCoeff = 10.0 );

  void NS_Search( Eigen::VectorXd &_q,
		  Eigen::VectorXd &_coeff,
		  std::vector<Eigen::VectorXd> &_configSet,
		  std::vector<Eigen::VectorXd> &_coeffSet );
  
  bool NS_GetSample( Eigen::VectorXd &_q, 
		     Eigen::VectorXd _coeff );
  
  // ** Specific Auxiliar functions **
  void GetCoeff( int _numCoeff = 11, 
		 double _minCoeff = -10.0,
		 double _maxCoeff = 10.0 );
  void GetCoeff_JRM();
  double JRM_Measure( Eigen::VectorXd _conf );

  // Member
  double mNSNorm;

  /// Constants for class
  double mMinCoeff;
  double mMaxCoeff;
  int mNumCoeff;
  double mdCoeff;
  
  /// Keep going
  Eigen::VectorXd mCoeff1_JRM;
  Eigen::VectorXd mCoeff2_JRM;
  double* mCoeff;

  /// BT_2 / BT_3
  std::vector< std::vector<Eigen::VectorXd> > NSSet;
  std::vector< std::vector<double> > NSValues;
  std::vector< std::vector<int> > NSPriority;
};

#endif /** _IK_SEARCH_ */

