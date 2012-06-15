/**
 * @file IKSearch
 * @author A. Huaman
 * @date 2012-06-08
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
					      const std::vector<Eigen::VectorXd> _WSPath );
  
  virtual bool GoToPose( Eigen::VectorXd &_q, 
			 Eigen::VectorXd _targetPose, 
			 std::vector<Eigen::VectorXd> &_jointPath );
  virtual Eigen::VectorXd Getdq( Eigen::VectorXd _q, Eigen::VectorXd _s );
  // ** Specific functions **
  Eigen::VectorXd Getdq2( Eigen::VectorXd _q, Eigen::VectorXd _s );
  bool GoToPose2( Eigen::VectorXd &_q, 
		  Eigen::VectorXd _targetPose, 
		  std::vector<Eigen::VectorXd> &_jointPath );
  Eigen::MatrixXd GetNS_Basis( Eigen::MatrixXd _J );
  
  // ** NS Specific functions **
  std::vector<Eigen::VectorXd> NS_ChainSearch( int _robotId, 
					       const Eigen::VectorXi &_links,
					       const Eigen::VectorXd _NSConf,
					       std::string _EEName,
					       int _EEId,
					       std::vector<int> _constraints,
					       int _maxChain = 10,
					       int _numCoeff = 10,
					       double _minCoeff = -10.0,
					       double _maxCoeff = 10.0 );

  std::vector<Eigen::VectorXd> NS_Search( Eigen::VectorXd &_q,
					  Eigen::VectorXd &_coeff );

  bool NS_GetSample( Eigen::VectorXd &_q, 
		     Eigen::VectorXd _coeff );
  // ** Specific Auxiliar functions **
  void GetCoeff( int _numCoeff = 10, 
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
};

#endif /** _IK_SEARCH_ */
