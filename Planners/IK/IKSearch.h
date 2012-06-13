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
  std::vector<Eigen::VectorXd> Getdq2( Eigen::VectorXd _q, Eigen::VectorXd _s );
  bool GoToPose2( Eigen::VectorXd &_q, 
		  Eigen::VectorXd _targetPose, 
		  std::vector<Eigen::VectorXd> &_jointPath );
  Eigen::MatrixXd GetNS_Basis( Eigen::MatrixXd _J );
  
  // ** Specific Auxiliar functions **
  void GetCoeff_JRM();
  double JRM_Measure( Eigen::VectorXd _conf );

  // Member
  double mNSNorm;

  /// Constants for class
  static const double sMinCoeff;
  static const double sMaxCoeff;
  static const int sNumCoeff;
  static const double sdCoeff;
  
  /// Keep going
  Eigen::VectorXd mCoeff1_JRM;
  Eigen::VectorXd mCoeff2_JRM;
  double* sCoeff;
};

#endif /** _IK_SEARCH_ */
