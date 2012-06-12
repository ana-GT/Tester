/**
 * @file IKGradient
 * @author A. Huaman
 * @date 2012-06-12
 */
#ifndef _IK_GRADIENT_
#define _IK_GRADIENT_

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <planning/World.h>
#include <Tools/Collision.h>

#include "globalStuff.h"
#include "IK.h"

/**
 * @class IKGradient
 * @brief IK with gradient function for nullspace
 */
class IKGradient : public IK {

 public:
  IKGradient() {};
  IKGradient( planning::World &_world,
	      Collision *_collision );
  virtual ~IKGradient();
  virtual Eigen::VectorXd Getdq( Eigen::VectorXd _q, Eigen::VectorXd _s );
  Eigen::VectorXd dJRA( Eigen::VectorXd _conf );
  void GetCoeff_dJRA();
  virtual std::vector<Eigen::VectorXd> Track( int _robotId,
					      const Eigen::VectorXi &_links,
					      const Eigen::VectorXd &_start,
					      std::string _EEName,
					      int _EEId,
					      std::vector<int> _constraints,
					      const std::vector<Eigen::VectorXd> _WSPath,
					      double _wJRA = 0.0,
					      double _wManip = 0.0 );
  
  /// Constants for class
  double mW_JRA;
  double mW_Manip;
  Eigen::VectorXd mCoeff1_JRA;
  Eigen::VectorXd mCoeff2_JRA;

};

#endif /** _IK_GRADIENT_  */
