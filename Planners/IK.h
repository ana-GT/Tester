/**
 * @file IK.h
 * @date 2012-06-07
 * @author A. Huaman
 */
#ifndef _IK_H_
#define _IK_H_

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <planning/World.h>
#include <Tools/Collision.h>

/**
 * @class IK
 * @brief Generic IK class
 */
class IK {

 public:

  /// Functions
  IK();
  IK( planning::World &_world,
      Collision *_collision );
  virtual ~IK();
  std::vector<Eigen::VectorXd> Track( int _robotId,
				      const Eigen::VectorXi &_links,
				      const Eigen::VectorXd &_start,
				      std::string _EEName,
				      int _EEId,
				      std::vector<int> _constraints,
				      const std::vector<Eigen::VectorXd> _WSPath );

  bool GoToPose( Eigen::VectorXd &_q, 
		 Eigen::VectorXd _targetPose, 
		 std::vector<Eigen::VectorXd> &_jointPath );
  Eigen::VectorXd GetPose( Eigen::VectorXd _q );
  Eigen::VectorXd GetPoseError( Eigen::VectorXd _s1, Eigen::VectorXd _s2 );
  Eigen::VectorXd GetGeneralIK( Eigen::VectorXd _q, Eigen::VectorXd _ds );
  Eigen::MatrixXd GetJ( const Eigen::VectorXd &_q );
  Eigen::MatrixXd GetJps( const Eigen::VectorXd _q );


  /// Member variables
  planning::World *mWorld;
  Collision *mCollision;
  int mRobotId;
  Eigen::VectorXi mLinks;
  int mNumLinks;

  kinematics::BodyNode *mEENode;
  int mEEId;
  std::vector<Eigen::VectorXd> mWSPath;
  
  std::vector<int> mConstraints;
  int mNumConstraints;
  double mPoseThresh;
  int mMaxIter;
  

};

#endif /** _IK_ */
