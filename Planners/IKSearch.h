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
  std::vector<Eigen::VectorXd> Track( int _robotId,
				      const Eigen::VectorXi &_links,
				      const Eigen::VectorXd &_start,
				      std::string _EEName,
				      int _EEId,
				      std::vector<int> _constraints,
				      const std::vector<Eigen::VectorXd> _WSPath );
};

#endif /** _IK_SEARCH_ */
