/**
 * @file IKSearch.cpp
 * @brief Implementation of IK Naive Search
 * @author A. Huaman
 */

#include "IKSearch.h"

/**
 * @function IKSearch
 * @brief Constructor
 */
IKSearch::IKSearch( planning::World &_world,
		    Collision *_collision) 
  : IK( _world, _collision ) {
  
}

/**
 * @function ~IKSearch
 * @brief Destructor
 */
IKSearch::~IKSearch() {
}
