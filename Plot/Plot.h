/**
 * @file Plot.h
 * @brief copied from lib/trajctrl/trajctrl.cpp
 * @author A. Huaman
 */

#ifndef _PLOT_H_
#define _PLOT_H_

#include <Eigen/Core>
#include <vector>
#include <string>



int plotVariables( std::vector<Eigen::VectorXd> _var, 
		   std::string _xlabel = "time", 
		   std::string _ylabel = "joints",
		   std::string _variable = "Joint",
		   std::string _title = "Joint vs time" );

int plotVariables_d1( std::vector<Eigen::VectorXd> _var, 
		      std::string _xlabel = "time", 
		      std::string _ylabel = "joints",
		      std::string _variable = "Joint",
		      std::string _title = "Joint vs time" );

#endif /** _PLOT_H_ */

