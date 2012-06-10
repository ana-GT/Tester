/**
 * @file Plot.cpp
 * @author A. Huaman
 * @date 2012/06/10
 * @brief Plot utilities
 */

#include "Plot.h"
#include <stdio.h>

/**
 * @function plotVariables
 */
int plotVariables( std::vector<Eigen::VectorXd> _var, 
		   std::string _xlabel, 
		   std::string _ylabel,
		   std::string _variable,
		   std::string _title ) {

  int numVar = _var[0].size();
  int numCount = _var.size();

  // Start Gnuplot
  FILE *g = popen( "gnuplot -persist", "w" );
  assert(g);

  // Set plot configuration
  fprintf( g, "set title '%s' \n", _title.c_str() );
  fprintf( g, "set xlabel '%s' \n", _xlabel.c_str() );
  fprintf( g, "set ylabel '%s'\n", _ylabel.c_str() );
  
  // Plot names
  fprintf( g, "plot '-' with lines title '%s 0'", _variable.c_str() );
  for( size_t j = 1; j < numVar; ++j ) {
    fprintf( g, ", '-' with lines title '%s %d'", _variable.c_str(), j);
  }
  fprintf( g, "\n" );

  // Actually do the plot!
  for( size_t j = 0; j < numVar; ++j ) {
    for( size_t i = 0; i < numCount; ++i ) {
      fprintf( g, "%d %f \n", i, _var[i][j] );
    }
    fprintf( g, "e\n" );
  }

  // Flush 
  fflush(g);
  int r = 0;
  r = pclose(g);
  
}

/**
 * @function plotVariables_d1
 * @brief Plot the first derivative of Variables
 */
int plotVariables_d1( std::vector<Eigen::VectorXd> _var, 
		      std::string _xlabel, 
		      std::string _ylabel,
		      std::string _variable,
		      std::string _title ) {
  int numVar = _var[0].size();
  int numCount = _var.size();
  
  // Start Gnuplot
  FILE *g = popen( "gnuplot -persist", "w" );
  assert(g);

  // Set plot configuration
  fprintf( g, "set title '%s' \n", _title.c_str() );
  fprintf( g, "set xlabel '%s' \n", _xlabel.c_str() );
  fprintf( g, "set ylabel '%s'\n", _ylabel.c_str() );
  
  // Plot names
  fprintf( g, "plot '-' with lines title '%s 0'", _variable.c_str() );
  for( size_t j = 1; j < numVar; ++j ) {
    fprintf( g, ", '-' with lines title '%s %d'", _variable.c_str(), j);
  }
  fprintf( g, "\n" );

  // Actually do the plot!
  for( size_t j = 0; j < numVar; ++j ) {
    for( size_t i = 1; i < numCount; ++i ) {
      fprintf( g, "%d %f \n", i, _var[i][j] - _var[i-1][j] );
    }
    fprintf( g, "e\n" );
  }

  // Flush 
  fflush(g);
  int r = 0;
  r = pclose(g);
  
}
