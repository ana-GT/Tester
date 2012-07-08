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
/**
 * @function printPlotData
 */
int printPlotData( std::vector<Eigen::VectorXd> _var,
	           double _dt,
		   std::string _name, 
		   std::vector<double> _t ) {

    int n = _var.size();

    if( n == 0 ) {
        printf( " (!) No data to print! \n" );
        return -1;        
    }

    int m = _var[0].size();
	
    if( _dt == 0 ) {
        _dt = 1;	// default time step of 1
    }
    	
    if( _t.size() == 0 ) {
	_t.resize( n );
        for( size_t i = 0; i < n; ++i ) {
            _t[i] = i*_dt;
	}
    }

    //-- Print
    FILE *pPlotData = fopen( _name.c_str(), "w" );
   
    for( size_t i = 0; i < n; ++i ) {
	fprintf( pPlotData, "%.3f", _t[i] );
	for( size_t j = 0; j < m; ++j ) {
	    fprintf( pPlotData, " %.3f", _var[i][j] );
	}
	fprintf( pPlotData, "\n" );
    }	

    fclose( pPlotData );	

}

