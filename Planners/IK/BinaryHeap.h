/**
 * @file BinaryHeap.h
 * @brief 
 */

#ifndef _BINARY_HEAP_H_
#define _BINARY_HEAP_H_

#include <vector>

/**
 * @function Heap_Insert
 */
int Heap_Insert( int _index, 
		 std::vector<int> &_heap,
		 std::vector<double> _vals );

int Heap_Pop( std::vector<int> &_heap,
	      std::vector<double> _vals );


#endif /** _BINARY_HEAP_H_ */
