/**
 * @file BinaryHeap.cpp
 * @author A. Huaman
 * @date 2012-06-25
 * @brief Adapted from Research/LJM2/MyFunctions/Dijkstra.cpp. If you need to lower the key value you have to use the HT (Hash Table), which I did not add here but is in the original file. For big arrays it is important in speed terms to use HT! (only if you need to lower the keys)
 */

#include <stdio.h>
#include "BinaryHeap.h"

/**
 * @function Heap_Insert
 */
int Heap_Insert( int _index, 
		 std::vector<int> &_heap,
		 std::vector<double> _vals ) {
  
  int n; 
  int node; int parent;
  int temp;

  _heap.push_back( _index );
  n = _heap.size() - 1;

  // If this is the first element added
  if( n == 0 )
    { return 0; }

  // If not, start on the bottom and go up
  node = n;

  int qp; int qn;

  while( node != 0 )
  {
    //parent = floor( (node - 1)/2 );
    parent = (node - 1)/2 ;
    qp = _heap[parent]; qn = _heap[node];   

    if( _vals[ qp ] > _vals[ qn ] ) // IT WAS >= should not change
      {
        temp = qp;
        _heap[parent] = qn;  
        _heap[node] = temp; 
        node = parent; 
      }  
    else
     { break; }
  }  

  return node;
}

/**
 * @function Heap_Pop
 */
int Heap_Pop( std::vector<int> &_heap,
	      std::vector<double> _vals ) {

  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  int temp;

  if( _heap.size() == 0 )
    { printf("--(!) [Heap_Pop] No more elements left -- return index (-1)\n");return -1; }

  // Save the pop-out element
  first = _heap[0];
  
  // Reorder your binary heap
  bottom = _heap.size() - 1;

  _heap[0] = _heap[bottom];
  _heap.pop_back();
  n = _heap.size();

  int u = 0;

  int qu;

  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( _vals[ _heap[node] ] >= _vals[ _heap[child_1] ] )
        { u = child_1;  }
       if( _vals[ _heap[u] ]  >= _vals[ _heap[child_2] ] )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( _vals[ _heap[node] ] >= _vals[ _heap[child_1] ] )
         { u = child_1; }
     }
     
    qu = _heap[u];    
    if( node != u )
     { temp = _heap[node]; 
       _heap[node] = qu;
       _heap[u] = temp;
     }

    else
     { break; } 
  }

  return first;

}
