## SpacePartitioning
A collection of c++11 Space Partitioning templates for Cinder

This is a work in progress; currently only ```KdTree``` and ```Bin-lattice / Grid``` are available, but ```HashTable```, ```Octree``` and ```BSPTree``` are on their way.

**Basic Use:**  
Each classes share the same syntax for common tasks.

```c++
// Create a 3d KdTree (use alias for KdTree<3,float>)
sp::KdTree3 kdTree; 

// Insert elements
for( auto node : mNodes ) {
	const vec3& pos = node->getPosition();
	void* userData 	= static_cast<void*>( node );
	kdTree.insert( pos, userData );
}

// Nearest neighbor search
auto nn = kdTree.nearestNeighborSearch( vec3( 0.0f ) );

// Nearest neighbor search and get the distance square to the position
float distSq;
nn = kdTree.nearestNeighborSearch( vec3( 1.0f, 2.0f, 3.0f ), &distSq );

// Range search
auto neighbors = kdTree.rangeSearch( vec3( 0.0f ), 35.0f );
for( auto neighbor : neighbors ) {
	auto node = neighbor.first;
	auto distSq = neighbor.second;
	// ...
}

// Visitor Range search
auto neighbors = kdTree.rangeSearch( vec3( 0.0f ), 35.0f, 
	[]( KdTree2::Node* node, float distSq ) {
		// ...
	} );

```

**KdTree:**

A KdTree is a versatile space partitioning structure. They work in most situations and are really fast for nearest neighbor searches. If memory or insertion time is important they might not be the right choice. This implementation is not particulary optimised for range searches.

```KdTrees``` come in the following flavors :
```c++
typedef KdTree<2, float>	KdTree2;
typedef KdTree<3, float>	KdTree3;
typedef KdTree<4, float>	KdTree4;
typedef KdTree<2, double>	dKdTree2;
typedef KdTree<3, double>	dKdTree3;
typedef KdTree<4, double>	dKdTree4;
```

**Grid:**

A Bin-Lattice spatial subdivision structure or uniform grid is easy and particulary fast. They clearly excel at range searches, usually have a small memory footprint and have fast insertion time. When it's not clear which structure to choose, Grid might be the first one to try. The choice of the ```k``` parameter is critical and might influence a lot the performance depending on the data. Providing the bounding rectangle or bounding box of the data is prefered if you need fast insertion. If you ommit the bounds parameters, insertion will obviously be much slower.

```Grids``` come in the following flavors :
```c++
typedef Grid<2, float>		Grid2;
typedef Grid<3, float>		Grid3;
typedef Grid<2, double>		dGrid2;
typedef Grid<3, double>		dGrid3;
```

**Test suite:**  
Running [```test.cpp```](test.cpp) should output something along those lines. It might be usefull to run these tests with your own data as the results might changes drastically according its type and distribution.

```
2d Brute Force

	Range search 				3.22801ms 		1990 neighbors
	Nearest search 				2.22701ms 		[    0.055,    0.027]
_____________________________________________________________________________________________

KdTree2

	Memory Footprint 			38.147mb
	Inserting 1000000 objects 	731.91ms
	Range Search 				0.497997ms 		1990 neighbors
	Range Search lambda 		0.222027ms 		1990 neighbors
	Nearest Search 				0.00196695ms 	[    0.055,    0.027]
	Clearing structure 			119.747ms
_____________________________________________________________________________________________

Grid2 k = 1

	Memory Footprint 			15.4923mb
	Inserting 1000000 objects 	93.623ms
	Range Search 				0.102997ms 		1990 neighbors
	Range Search lambda 		0.0450015ms 	1990 neighbors
	Nearest Search 				0.0119805ms 	[    0.055,    0.027]
	Clearing structure 			86.762ms
_____________________________________________________________________________________________

Grid2 k = 2

	Memory Footprint 			15.3184mb
	Inserting 1000000 objects 	116.088ms
	Range Search 				0.102997ms 		1990 neighbors
	Range Search lambda 		0.0609756ms 	1990 neighbors
	Nearest Search 				0.00995398ms 	[    0.055,    0.027]
	Clearing structure 			87.708ms
_____________________________________________________________________________________________

Grid2 k = 3

	Memory Footprint 			15.2743mb
	Inserting 1000000 objects 	117.888ms
	Range Search 				0.140011ms 		1990 neighbors
	Range Search lambda 		0.115037ms 		1990 neighbors
	Nearest Search 				0.145972ms 		[    0.055,    0.027]
	Clearing structure 			92.24ms
_____________________________________________________________________________________________

Grid2 k = 4

	Memory Footprint 			15.2627mb
	Inserting 1000000 objects 	120.338ms
	Range Search 				0.141025ms 		1990 neighbors
	Range Search lambda 		0.113964ms 		1990 neighbors
	Nearest Search 				0.118971ms 		[    0.055,    0.027]
	Clearing structure 			92.325ms
_____________________________________________________________________________________________

Unbounded-Grid2 k = 1

	Memory Footprint 			15.4923mb
	Inserting 1000000 objects 	297.285ms
	Range Search 				0.102997ms 		1990 neighbors
	Range Search lambda 		0.0700355ms 	1990 neighbors
	Nearest Search 				0.0090003ms 	[    0.055,    0.027]
	Clearing structure 			90.626ms
_____________________________________________________________________________________________



3d Brute Force

	Range search 				1.97297ms 		73 neighbors
	Nearest search 				1.90198ms 		[    0.103,    0.016,    1.801]
_____________________________________________________________________________________________

KdTree3

	Memory Footprint 			38.147mb
	Inserting 1000000 objects 	1009.77ms
	Range Search 				0.0450015ms 		73 neighbors
	Range Search lambda 		0.0160336ms 		73 neighbors
	Nearest Search 				0.00196695ms 		[    0.103,    0.016,    1.801]
	Clearing structure 			128.187ms
_____________________________________________________________________________________________

Grid3 k = 1

	Memory Footprint 			46.47mb
	Inserting 1000000 objects 	281.196ms
	Range Search 				0.0350475ms 		73 neighbors
	Range Search lambda 		0.00798702ms 		73 neighbors
	Nearest Search 				0.00202656ms 		[    0.103,    0.016,    1.801]
	Clearing structure 			117.986ms
_____________________________________________________________________________________________

Grid3 k = 2

	Memory Footprint 			25.9244mb
	Inserting 1000000 objects 	277.803ms
	Range Search 				0.0240207ms 		73 neighbors
	Range Search lambda 		0.00399351ms 		73 neighbors
	Nearest Search 				0.00303984ms 		[    0.103,    0.016,    1.801]
	Clearing structure 			95.844ms
_____________________________________________________________________________________________

Grid3 k = 3

	Memory Footprint 			23.2906mb
	Inserting 1000000 objects 	102.613ms
	Range Search 				0.0320077ms 		73 neighbors
	Range Search lambda 		0.0100136ms 		73 neighbors
	Nearest Search 				0.00298023ms 		[    0.103,    0.016,    1.801]
	Clearing structure 			98.442ms
_____________________________________________________________________________________________

Grid3 k = 4

	Memory Footprint 			22.9386mb
	Inserting 1000000 objects 	151.011ms
	Range Search 				0.048995ms 			73 neighbors
	Range Search lambda 		0.0529885ms 		73 neighbors
	Nearest Search 				0.0169873ms 		[    0.103,    0.016,    1.801]
	Clearing structure 			101.531ms
_____________________________________________________________________________________________

Unbounded-Grid3 k = 3

	Memory Footprint 			23.2906mb
	Inserting 1000000 objects 	378.761ms
	Range Search 				0.050962ms 			73 neighbors
	Range Search lambda 		0.0370145ms 		73 neighbors
	Nearest Search 				0.0370145ms 		[    0.103,    0.016,    1.801]
	Clearing structure 			97.918ms
_____________________________________________________________________________________________

```