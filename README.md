## SpacePartitioning
A collection of c++11 Space Partitioning templates for Cinder

- [x] KdTree
- [x] Bin-lattice / Grid
- [ ] HashTable
- [ ] OctTree
- [ ] BSPTree

Each classes share the same syntax for common tasks :

```c++
// Create a 2d KdTree
KdTree2 kdTree;

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