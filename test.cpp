#include <limits>
#include "cinder/CinderAssert.h"
#include "cinder/Rand.h"
#include "cinder/Vector.h"
#include "SpacePartitioning.h"

using namespace ci;
using namespace std;

template<class Structure, class VecT>
void test( const std::string &name, Structure *structure, const vector<VecT> &dataSet, size_t numNeighbors, const VecT &nn )
{
	Timer timer;
	
	// insert elements
	timer.start();
	for( auto p : dataSet ) {
		structure->insert( p );
	}
	timer.stop();
	cout << name << endl << endl;
	// evaluate the memory footprint
	cout << "\tMemory Footprint \t\t\t" << structure->size() / 1048576.0f << "mb" << endl;
	// print out the time to insert everything
	cout << "\tInserting " << dataSet.size() << " objects \t" << timer.getSeconds() * 1000 << "ms" << endl;
	
	// try a range search
	timer.start();
	auto nearestN = structure->rangeSearch( VecT(0), 10 );
	timer.stop();
	cout << "\tRange Search \t\t\t\t" << timer.getSeconds() * 1000 << "ms \t\t" << nearestN.size() << " neighbors" << endl;
	CI_ASSERT( nearestN.size() == numNeighbors );
	
	// try the same range search but with a lambda
	timer.start();
	int lambdaNeighbors = 0;
	structure->rangeSearch( VecT(0), 10, [&]( typename Structure::Node *node, float distSq ) { lambdaNeighbors++; } );
	timer.stop();
	cout << "\tRange Search lambda \t\t\t" << timer.getSeconds() * 1000 << "ms \t\t" << lambdaNeighbors << " neighbors" << endl;
	CI_ASSERT( lambdaNeighbors == numNeighbors );
	
	// try a nearest neighbor search
	timer.start();
	auto nearest = structure->nearestNeighborSearch( VecT(0) );
	timer.stop();
	cout << "\tNearest Search \t\t\t\t" << timer.getSeconds() * 1000 << "ms \t\t" << nearest->getPosition() << endl;
	CI_ASSERT( glm::all( glm::equal( nn, nearest->getPosition() ) ) );
	
	// clear the spatial partitioning structure
	timer.start();
	structure->clear();
	timer.stop();
	cout << "\tClearing structure \t\t\t" << timer.getSeconds() * 1000 << "ms" << endl;
	cout << "_____________________________________________________________________________________________" << endl << endl;
	
	// release structure
	delete structure;
}

template<typename VecT>
void reference( const std::string &name, const vector<VecT> &dataSet, size_t *numNeighbors, VecT *nn )
{
	cout << endl << endl << name << endl << endl;
	Timer timer;
	
	// brute force range search
	timer.start();
	float radiusSq = 10*10, distSq;
	vector<pair<VecT,float>> bruteNearests;
	for( auto p : dataSet ) {
		distSq = glm::distance2( p, VecT(0) );
		if( distSq < radiusSq ) {
			bruteNearests.push_back( make_pair( p, distSq ) );
		}
	}
	timer.stop();
	cout << "\tRange search \t\t\t" << timer.getSeconds() * 1000 << "ms \t\t" << bruteNearests.size() << " neighbors" << endl;
	*numNeighbors = bruteNearests.size();
	
	// brute force nearest neighbor search
	timer.start();
	VecT nearest;
	float minDist = numeric_limits<float>::max();
	for( auto p : dataSet ) {
		distSq = glm::distance2( p, VecT(0) );
		if( distSq < minDist ) {
			minDist = distSq;
			nearest = p;
		}
	}
	timer.stop();
	cout << "\tNearest search \t\t\t" << timer.getSeconds() * 1000 << "ms \t\t" << nearest << endl;
	*nn = nearest;
	cout << "_____________________________________________________________________________________________" << endl << endl;
}

int main( int argc, char* argv[] )
{
	// init data set
	vec2 min2d = vec2( numeric_limits<float>::max() ), max2d = vec2( numeric_limits<float>::min() );
	vec3 min3d = vec3( numeric_limits<float>::max() ), max3d = vec3( numeric_limits<float>::min() );
	vector<vec2> dataSet2d;
	vector<vec3> dataSet3d;
	for( int i = 0; i < 1000000; i++ ) {
		vec2 p2d = vec2( randFloat(), randFloat() ) * 200.0f;
		vec3 p3d = vec3( randFloat(), randFloat(), randFloat() ) * 200.0f;
		dataSet2d.push_back( p2d );
		dataSet3d.push_back( p3d );
		min2d = glm::min( min2d, p2d );
		max2d = glm::max( max2d, p2d );
		min3d = glm::min( min3d, p3d );
		max3d = glm::max( max3d, p3d );
	}
	
	cout << "2d set bounds " << min2d << " " << max2d << endl;
	cout << "3d set bounds " << min3d << " " << max3d << endl;
	
	// 2d Tests
	vec2 nearest2d;
	size_t neighbors2d;
	reference<vec2>( "2d Brute Force", dataSet2d, &neighbors2d, &nearest2d );
	test<sp::KdTree2,vec2>( "KdTree2", new sp::KdTree2(), dataSet2d, neighbors2d, nearest2d );
	test<sp::Grid2,vec2>( "Grid2 k = 1", new sp::Grid2( min2d, max2d, 1 ), dataSet2d, neighbors2d, nearest2d );
	test<sp::Grid2,vec2>( "Grid2 k = 2", new sp::Grid2( min2d, max2d, 2 ), dataSet2d, neighbors2d, nearest2d );
	test<sp::Grid2,vec2>( "Grid2 k = 3", new sp::Grid2( min2d, max2d, 3 ), dataSet2d, neighbors2d, nearest2d );
	test<sp::Grid2,vec2>( "Grid2 k = 4", new sp::Grid2( min2d, max2d, 4 ), dataSet2d, neighbors2d, nearest2d );
	test<sp::Grid2,vec2>( "Unbounded-Grid2 k = 1", new sp::Grid2( 1 ), dataSet2d, neighbors2d, nearest2d );
	
	// HashTable needs more work
	/*test<sp::HashTable2,vec2>( "HashTable2 c = vec2( 10 ) s = 7500", new sp::HashTable2( min2d, max2d, vec2( 10 ), 7500 ), dataSet2d, neighbors2d, nearest2d );
	test<sp::HashTable2,vec2>( "HashTable2 c = vec2( 10 ) s = 7900", new sp::HashTable2( min2d, max2d, vec2( 10 ), 7900 ), dataSet2d, neighbors2d, nearest2d );
	test<sp::HashTable2,vec2>( "HashTable2 c = vec2( 10 ) s = 8000", new sp::HashTable2( min2d, max2d, vec2( 10 ), 8000 ), dataSet2d, neighbors2d, nearest2d );
	test<sp::HashTable2,vec2>( "HashTable2 c = vec2( 10 ) s = 8100", new sp::HashTable2( min2d, max2d, vec2( 10 ), 8100 ), dataSet2d, neighbors2d, nearest2d );
	test<sp::HashTable2,vec2>( "HashTable2 c = vec2( 10 ) s = 8500", new sp::HashTable2( min2d, max2d, vec2( 10 ), 8500 ), dataSet2d, neighbors2d, nearest2d );*/
	
	// 3d Tests
	vec3 nearest3d;
	size_t neighbors3d;
	reference<vec3>( "3d Brute Force", dataSet3d, &neighbors3d, &nearest3d );
	test<sp::KdTree3,vec3>( "KdTree3", new sp::KdTree3(), dataSet3d, neighbors3d, nearest3d );
	test<sp::Grid3,vec3>( "Grid3 k = 1", new sp::Grid3( min3d, max3d, 1 ), dataSet3d, neighbors3d, nearest3d );
	test<sp::Grid3,vec3>( "Grid3 k = 2", new sp::Grid3( min3d, max3d, 2 ), dataSet3d, neighbors3d, nearest3d );
	test<sp::Grid3,vec3>( "Grid3 k = 3", new sp::Grid3( min3d, max3d, 3 ), dataSet3d, neighbors3d, nearest3d );
	test<sp::Grid3,vec3>( "Grid3 k = 4", new sp::Grid3( min3d, max3d, 4 ), dataSet3d, neighbors3d, nearest3d );
	test<sp::Grid3,vec3>( "Unbounded-Grid3 k = 3", new sp::Grid3( 3 ), dataSet3d, neighbors3d, nearest3d );
	
	return 0;
}
