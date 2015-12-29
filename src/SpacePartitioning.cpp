/*
 Space Partitioning algorithms for Cinder
 
 Copyright (c) 2015, Simon Geilfus, All rights reserved.
 This code is intended for use with the Cinder C++ library: http://libcinder.org
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:
 
 * Redistributions of source code must retain the above copyright notice, this list of conditions and
	the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
	the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */

#include "SpacePartitioning.h"
#include "cinder/Utilities.h"
#include <limits>

namespace SpacePartitioning {
	
// MARK: KDTree
	
// https://en.wikipedia.org/wiki/K-d_tree
// https://github.com/jtsiomb/kdtree
// https://github.com/mikolalysenko/static-kdtree
// http://www.flipcode.com/archives/Raytracing_Topics_Techniques-Part_7_Kd-Trees_and_More_Speed.shtml
	
// KdTree::HyperRect
template<uint8_t DIM, class T>
KdTree<DIM,T>::HyperRect::HyperRect()
: mMin( std::numeric_limits<T>::max() ), mMax( std::numeric_limits<T>::min() )
{
}
template<uint8_t DIM, class T>
KdTree<DIM,T>::HyperRect::HyperRect( const vec_t &min, const vec_t &max )
: mMin( min ), mMax( max )
{
}
template<uint8_t DIM, class T>
void KdTree<DIM,T>::HyperRect::extend( const vec_t &position )
{
	mMin = glm::min( mMin, position );
	mMax = glm::max( mMax, position );
}
template<uint8_t DIM, class T>
T KdTree<DIM,T>::HyperRect::distance2( const vec_t &position ) const
{
	T result = 0;
	for( size_t axis = 0; axis < DIM; ++axis ) {
		if( position[axis] < mMin[axis] ) {
			result += glm::distance2( mMin[axis], position[axis] );
		}
		else if( position[axis] > mMax[axis] ) {
			result += glm::distance2( mMax[axis], position[axis] );
		}
	}
	return result;
}

// KdTree::Node
template<uint8_t DIM, class T>
KdTree<DIM,T>::Node::Node( const vec_t &position, int axis, void *data )
: mPosition( position ), mAxis( axis ), mData( data ), mLeft( nullptr ), mRight( nullptr )
{
}
template<uint8_t DIM, class T>
KdTree<DIM,T>::Node::~Node()
{
	if( mLeft ) {
		delete mLeft;
		mLeft = nullptr;
	}
	if( mRight ) {
		delete mRight;
		mRight = nullptr;
	}
}

// KdTree
template<uint8_t DIM, class T>
KdTree<DIM,T>::KdTree()
: mRoot( nullptr )
{
}
template<uint8_t DIM, class T>
KdTree<DIM,T>::~KdTree()
{
	if( mRoot ) {
		delete mRoot;
		mRoot = nullptr;
	}
}

template<uint8_t DIM, class T>
void KdTree<DIM,T>::insertImpl( Node **nptr, const vec_t &position, void *data, int axis )
{
	using Node_t = typename KdTree<DIM,T>::Node;
	
	Node_t* node = *nptr;
	if( ! *nptr ) {
		node = new Node_t( position, axis, data );
		*nptr = node;
		return;
	}
	int newDir = ( node->mAxis + 1 ) % DIM;
	if( position[node->mAxis] < node->mPosition[node->mAxis] ) {
		insertImpl( &(*nptr)->mLeft, position, data, newDir );
		return;
	}
	insertImpl( &(*nptr)->mRight, position, data, newDir );
}

template<uint8_t DIM, class T>
void KdTree<DIM,T>::insert( const vec_t &position, void *data )
{
	insertImpl( &mRoot, position, data, 0 );
	mHyperRect.extend( position );
}
template<uint8_t DIM, class T>
void KdTree<DIM,T>::clear()
{
	if( mRoot ) {
		delete mRoot;
		mRoot = nullptr;
	}
}

template<uint8_t DIM, class T>
size_t KdTree<DIM,T>::sizeImpl( Node *node ) const
{
	size_t size = 0;
	if( node ) {
		size += sizeof( *node );
		if( node->mLeft )
			size += sizeImpl( node->mLeft );
		if( node->mRight )
			size += sizeImpl( node->mRight );
	}
	return size;
}

template<uint8_t DIM, class T>
size_t KdTree<DIM,T>::size() const
{
	return sizeof( *this ) + sizeImpl( mRoot );
}
	
template<uint8_t DIM, class T>
typename KdTree<DIM,T>::Node* KdTree<DIM,T>::nearestNeighborSearch( const vec_t &position, T *distanceSq ) const
{
	Node* result	= mRoot;
	T dSq			= glm::distance2( position, result->mPosition );
	HyperRect rect	= mHyperRect;
	nearestNeighborSearchImpl( mRoot, &rect, position, &result, &dSq );
	if( distanceSq )
		*distanceSq = dSq;
	
	return result;
}

template<uint8_t DIM, class T>
void KdTree<DIM,T>::nearestNeighborSearchImpl( Node *node, HyperRect *rect, const vec_t &position, Node **result, T *resultDistanceSq ) const
{
	Node* nearestNode;
	Node* furthestNode;
	T* nearestSplit;
	T* furthestSplit;
	
	// if left is closer
	if( position[node->mAxis] - node->mPosition[node->mAxis] <= 0 ) {
		nearestNode = node->mLeft;
		furthestNode = node->mRight;
		nearestSplit = &rect->mMax[node->mAxis];
		furthestSplit = &rect->mMin[node->mAxis];
	}
	else { // if right is closer
		nearestNode = node->mRight;
		furthestNode = node->mLeft;
		nearestSplit = &rect->mMin[node->mAxis];
		furthestSplit = &rect->mMax[node->mAxis];
	}
	
	// recursively search into the nearest sub-tree
	if( nearestNode ) {
		T temp = *nearestSplit;
		*nearestSplit = node->mPosition[node->mAxis];
		nearestNeighborSearchImpl( nearestNode, rect, position, result, resultDistanceSq );
		*nearestSplit = temp;
	}
	
	// update distances
	T distanceSq = glm::distance2( node->mPosition, position );
	if( distanceSq < *resultDistanceSq ) {
		*result = node;
		*resultDistanceSq = distanceSq;
	}
	
	// recursively search into the furthest sub-tree
	if( furthestNode ) {
		T temp = *furthestSplit;
		*furthestSplit = node->mPosition[node->mAxis];
		// check if we still need to go down the furthest sub-tree
		if( rect->distance2( position ) < *resultDistanceSq ) {
			nearestNeighborSearchImpl( furthestNode, rect, position, result, resultDistanceSq );
		}
		*furthestSplit = temp;
	}
}

template<uint8_t DIM, class T>
std::vector<typename KdTree<DIM,T>::NodePair> KdTree<DIM,T>::rangeSearch( const vec_t &position, T radius ) const
{
	std::vector<NodePair> results;
	rangeSearchImpl( mRoot, position, radius, &results );
	return results;
}
template<uint8_t DIM, class T>
void KdTree<DIM,T>::rangeSearchImpl( Node *node, const vec_t &position, T radius, std::vector<typename KdTree<DIM,T>::NodePair> *results ) const
{
	if( ! node )
		return;
	
	// if node is within the range add it to the results
	T distanceSq = glm::distance2( node->mPosition, position );
	if( distanceSq <= radius * radius ) {
		results->emplace_back( std::make_pair( node, distanceSq ) );
	}
	
	// recursively check the first child
	T dx = position[node->mAxis] - node->mPosition[node->mAxis];
	rangeSearchImpl( dx <= 0.0 ? node->mLeft : node->mRight, position, radius, results );
	// check if we still need to go down the other child
	if( glm::abs( dx ) < radius ) {
		rangeSearchImpl(dx <= 0.0 ? node->mRight : node->mLeft, position, radius, results );
	}
}

template<uint8_t DIM, class T>
void KdTree<DIM,T>::rangeSearch( const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const
{
	rangeSearchImpl( mRoot, position, radius, visitor );
}
template<uint8_t DIM, class T>
void KdTree<DIM,T>::rangeSearchImpl( Node *node, const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const
{
	if( ! node )
		return;
	
	// if node is within the range add it to the results
	T distanceSq = glm::distance2( node->mPosition, position );
	if( distanceSq <= radius * radius ) {
		visitor( node, distanceSq );
	}
	
	// recursively check the first child
	T dx = position[node->mAxis] - node->mPosition[node->mAxis];
	rangeSearchImpl( dx <= 0.0 ? node->mLeft : node->mRight, position, radius, visitor );
	// check if we still need to go down the other child
	if( glm::abs( dx ) < radius ) {
		rangeSearchImpl(dx <= 0.0 ? node->mRight : node->mLeft, position, radius, visitor );
	}
}

// explicit template instantiations
template class KdTree<2, float>;
template class KdTree<3, float>;
template class KdTree<4, float>;
template class KdTree<2, double>;
template class KdTree<3, double>;
template class KdTree<4, double>;
	
	
// MARK: Grid
	
// http://www.red3d.com/cwr/papers/2000/pip.pdf
// https://en.wikipedia.org/wiki/Regular_grid
	
template<uint8_t DIM, class T> struct GridTraits {};
template<class T>
struct GridTraits<2,T> {
	static typename Grid<2,T>::ivec_t toGridPosition( const typename Grid<2,T>::vec_t &position, const typename Grid<2,T>::vec_t &offset, uint32_t k ) {
		return typename Grid<2,T>::ivec_t( static_cast<uint32_t>( position.x + offset.x ) >> k,
										  static_cast<uint32_t>( position.y + offset.y ) >> k );
	}
	static uint32_t toIndex( const typename Grid<2,T>::ivec_t &gridPos, const typename Grid<2,T>::ivec_t &numCells ) {
		return gridPos.x + numCells.x * gridPos.y;
	}
	static uint32_t toIndex( const typename Grid<2,T>::vec_t &position, const typename Grid<2,T>::vec_t &offset, const typename Grid<2,T>::ivec_t &numCells, uint32_t k ) {
		typename Grid<2,T>::ivec_t gridPos = toGridPosition( position, offset, k );
		return gridPos.x + numCells.x * gridPos.y;
	}
	static uint32_t gridSize( const typename Grid<2,T>::vec_t &numCells )
	{
		return numCells.x * numCells.y;
	}
	static void rangeSearch( std::vector<typename Grid<2,T>::NodePair> *results, const typename Grid<2,T>::Vector &grid, const typename Grid<2,T>::vec_t &position, T radius, const typename Grid<2,T>::ivec_t &minCell, const typename Grid<2,T>::ivec_t &maxCell, const typename Grid<2,T>::ivec_t &numCells )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename Grid<2,T>::ivec_t pos;
		for( pos.y = minCell.y; pos.y < maxCell.y; pos.y++ ) {
			for( pos.x = minCell.x; pos.x < maxCell.x; pos.x++ ) {
				const std::vector<typename Grid<2,T>::Node*>& cell = grid[GridTraits<2,T>::toIndex( pos, numCells )];
				for( const auto& node : cell ) {
					distSq = glm::distance2( position, node->getPosition() );
					if( distSq < radiusSq ) {
						results->emplace_back( std::make_pair( node, distSq ) );
					}
				}
			}
		}
	}
	static void rangeSearch( const std::function<void(typename Grid<2,T>::Node*,T)> &visitor, const typename Grid<2,T>::Vector &grid, const typename Grid<2,T>::vec_t &position, T radius, const typename Grid<2,T>::ivec_t &minCell, const typename Grid<2,T>::ivec_t &maxCell, const typename Grid<2,T>::ivec_t &numCells )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename Grid<2,T>::ivec_t pos;
		for( pos.y = minCell.y; pos.y < maxCell.y; pos.y++ ) {
			for( pos.x = minCell.x; pos.x < maxCell.x; pos.x++ ) {
				const std::vector<typename Grid<2,T>::Node*>& cell = grid[GridTraits<2,T>::toIndex( pos, numCells )];
				for( const auto& node : cell ) {
					distSq = glm::distance2( position, node->getPosition() );
					if( distSq < radiusSq ) {
						visitor( node, distSq );
					}
				}
			}
		}
	}
};
template<class T>
struct GridTraits<3,T> {
	static typename Grid<3,T>::ivec_t toGridPosition( const typename Grid<3,T>::vec_t &position, const typename Grid<3,T>::vec_t &offset, uint32_t k ) {
		return typename Grid<3,T>::ivec_t( static_cast<uint32_t>( position.x + offset.x ) >> k,
										  static_cast<uint32_t>( position.y + offset.y ) >> k,
										  static_cast<uint32_t>( position.z + offset.z ) >> k );
	}
	static uint32_t toIndex( const typename Grid<3,T>::ivec_t &gridPos, const typename Grid<3,T>::ivec_t &numCells )
	{
		return gridPos.x + numCells.x * ( gridPos.y + numCells.y * gridPos.z );
	}
	static uint32_t toIndex( const typename Grid<3,T>::vec_t &position, const typename Grid<3,T>::vec_t &offset, const typename Grid<3,T>::ivec_t &numCells, uint32_t k )
	{
		typename Grid<3,T>::ivec_t gridPos = toGridPosition( position, offset, k );
		return gridPos.x + numCells.x * ( gridPos.y + numCells.y * gridPos.z );
	}
	static uint32_t gridSize( const typename Grid<3,T>::vec_t &numCells )
	{
		return numCells.x * numCells.y * numCells.z;
	}
	static void rangeSearch( std::vector<typename Grid<3,T>::NodePair> *results, const typename Grid<3,T>::Vector &grid, const typename Grid<3,T>::vec_t &position, T radius, const typename Grid<3,T>::ivec_t &minCell, const typename Grid<3,T>::ivec_t &maxCell, const typename Grid<3,T>::ivec_t &numCells )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename Grid<3,T>::ivec_t pos;
		for( pos.z = minCell.z; pos.z < maxCell.z; pos.z++ ) {
			for( pos.y = minCell.y; pos.y < maxCell.y; pos.y++ ) {
				for( pos.x = minCell.x; pos.x < maxCell.x; pos.x++ ) {
					const std::vector<typename Grid<3,T>::Node*>& cell = grid[GridTraits<3,T>::toIndex( pos, numCells )];
					for( const auto& node : cell ) {
						distSq = glm::distance2( position, node->getPosition() );
						if( distSq < radiusSq ) {
							results->emplace_back( std::make_pair( node, distSq ) );
						}
					}
				}
			}
		}
	}
	static void rangeSearch( const std::function<void(typename Grid<3,T>::Node*,T)> &visitor, const typename Grid<3,T>::Vector &grid, const typename Grid<3,T>::vec_t &position, T radius, const typename Grid<3,T>::ivec_t &minCell, const typename Grid<3,T>::ivec_t &maxCell, const typename Grid<3,T>::ivec_t &numCells )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename Grid<3,T>::ivec_t pos;
		for( pos.z = minCell.z; pos.z < maxCell.z; pos.z++ ) {
			for( pos.y = minCell.y; pos.y < maxCell.y; pos.y++ ) {
				for( pos.x = minCell.x; pos.x < maxCell.x; pos.x++ ) {
					const std::vector<typename Grid<3,T>::Node*>& cell = grid[GridTraits<3,T>::toIndex( pos, numCells )];
					for( const auto& node : cell ) {
						distSq = glm::distance2( position, node->getPosition() );
						if( distSq < radiusSq ) {
							visitor( node, distSq );
						}
					}
				}
			}
		}
	}
};
	
template<uint8_t DIM, class T>
Grid<DIM,T>::Node::Node( const vec_t &position, void *data )
: mPosition( position ), mData( data )
{
}
	
template<uint8_t DIM, class T>
Grid<DIM,T>::Grid( uint32_t k )
{
	resize( vec_t(0), vec_t(1), k );
}
template<uint8_t DIM, class T>
Grid<DIM,T>::Grid( const vec_t &min, const vec_t &max, uint32_t k )
{
	resize( min, max, k );
}
template<uint8_t DIM, class T>
Grid<DIM,T>::~Grid(){
	clear();
}
	
template<uint8_t DIM, class T>
void Grid<DIM,T>::insert( const vec_t &position, void *data )
{
	// Check if it fits the size of the grid's container
	if( glm::any( glm::greaterThan( position, mMax ) ) || glm::any( glm::lessThan( position, mMin ) ) )
		resize( glm::min( position, mMin ), glm::max( position, mMax ) );
	// Convert the position to 1D index
	uint32_t j = GridTraits<DIM,T>::toIndex( position, mOffset, mNumCells, mK );
	// And try to insert it in the grid
	if( j >= 0 && j < mGrid.size() )
		mGrid[j].push_back( new Node( position, data ) );
	else
		throw GridOutOfBoundsException( ci::toString( position ) );
}
template<uint8_t DIM, class T>
void Grid<DIM,T>::insert( Node *node )
{
	// Check if it fits the size of the grid's container
	if( glm::any( glm::greaterThan( node->getPosition(), mMax ) ) || glm::any( glm::lessThan( node->getPosition(), mMin ) ) )
		resize( glm::min( node->getPosition(), mMin ), glm::max( node->getPosition(), mMax ) );
	// Convert the position to 1D index
	uint32_t j = GridTraits<DIM,T>::toIndex( node->getPosition(), mOffset, mNumCells, mK );
	// And try to insert it in the grid
	if( j >= 0 && j < mGrid.size() )
		mGrid[j].push_back( node );
	else
		throw GridOutOfBoundsException( ci::toString( node->getPosition() ) );
}
template<uint8_t DIM, class T>
void Grid<DIM,T>::clear()
{
	for( auto& cell : mGrid ) {
		for( auto& node : cell ) {
			if( node )
				delete node;
		}
		cell.clear();
	}
}
template<uint8_t DIM, class T>
size_t Grid<DIM,T>::size() const
{
	size_t size = sizeof( *this );
	for( const auto& cell : mGrid ) {
		size += sizeof( cell ) + sizeof(Node) * cell.size();
	}
	return size;
}
// TODO: Must be a better way to do this
template<uint8_t DIM, class T>
typename Grid<DIM,T>::Node* Grid<DIM,T>::nearestNeighborSearch( const vec_t &position, T *distanceSq ) const
{
	// Grow search radius until found something
	// TODO: !!! Might grow forever !!!
	std::vector<typename Grid<DIM,T>::NodePair> results;
	T cellSize = static_cast<T>( mCellSize );
	while( ! results.size() ) {
		results = rangeSearch( position, cellSize );
		cellSize *= 2;
	}
	
	// Once we have nodes to look at, iterate and find the closest one
	Node* nearestNode = nullptr;
	T minDist = std::numeric_limits<T>::max();
	for( const auto& node : results ) {
		if( node.second < minDist ) {
			nearestNode = node.first;
			minDist = node.second;
		}
	}
	if( distanceSq != nullptr )
		*distanceSq = minDist;
	
	return nearestNode;
}

template<uint8_t DIM, class T>
std::vector<typename Grid<DIM,T>::NodePair> Grid<DIM,T>::rangeSearch( const vec_t &position, T radius ) const
{
	vec_t radiusVec = vec_t( radius );
	vec_t min       = glm::clamp( position - radiusVec, mMin, mMax + vec_t( 1 ) );
	vec_t max       = glm::clamp( position + radiusVec, mMin, mMax + vec_t( 1 ) );
	ivec_t minCell	= glm::max( GridTraits<DIM,T>::toGridPosition( min, mOffset, mK ), ivec_t( 0 ) );
	ivec_t maxCell	= glm::min( ivec_t(1) + GridTraits<DIM,T>::toGridPosition( max, mOffset, mK ), mNumCells );
	std::vector<typename Grid<DIM,T>::NodePair> results;
	GridTraits<DIM,T>::rangeSearch( &results, mGrid, position, radius, minCell, maxCell, mNumCells );
	return results;
}
template<uint8_t DIM, class T>
void Grid<DIM,T>::rangeSearch( const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const
{
	vec_t radiusVec = vec_t( radius );
	vec_t min       = glm::clamp( position - radiusVec, mMin, mMax + vec_t( 1 ) );
	vec_t max       = glm::clamp( position + radiusVec, mMin, mMax + vec_t( 1 ) );
	ivec_t minCell	= glm::max( GridTraits<DIM,T>::toGridPosition( min, mOffset, mK ), ivec_t( 0 ) );
	ivec_t maxCell	= glm::min( ivec_t(1) + GridTraits<DIM,T>::toGridPosition( max, mOffset, mK ), mNumCells );
	GridTraits<DIM,T>::rangeSearch( visitor, mGrid, position, radius, minCell, maxCell, mNumCells );
}
	
	
template<uint8_t DIM, class T>
void Grid<DIM,T>::resize( const vec_t &min, const vec_t &max )
{
	mMin = min;
	mMax = max;
	resize( mK );
}
template<uint8_t DIM, class T>
void Grid<DIM,T>::resize( const vec_t &min, const vec_t &max, uint32_t k )
{
	mMin = min;
	mMax = max;
	resize( k );
}
template<uint8_t DIM, class T>
void Grid<DIM,T>::resize( uint32_t k )
{
	// If we have existing nodes we need to save them
	std::vector<Node*> nodes;
	for( auto& cell : mGrid ) {
		nodes.insert( nodes.end(), cell.begin(), cell.end() );
		cell.clear();
	}
	
	// Update grid settings
	mK          = k;
	mCellSize   = 1 << k;
	mOffset     = -mMin;
	mNumCells   = glm::ceil( ( ( mMax - mMin ) + vec_t( 1 ) ) / static_cast<T>( mCellSize ) );
	mGrid.resize( GridTraits<DIM,T>::gridSize( mNumCells ) );
	
	// Re-insert old nodes
	for( const auto& node : nodes ) {
		insert( node );
	}
}
	
// explicit template instantiations
template class Grid<2, float>;
template class Grid<3, float>;
template class Grid<2, double>;
template class Grid<3, double>;
	
	
// MARK: HashTable
	
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.4.5881&rep=rep1&type=pdf
// https://en.wikipedia.org/wiki/Locality-sensitive_hashing
	

template<uint8_t DIM, class T> struct HashTableTraits {};
template<class T>
struct HashTableTraits<2,T> {
	static uint32_t getHash( const typename HashTable<2,T>::vec_t &position, const typename HashTable<2,T>::vec_t &cellSize, uint32_t tableSize )
	{
		const typename HashTable<2,T>::ivec_t largePrime( 73856093, 19349663 );
		typename HashTable<2,T>::ivec_t p = glm::floor( position / cellSize );
		return ( ( p.x * largePrime.x ) ^ ( p.y * largePrime.y ) ) % tableSize;
	}
	static void rangeSearch( std::vector<typename HashTable<2,T>::NodePair> *results, const typename HashTable<2,T>::Vector &hashTable, const typename HashTable<2,T>::vec_t &position, T radius, const typename HashTable<2,T>::vec_t &minCell, const typename HashTable<2,T>::vec_t &maxCell, const typename HashTable<2,T>::vec_t &cellSize, uint32_t tableSize )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename HashTable<2,T>::vec_t pos;
		for( pos.y = minCell.y; pos.y < maxCell.y; pos.y += cellSize.y ) {
			for( pos.x = minCell.x; pos.x < maxCell.x; pos.x += cellSize.x ) {
				const std::vector<typename HashTable<2,T>::Node*>& cell = hashTable[HashTableTraits<2,T>::getHash( pos, cellSize, tableSize )];
				for( const auto& node : cell ) {
					distSq = glm::distance2( position, node->getPosition() );
					if( distSq < radiusSq ) {
						results->emplace_back( std::make_pair( node, distSq ) );
					}
				}
			}
		}
		
	}
	static void rangeSearch( const std::function<void(typename HashTable<2,T>::Node*,T)> &visitor, const typename HashTable<2,T>::Vector &hashTable, const typename HashTable<2,T>::vec_t &position, T radius, const typename HashTable<2,T>::vec_t &minCell, const typename HashTable<2,T>::vec_t &maxCell, const typename HashTable<2,T>::vec_t &cellSize, uint32_t tableSize )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename HashTable<2,T>::vec_t pos;
		for( pos.y = minCell.y; pos.y < maxCell.y; pos.y += cellSize.y ) {
			for( pos.x = minCell.x; pos.x < maxCell.x; pos.x += cellSize.x ) {
				const std::vector<typename HashTable<2,T>::Node*>& cell = hashTable[HashTableTraits<2,T>::getHash( pos, cellSize, tableSize )];
				for( const auto& node : cell ) {
					distSq = glm::distance2( position, node->getPosition() );
					if( distSq < radiusSq ) {
						visitor( node, distSq );
					}
				}
			}
		}
		
	}
};

template<class T>
struct HashTableTraits<3,T> {
	static uint32_t getHash( const typename HashTable<3,T>::vec_t &position, const typename HashTable<3,T>::vec_t &cellSize, uint32_t tableSize )
	{
		const typename HashTable<3,T>::ivec_t largePrime( 73856093, 19349663, 83492791 );
		typename HashTable<3,T>::ivec_t p = glm::floor( position / cellSize );
		return ( ( p.x * largePrime.x ) ^ ( p.y * largePrime.y ) ^ ( p.z * largePrime.z ) ) % tableSize;
	}
	static void rangeSearch( std::vector<typename HashTable<3,T>::NodePair> *results, const typename HashTable<3,T>::Vector &hashTable, const typename HashTable<3,T>::vec_t &position, T radius, const typename HashTable<3,T>::vec_t &minCell, const typename HashTable<3,T>::vec_t &maxCell, const typename HashTable<3,T>::vec_t &cellSize, uint32_t tableSize )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename HashTable<3,T>::vec_t pos;
		for( pos.z = minCell.z; pos.z < maxCell.z; pos.z += cellSize.z ) {
			for( pos.y = minCell.y; pos.y < maxCell.y; pos.y += cellSize.y ) {
				for( pos.x = minCell.x; pos.x < maxCell.x; pos.x += cellSize.x ) {
					const std::vector<typename HashTable<3,T>::Node*>& cell = hashTable[HashTableTraits<3,T>::getHash( pos, cellSize, tableSize )];
					for( const auto& node : cell ) {
						distSq = glm::distance2( position, node->getPosition() );
						if( distSq < radiusSq ) {
							results->emplace_back( std::make_pair( node, distSq ) );
						}
					}
				}
			}
		}
	}
	static void rangeSearch( const std::function<void(typename HashTable<3,T>::Node*,T)> &visitor, const typename HashTable<3,T>::Vector &hashTable, const typename HashTable<3,T>::vec_t &position, T radius, const typename HashTable<3,T>::vec_t &minCell, const typename HashTable<3,T>::vec_t &maxCell, const typename HashTable<3,T>::vec_t &cellSize, uint32_t tableSize )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename HashTable<3,T>::vec_t pos;
		for( pos.z = minCell.z; pos.z < maxCell.z; pos.z += cellSize.z ) {
			for( pos.y = minCell.y; pos.y < maxCell.y; pos.y += cellSize.y ) {
				for( pos.x = minCell.x; pos.x < maxCell.x; pos.x += cellSize.x ) {
					const std::vector<typename HashTable<3,T>::Node*>& cell = hashTable[HashTableTraits<3,T>::getHash( pos, cellSize, tableSize )];
					for( const auto& node : cell ) {
						distSq = glm::distance2( position, node->getPosition() );
						if( distSq < radiusSq ) {
							visitor( node, distSq );
						}
					}
				}
			}
		}
	}
};
	
template<uint8_t DIM, class T>
HashTable<DIM,T>::Node::Node( const vec_t &position, void *data )
: mPosition( position ), mData( data )
{
}
	
template<uint8_t DIM, class T>
HashTable<DIM,T>::HashTable( const vec_t &cellSize, uint32_t tableSize )
: mCellSize( cellSize ), mHashTableSize( tableSize ), mMin( 0 ), mMax( 10 ), mOffset( -mMin )
{
	for( size_t i = 0; i < mHashTableSize; i++ )
		mHashTable.push_back( std::vector<Node*>() );
}
template<uint8_t DIM, class T>
HashTable<DIM,T>::HashTable( const vec_t &min, const vec_t &max, const vec_t &cellSize, uint32_t tableSize )
: mCellSize( cellSize ), mHashTableSize( tableSize )
{
	for( size_t i = 0; i < mHashTableSize; i++ )
		mHashTable.push_back( std::vector<Node*>() );
}
template<uint8_t DIM, class T>
HashTable<DIM,T>::~HashTable()
{
	clear();
}
	
template<uint8_t DIM, class T>
void HashTable<DIM,T>::insert( const vec_t &position, void *data )
{
	uint32_t hash = HashTableTraits<DIM,T>::getHash( position, mCellSize, mHashTableSize );
        mHashTable[hash].emplace_back( new Node( position, data ) );
}
template<uint8_t DIM, class T>
void HashTable<DIM,T>::clear()
{
	for( auto& cell : mHashTable ) {
		for( auto& node : cell ) {
			if( node )
				delete node;
		}
		cell.clear();
	}
}
template<uint8_t DIM, class T>
size_t HashTable<DIM,T>::size() const
{
	size_t size = sizeof( *this );
	for( const auto& cell : mHashTable ) {
		size += sizeof( cell ) + sizeof(Node) * cell.size();
	}
	return size;
}
	
// TODO: Must be a better way to do this
template<uint8_t DIM, class T>
typename HashTable<DIM,T>::Node* HashTable<DIM,T>::nearestNeighborSearch( const vec_t &position, T *distanceSq ) const
{
	// Grow search radius until found something
	// TODO: !!! Might grow forever !!!
	std::vector<typename HashTable<DIM,T>::NodePair> results;
	T cellSize = static_cast<T>( mCellSize.x );
	while( ! results.size() ) {
		results = rangeSearch( position, cellSize );
		cellSize *= 2;
	}

	// Once we have nodes to look at, iterate and find the closest one
	Node* nearestNode = nullptr;
	T minDist = std::numeric_limits<T>::max();
	for( const auto& node : results ) {
		if( node.second < minDist ) {
			nearestNode = node.first;
			minDist = node.second;
		}
	}
	if( distanceSq != nullptr )
		*distanceSq = minDist;

	return nearestNode;
}

template<uint8_t DIM, class T>
std::vector<typename HashTable<DIM,T>::NodePair> HashTable<DIM,T>::rangeSearch( const vec_t &position, T radius ) const
{
	vec_t radiusVec = vec_t( radius );
	vec_t min       = glm::clamp( position - radiusVec, mMin, mMax + vec_t( 1 ) );
	vec_t max       = glm::clamp( position + radiusVec, mMin, mMax + vec_t( 1 ) );
	std::vector<typename HashTable<DIM,T>::NodePair> results;
	HashTableTraits<DIM,T>::rangeSearch( &results, mHashTable, position, radius, min, max + vec_t( mCellSize ), mCellSize, mHashTableSize );
	return results;
}

template<uint8_t DIM, class T>
void HashTable<DIM,T>::rangeSearch( const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const
{
	vec_t radiusVec = vec_t( radius );
	vec_t min       = glm::clamp( position - radiusVec, mMin, mMax + vec_t( 1 ) );
	vec_t max       = glm::clamp( position + radiusVec, mMin, mMax + vec_t( 1 ) );
	HashTableTraits<DIM,T>::rangeSearch( visitor, mHashTable, position, radius, min, max + vec_t( mCellSize ), mCellSize, mHashTableSize );
}
	
// explicit template instantiations
template class HashTable<2, float>;
template class HashTable<3, float>;
template class HashTable<2, double>;
template class HashTable<3, double>;
	
}