/*
 Grid - Space Partitioning algorithms for Cinder
 
 Copyright (c) 2016, Simon Geilfus, All rights reserved.
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

#pragma once

#include <vector>
#include <limits>
#include "cinder/AxisAlignedBox.h"
#include "cinder/Exception.h"
#include "cinder/Rect.h"
#include "cinder/Utilities.h"
#include "cinder/Vector.h"

namespace SpacePartitioning {
	
template<uint8_t DIM, class T, class DataT> struct GridTraits {};

//! Represents a Grid / Bin-lattice space partitioning structure
template<uint8_t DIM, class T, class DataT>
class Grid {
public:
	using vec_t = typename ci::VECDIM<DIM, T>::TYPE;
	using ivec_t = typename ci::VECDIM<DIM, int>::TYPE;
	
	//! Constructs an unbounded Grid, insertion will be much slower than the bounded version
	Grid( uint32_t k = 3 );
	//! Constructs a Grid with bounds, this should be the prefered constructor in terms of insertion performance
	Grid( const vec_t &min, const vec_t &max, uint32_t k = 3 );
	
	//! Inserts a new point in the Grid with optional user data
	void insert( const vec_t &position, const DataT &data = DataT() );
	//! Removes all the nodes from the Grid structure
	void clear();
	//! Returns the size of the Grid
	size_t size() const;
	
	//! Represents a single element of the Grid
	class Node {
	public:
		//! Returns the position of the node
		vec_t getPosition() const { return mPosition; }
		//! Returns the user data
		const DataT &getData() const { return mData; }
		
		Node( const vec_t &position, const DataT &data );
	protected:
		vec_t mPosition;
		DataT mData;
		friend class Grid;
	};
	
	using NodePair = std::pair<Node*,T>;
	using Vector = std::vector<std::vector<Node*>>;
	using bounds_t = typename GridTraits<DIM,T,DataT>::Bounds;
	
	//! Returns a pointer to the nearest Node with its square distance to the position
	Node* nearestNeighborSearch( const vec_t &position, T *distanceSq = nullptr ) const;
	//! Returns a vector of Nodes within a radius along with their square distances to the position
	std::vector<NodePair> rangeSearch( const vec_t &position, T radius ) const;
	//! Returns a vector of Nodes within a radius along with their square distances to the position
	void rangeSearch( const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const;
	
	//! Returns the number of bins of the grid
	size_t getNumBins() const { return mBins.size(); }
	//! Returns the number of bins of the grid in each dimension
	vec_t getNumCells() const { return mNumCells; }
	//! Returns the ith bin as a std::vector of Node*
	const std::vector<Node*> getBin( size_t i ) const { return mBins[i]; }
	//! Returns the bin at a position as a std::vector of Node*
	const std::vector<Node*> getBinAt( const vec_t &position ) const;
	//! Returns the bin index at a position
	size_t getBinIndexAt( const vec_t &position ) const;
	//! Returns the center of the ith bin
	vec_t getBinCenter( size_t i ) const;
	//! Returns the bounds of the ith bin
	bounds_t getBinBounds( size_t i ) const;
	//! Returns the size of a bin
	vec_t getBinsSize() const { return vec_t( mCellSize ); }
	
	//! Returns the minimum of the Grid
	vec_t getMin() const { return mMin; }
	//! Returns the maximum of the Grid
	vec_t getMax() const { return mMax; }
	//! Returns the bounds of the Grid
	bounds_t getBounds() const;
	
	~Grid();
protected:
	void resize( uint32_t k );
	void resize( const vec_t &min, const vec_t &max );
	void resize( const vec_t &min, const vec_t &max, uint32_t k );
	void insert( Node *node );
	
	Vector		mBins;
	ivec_t		mNumCells, mGridMin, mGridMax;
	vec_t		mMin, mMax, mOffset;
	uint32_t	mK, mCellSize;
};

	
class GridOutOfBoundsException : public ci::Exception {
public:
	GridOutOfBoundsException( const std::string &value ) : ci::Exception( value + " out of bounds" ) {}
};
	
// MARK: Grid Impl.
	
// http://www.red3d.com/cwr/papers/2000/pip.pdf
// https://en.wikipedia.org/wiki/Regular_grid
	
template<class T, class DataT>
struct GridTraits<2,T,DataT> {
	static typename Grid<2,T,DataT>::ivec_t toGridPosition( const typename Grid<2,T,DataT>::vec_t &position, const typename Grid<2,T,DataT>::vec_t &offset, uint32_t k ) {
		return typename Grid<2,T,DataT>::ivec_t( static_cast<uint32_t>( position.x + offset.x ) >> k,
										  static_cast<uint32_t>( position.y + offset.y ) >> k );
	}
	static typename Grid<2,T,DataT>::ivec_t toGridPosition( uint32_t index, const typename Grid<2,T,DataT>::ivec_t &numCells )
	{
		return typename Grid<2,T,DataT>::ivec_t( index % numCells.x, ( index / numCells.x ) % numCells.y );
	}
	static typename Grid<2,T,DataT>::vec_t toPosition( const typename Grid<2,T,DataT>::ivec_t &gridPosition, const typename Grid<2,T,DataT>::vec_t &offset, uint32_t k )
	{
		return typename Grid<2,T,DataT>::vec_t( static_cast<T>( gridPosition.x << k ) - offset.x,
											   static_cast<T>( gridPosition.y << k ) - offset.y );
	}
	static uint32_t toIndex( const typename Grid<2,T,DataT>::ivec_t &gridPos, const typename Grid<2,T,DataT>::ivec_t &numCells ) {
		return gridPos.x + numCells.x * gridPos.y;
	}
	static uint32_t toIndex( const typename Grid<2,T,DataT>::vec_t &position, const typename Grid<2,T,DataT>::vec_t &offset, const typename Grid<2,T,DataT>::ivec_t &numCells, uint32_t k ) {
		typename Grid<2,T,DataT>::ivec_t gridPos = toGridPosition( position, offset, k );
		return gridPos.x + numCells.x * gridPos.y;
	}
	static uint32_t gridSize( const typename Grid<2,T,DataT>::vec_t &numCells )
	{
		return numCells.x * numCells.y;
	}
	static void rangeSearch( std::vector<typename Grid<2,T,DataT>::NodePair> *results, const typename Grid<2,T,DataT>::Vector &grid, const typename Grid<2,T,DataT>::vec_t &position, T radius, const typename Grid<2,T,DataT>::ivec_t &minCell, const typename Grid<2,T,DataT>::ivec_t &maxCell, const typename Grid<2,T,DataT>::ivec_t &numCells )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename Grid<2,T,DataT>::ivec_t pos;
		for( pos.y = minCell.y; pos.y < maxCell.y; pos.y++ ) {
			for( pos.x = minCell.x; pos.x < maxCell.x; pos.x++ ) {
				const std::vector<typename Grid<2,T,DataT>::Node*>& cell = grid[GridTraits<2,T,DataT>::toIndex( pos, numCells )];
				for( const auto& node : cell ) {
					distSq = glm::distance2( position, node->getPosition() );
					if( distSq < radiusSq ) {
						results->emplace_back( std::make_pair( node, distSq ) );
					}
				}
			}
		}
	}
	static void rangeSearch( const std::function<void(typename Grid<2,T,DataT>::Node*,T)> &visitor, const typename Grid<2,T,DataT>::Vector &grid, const typename Grid<2,T,DataT>::vec_t &position, T radius, const typename Grid<2,T,DataT>::ivec_t &minCell, const typename Grid<2,T,DataT>::ivec_t &maxCell, const typename Grid<2,T,DataT>::ivec_t &numCells )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename Grid<2,T,DataT>::ivec_t pos;
		for( pos.y = minCell.y; pos.y < maxCell.y; pos.y++ ) {
			for( pos.x = minCell.x; pos.x < maxCell.x; pos.x++ ) {
				const std::vector<typename Grid<2,T,DataT>::Node*>& cell = grid[GridTraits<2,T,DataT>::toIndex( pos, numCells )];
				for( const auto& node : cell ) {
					distSq = glm::distance2( position, node->getPosition() );
					if( distSq < radiusSq ) {
						visitor( node, distSq );
					}
				}
			}
		}
	}
	typedef ci::RectT<T> Bounds;
};
template<class T, class DataT>
struct GridTraits<3,T,DataT> {
	static typename Grid<3,T,DataT>::ivec_t toGridPosition( const typename Grid<3,T,DataT>::vec_t &position, const typename Grid<3,T,DataT>::vec_t &offset, uint32_t k ) {
		return typename Grid<3,T,DataT>::ivec_t( static_cast<uint32_t>( position.x + offset.x ) >> k,
										  static_cast<uint32_t>( position.y + offset.y ) >> k,
										  static_cast<uint32_t>( position.z + offset.z ) >> k );
	}
	static typename Grid<3,T,DataT>::ivec_t toGridPosition( uint32_t index, const typename Grid<3,T,DataT>::ivec_t &numCells )
	{
		return typename Grid<3,T,DataT>::ivec_t( index % numCells.x, ( index / numCells.x ) % numCells.y, ( index / numCells.x ) / numCells.y );
	}
	static typename Grid<3,T,DataT>::vec_t toPosition( const typename Grid<3,T,DataT>::ivec_t &gridPosition, const typename Grid<3,T,DataT>::vec_t &offset, uint32_t k )
	{
		return typename Grid<3,T,DataT>::vec_t( static_cast<T>( gridPosition.x << k ) - offset.x,
											   static_cast<T>( gridPosition.y << k ) - offset.y,
											   static_cast<T>( gridPosition.z << k ) - offset.z );
	}
	static uint32_t toIndex( const typename Grid<3,T,DataT>::ivec_t &gridPos, const typename Grid<3,T,DataT>::ivec_t &numCells )
	{
		return gridPos.x + numCells.x * ( gridPos.y + numCells.y * gridPos.z );
	}
	static uint32_t toIndex( const typename Grid<3,T,DataT>::vec_t &position, const typename Grid<3,T,DataT>::vec_t &offset, const typename Grid<3,T,DataT>::ivec_t &numCells, uint32_t k )
	{
		typename Grid<3,T,DataT>::ivec_t gridPos = toGridPosition( position, offset, k );
		return gridPos.x + numCells.x * ( gridPos.y + numCells.y * gridPos.z );
	}
	static uint32_t gridSize( const typename Grid<3,T,DataT>::vec_t &numCells )
	{
		return numCells.x * numCells.y * numCells.z;
	}
	static void rangeSearch( std::vector<typename Grid<3,T,DataT>::NodePair> *results, const typename Grid<3,T,DataT>::Vector &bins, const typename Grid<3,T,DataT>::vec_t &position, T radius, const typename Grid<3,T,DataT>::ivec_t &minCell, const typename Grid<3,T,DataT>::ivec_t &maxCell, const typename Grid<3,T,DataT>::ivec_t &numCells )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename Grid<3,T,DataT>::ivec_t pos;
		for( pos.z = minCell.z; pos.z < maxCell.z; pos.z++ ) {
			for( pos.y = minCell.y; pos.y < maxCell.y; pos.y++ ) {
				for( pos.x = minCell.x; pos.x < maxCell.x; pos.x++ ) {
					const std::vector<typename Grid<3,T,DataT>::Node*>& cell = bins[GridTraits<3,T,DataT>::toIndex( pos, numCells )];
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
	static void rangeSearch( const std::function<void(typename Grid<3,T,DataT>::Node*,T)> &visitor, const typename Grid<3,T,DataT>::Vector &bins, const typename Grid<3,T,DataT>::vec_t &position, T radius, const typename Grid<3,T,DataT>::ivec_t &minCell, const typename Grid<3,T,DataT>::ivec_t &maxCell, const typename Grid<3,T,DataT>::ivec_t &numCells )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename Grid<3,T,DataT>::ivec_t pos;
		for( pos.z = minCell.z; pos.z < maxCell.z; pos.z++ ) {
			for( pos.y = minCell.y; pos.y < maxCell.y; pos.y++ ) {
				for( pos.x = minCell.x; pos.x < maxCell.x; pos.x++ ) {
					const std::vector<typename Grid<3,T,DataT>::Node*>& cell = bins[GridTraits<3,T,DataT>::toIndex( pos, numCells )];
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
	typedef ci::AxisAlignedBox Bounds;
};
	
template<uint8_t DIM, class T, class DataT>
Grid<DIM,T,DataT>::Node::Node( const vec_t &position, const DataT &data )
: mPosition( position ), mData( data )
{
}
	
template<uint8_t DIM, class T, class DataT>
Grid<DIM,T,DataT>::Grid( uint32_t k )
{
	resize( vec_t(std::numeric_limits<T>::max()), vec_t(std::numeric_limits<T>::min()), k );
}
template<uint8_t DIM, class T, class DataT>
Grid<DIM,T,DataT>::Grid( const vec_t &min, const vec_t &max, uint32_t k )
{
	resize( min, max, k );
}
template<uint8_t DIM, class T, class DataT>
Grid<DIM,T,DataT>::~Grid(){
	clear();
}
	
template<uint8_t DIM, class T, class DataT>
void Grid<DIM,T,DataT>::insert( const vec_t &position, const DataT &data )
{
	// Check if it fits the size of the grid's container
	if( glm::any( glm::greaterThan( position, mMax ) ) || glm::any( glm::lessThan( position, mMin ) ) )
		resize( glm::min( position, mMin ), glm::max( position, mMax ) );
	// Convert the position to 1D index
	uint32_t j = GridTraits<DIM,T,DataT>::toIndex( position, mOffset, mNumCells, mK );
	// And try to insert it in the grid
	if( j >= 0 && j < mBins.size() )
		mBins[j].push_back( new Node( position, data ) );
	else
		throw GridOutOfBoundsException( ci::toString( position ) );
}
template<uint8_t DIM, class T, class DataT>
void Grid<DIM,T,DataT>::insert( Node *node )
{
	// Check if it fits the size of the grid's container
	if( glm::any( glm::greaterThan( node->getPosition(), mMax ) ) || glm::any( glm::lessThan( node->getPosition(), mMin ) ) )
		resize( glm::min( node->getPosition(), mMin ), glm::max( node->getPosition(), mMax ) );
	// Convert the position to 1D index
	uint32_t j = GridTraits<DIM,T,DataT>::toIndex( node->getPosition(), mOffset, mNumCells, mK );
	// And try to insert it in the grid
	if( j >= 0 && j < mBins.size() )
		mBins[j].push_back( node );
	else
		throw GridOutOfBoundsException( ci::toString( node->getPosition() ) );
}
template<uint8_t DIM, class T, class DataT>
void Grid<DIM,T,DataT>::clear()
{
	for( auto& cell : mBins ) {
		for( auto& node : cell ) {
			if( node )
				delete node;
		}
		cell.clear();
	}
}
template<uint8_t DIM, class T, class DataT>
size_t Grid<DIM,T,DataT>::size() const
{
	size_t size = 0;
	for( const auto& cell : mBins ) {
		size += cell.size();
	}
	return size;
}
// TODO: Must be a better way to do this
template<uint8_t DIM, class T, class DataT>
typename Grid<DIM,T,DataT>::Node* Grid<DIM,T,DataT>::nearestNeighborSearch( const vec_t &position, T *distanceSq ) const
{
	// Grow search radius until found something
	// TODO: !!! Might grow forever !!!
	std::vector<typename Grid<DIM,T,DataT>::NodePair> results;
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

template<uint8_t DIM, class T, class DataT>
std::vector<typename Grid<DIM,T,DataT>::NodePair> Grid<DIM,T,DataT>::rangeSearch( const vec_t &position, T radius ) const
{
	vec_t radiusVec = vec_t( radius );
	vec_t min       = glm::clamp( position - radiusVec, mMin, mMax + vec_t( 1 ) );
	vec_t max       = glm::clamp( position + radiusVec, mMin, mMax + vec_t( 1 ) );
	ivec_t minCell	= glm::max( GridTraits<DIM,T,DataT>::toGridPosition( min, mOffset, mK ), ivec_t( 0 ) );
	ivec_t maxCell	= glm::min( ivec_t(1) + GridTraits<DIM,T,DataT>::toGridPosition( max, mOffset, mK ), mNumCells );
	std::vector<typename Grid<DIM,T,DataT>::NodePair> results;
	GridTraits<DIM,T,DataT>::rangeSearch( &results, mBins, position, radius, minCell, maxCell, mNumCells );
	return results;
}
template<uint8_t DIM, class T, class DataT>
void Grid<DIM,T,DataT>::rangeSearch( const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const
{
	vec_t radiusVec = vec_t( radius );
	vec_t min       = glm::clamp( position - radiusVec, mMin, mMax + vec_t( 1 ) );
	vec_t max       = glm::clamp( position + radiusVec, mMin, mMax + vec_t( 1 ) );
	ivec_t minCell	= glm::max( GridTraits<DIM,T,DataT>::toGridPosition( min, mOffset, mK ), ivec_t( 0 ) );
	ivec_t maxCell	= glm::min( ivec_t(1) + GridTraits<DIM,T,DataT>::toGridPosition( max, mOffset, mK ), mNumCells );
	GridTraits<DIM,T,DataT>::rangeSearch( visitor, mBins, position, radius, minCell, maxCell, mNumCells );
}
	
	
template<uint8_t DIM, class T, class DataT>
void Grid<DIM,T,DataT>::resize( const vec_t &min, const vec_t &max )
{
	mMin = min;
	mMax = max;
	mGridMin = GridTraits<DIM,T,DataT>::toGridPosition( min, mOffset, mK );
	mGridMax = GridTraits<DIM,T,DataT>::toGridPosition( max, mOffset, mK );
	resize( mK );
}
template<uint8_t DIM, class T, class DataT>
void Grid<DIM,T,DataT>::resize( const vec_t &min, const vec_t &max, uint32_t k )
{
	mMin = min;
	mMax = max;
	mGridMin = GridTraits<DIM,T,DataT>::toGridPosition( min, mOffset, mK );
	mGridMax = GridTraits<DIM,T,DataT>::toGridPosition( max, mOffset, mK );
	resize( k );
}
template<uint8_t DIM, class T, class DataT>
void Grid<DIM,T,DataT>::resize( uint32_t k )
{
	// If we have existing nodes we need to save them
	std::vector<Node*> nodes;
	for( auto& cell : mBins ) {
		nodes.insert( nodes.end(), cell.begin(), cell.end() );
		cell.clear();
	}
	
	// Update grid settings
	mK          = k;
	mCellSize   = 1 << k;
	mOffset     = -mMin;
	mNumCells   = glm::ceil( ( ( mMax - mMin ) + vec_t( 1 ) ) / static_cast<T>( mCellSize ) );
	mBins.resize( GridTraits<DIM,T,DataT>::gridSize( mNumCells ) );
	
	// Re-insert old nodes
	for( const auto& node : nodes ) {
		insert( node );
	}
}
	
template<uint8_t DIM, class T, class DataT>
const std::vector<typename Grid<DIM,T,DataT>::Node*> Grid<DIM,T,DataT>::getBinAt( const vec_t &position ) const
{
	// throw an exception if we're not in the grid bounds
	auto gridPos = GridTraits<DIM,T,DataT>::toGridPosition( position, mOffset, mK );
	if( glm::any( glm::greaterThan( gridPos, mGridMax ) ) || glm::any( glm::lessThan( gridPos, mGridMin ) ) )
		throw GridOutOfBoundsException( ci::toString( position ) );
	// get the converted position as a 1D index and return the corresponding bin
	auto i = GridTraits<DIM,T,DataT>::toIndex( gridPos, mNumCells );
	return mBins[i];
}
template<uint8_t DIM, class T, class DataT>
size_t Grid<DIM,T,DataT>::getBinIndexAt( const vec_t &position ) const
{
	// throw an exception if we're not in the grid bounds
	auto gridPos = GridTraits<DIM,T,DataT>::toGridPosition( position, mOffset, mK );
	if( glm::any( glm::greaterThan( gridPos, mGridMax ) ) || glm::any( glm::lessThan( gridPos, mGridMin ) ) )
		throw GridOutOfBoundsException( ci::toString( position ) );
	// return the converted position as a 1D index
	return GridTraits<DIM,T,DataT>::toIndex( gridPos, mNumCells );
}
template<uint8_t DIM, class T, class DataT>
typename Grid<DIM,T,DataT>::vec_t Grid<DIM,T,DataT>::getBinCenter( size_t i ) const
{
	ivec_t gridPosition = GridTraits<DIM,T,DataT>::toGridPosition( i, mNumCells );
	vec_t position = GridTraits<DIM,T,DataT>::toPosition( gridPosition, mOffset, mK );
	return position + vec_t( static_cast<T>( mCellSize ) ) / static_cast<T>( 2 );
}
template<uint8_t DIM, class T, class DataT>
typename Grid<DIM,T,DataT>::bounds_t Grid<DIM,T,DataT>::getBinBounds( size_t i ) const
{
	ivec_t gridPosition = GridTraits<DIM,T,DataT>::toGridPosition( i, mNumCells );
	vec_t position = GridTraits<DIM,T,DataT>::toPosition( gridPosition, mOffset, mK );
	return bounds_t( position, position + vec_t( static_cast<T>( mCellSize ) ) );
}
template<uint8_t DIM, class T, class DataT>
typename Grid<DIM,T,DataT>::bounds_t Grid<DIM,T,DataT>::getBounds() const
{
	vec_t min = GridTraits<DIM,T,DataT>::toPosition( mGridMin, mOffset, mK );
	vec_t max = GridTraits<DIM,T,DataT>::toPosition( mGridMax + ivec_t( 1 ), mOffset, mK );
	return bounds_t( min, max );
}

	
//! Represents a 2D float Grid / Bin-lattice space partitioning structure
template<class DataT=uint32_t> using Grid2 = Grid<2,float,DataT>;
//! Represents a 3D float Grid / Bin-lattice space partitioning structure
template<class DataT=uint32_t> using Grid3 = Grid<3,float,DataT>;
//! Represents a 2D double Grid / Bin-lattice space partitioning structure
template<class DataT=uint32_t> using dGrid2 = Grid<2,double,DataT>;
//! Represents a 3D double Grid / Bin-lattice space partitioning structure
template<class DataT=uint32_t> using dGrid3 = Grid<3,double,DataT>;
	
};

namespace sp = SpacePartitioning;