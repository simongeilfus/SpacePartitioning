/*
 HashTable - Space Partitioning algorithms for Cinder
 
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
#include "cinder/Exception.h"
#include "cinder/Utilities.h"
#include "cinder/Vector.h"

namespace SpacePartitioning {
	
//! Represents a Spatial Hash Table partitioning structure
template<uint8_t DIM, class T, class DataT>
class HashTable {
public:
	using vec_t = typename ci::VECDIM<DIM, T>::TYPE;
	using ivec_t = typename ci::VECDIM<DIM, int>::TYPE;
	
	HashTable( const vec_t &cellSize, uint32_t tableSize );
	HashTable( const vec_t &min, const vec_t &max, const vec_t &cellSize, uint32_t tableSize );
	~HashTable();
	
	//! Inserts a new point in the HashTable with optional user data
	void insert( const vec_t &position, const DataT &data = DataT() );
	//! Removes all the nodes from the HashTable structure
	void clear();
	//! Returns the size of the HashTable
	size_t size() const;
	
	//! Represents a single element of the HashTable
	class Node {
	public:
		//! Returns the position of the node
		vec_t getPosition() const { return mPosition; }
		//! Returns the user data
		void* getData() const { return mData; }
		
		Node( const vec_t &position, const DataT &data );
	protected:
		vec_t mPosition;
		DataT mData;
		friend class HashTable;
	};
	
	using NodePair = std::pair<Node*,T>;
	using Vector = typename std::vector<std::vector<Node*>>;
	
	//! Returns a pointer to the nearest Node with its square distance to the position
	Node*			nearestNeighborSearch( const vec_t &position, T *distanceSq = nullptr ) const;
	//! Returns a vector of Nodes within a radius along with their square distances to the position
	std::vector<NodePair>	rangeSearch( const vec_t &position, T radius ) const;
	//! Returns a vector of Nodes within a radius along with their square distances to the position
	void			rangeSearch( const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const;
protected:
	
	Vector		mHashTable;
	vec_t		mCellSize;
	uint32_t	mHashTableSize;
	vec_t		mMin, mMax, mOffset;
};
	
class HashTableInsertionFailedException : public ci::Exception {
public:
	HashTableInsertionFailedException() : ci::Exception( "Failed to insert an element. You might want to try to change the table size." ) {}
};


	
// MARK: HashTable Impl.
	
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.4.5881&rep=rep1&type=pdf
// https://en.wikipedia.org/wiki/Locality-sensitive_hashing
	

template<uint8_t DIM, class T, class DataT> struct HashTableTraits {};
template<class T, class DataT>
struct HashTableTraits<2,T,DataT> {
	static uint32_t getHash( const typename HashTable<2,T,DataT>::vec_t &position, const typename HashTable<2,T,DataT>::vec_t &cellSize, uint32_t tableSize )
	{
		const typename HashTable<2,T,DataT>::ivec_t largePrime( 73856093, 19349663 );
		typename HashTable<2,T,DataT>::ivec_t p = glm::floor( position / cellSize );
		return ( ( p.x * largePrime.x ) ^ ( p.y * largePrime.y ) ) % tableSize;
	}
	static void rangeSearch( std::vector<typename HashTable<2,T,DataT>::NodePair> *results, const typename HashTable<2,T,DataT>::Vector &hashTable, const typename HashTable<2,T,DataT>::vec_t &position, T radius, const typename HashTable<2,T,DataT>::vec_t &minCell, const typename HashTable<2,T,DataT>::vec_t &maxCell, const typename HashTable<2,T,DataT>::vec_t &cellSize, uint32_t tableSize )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename HashTable<2,T,DataT>::vec_t pos;
		for( pos.y = minCell.y; pos.y < maxCell.y; pos.y += cellSize.y ) {
			for( pos.x = minCell.x; pos.x < maxCell.x; pos.x += cellSize.x ) {
				const std::vector<typename HashTable<2,T,DataT>::Node*>& cell = hashTable[HashTableTraits<2,T,DataT>::getHash( pos, cellSize, tableSize )];
				for( const auto& node : cell ) {
					distSq = glm::distance2( position, node->getPosition() );
					if( distSq < radiusSq ) {
						results->emplace_back( std::make_pair( node, distSq ) );
					}
				}
			}
		}
		
	}
	static void rangeSearch( const std::function<void(typename HashTable<2,T,DataT>::Node*,T)> &visitor, const typename HashTable<2,T,DataT>::Vector &hashTable, const typename HashTable<2,T,DataT>::vec_t &position, T radius, const typename HashTable<2,T,DataT>::vec_t &minCell, const typename HashTable<2,T,DataT>::vec_t &maxCell, const typename HashTable<2,T,DataT>::vec_t &cellSize, uint32_t tableSize )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename HashTable<2,T,DataT>::vec_t pos;
		for( pos.y = minCell.y; pos.y < maxCell.y; pos.y += cellSize.y ) {
			for( pos.x = minCell.x; pos.x < maxCell.x; pos.x += cellSize.x ) {
				const std::vector<typename HashTable<2,T,DataT>::Node*>& cell = hashTable[HashTableTraits<2,T,DataT>::getHash( pos, cellSize, tableSize )];
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

template<class T, class DataT>
struct HashTableTraits<3,T,DataT> {
	static uint32_t getHash( const typename HashTable<3,T,DataT>::vec_t &position, const typename HashTable<3,T,DataT>::vec_t &cellSize, uint32_t tableSize )
	{
		const typename HashTable<3,T,DataT>::ivec_t largePrime( 73856093, 19349663, 83492791 );
		typename HashTable<3,T,DataT>::ivec_t p = glm::floor( position / cellSize );
		return ( ( p.x * largePrime.x ) ^ ( p.y * largePrime.y ) ^ ( p.z * largePrime.z ) ) % tableSize;
	}
	static void rangeSearch( std::vector<typename HashTable<3,T,DataT>::NodePair> *results, const typename HashTable<3,T,DataT>::Vector &hashTable, const typename HashTable<3,T,DataT>::vec_t &position, T radius, const typename HashTable<3,T,DataT>::vec_t &minCell, const typename HashTable<3,T,DataT>::vec_t &maxCell, const typename HashTable<3,T,DataT>::vec_t &cellSize, uint32_t tableSize )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename HashTable<3,T,DataT>::vec_t pos;
		for( pos.z = minCell.z; pos.z < maxCell.z; pos.z += cellSize.z ) {
			for( pos.y = minCell.y; pos.y < maxCell.y; pos.y += cellSize.y ) {
				for( pos.x = minCell.x; pos.x < maxCell.x; pos.x += cellSize.x ) {
					const std::vector<typename HashTable<3,T,DataT>::Node*>& cell = hashTable[HashTableTraits<3,T,DataT>::getHash( pos, cellSize, tableSize )];
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
	static void rangeSearch( const std::function<void(typename HashTable<3,T,DataT>::Node*,T)> &visitor, const typename HashTable<3,T,DataT>::Vector &hashTable, const typename HashTable<3,T,DataT>::vec_t &position, T radius, const typename HashTable<3,T,DataT>::vec_t &minCell, const typename HashTable<3,T,DataT>::vec_t &maxCell, const typename HashTable<3,T,DataT>::vec_t &cellSize, uint32_t tableSize )
	{
		T distSq;
		T radiusSq = radius * radius;
		typename HashTable<3,T,DataT>::vec_t pos;
		for( pos.z = minCell.z; pos.z < maxCell.z; pos.z += cellSize.z ) {
			for( pos.y = minCell.y; pos.y < maxCell.y; pos.y += cellSize.y ) {
				for( pos.x = minCell.x; pos.x < maxCell.x; pos.x += cellSize.x ) {
					const std::vector<typename HashTable<3,T,DataT>::Node*>& cell = hashTable[HashTableTraits<3,T,DataT>::getHash( pos, cellSize, tableSize )];
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
	
template<uint8_t DIM, class T, class DataT>
HashTable<DIM,T,DataT>::Node::Node( const vec_t &position, const DataT &data )
: mPosition( position ), mData( data )
{
}
	
template<uint8_t DIM, class T, class DataT>
HashTable<DIM,T,DataT>::HashTable( const vec_t &cellSize, uint32_t tableSize )
: mCellSize( cellSize ), mHashTableSize( tableSize ), mMin( 0 ), mMax( 10 ), mOffset( -mMin )
{
	for( size_t i = 0; i < mHashTableSize; i++ )
		mHashTable.push_back( std::vector<Node*>() );
}
template<uint8_t DIM, class T, class DataT>
HashTable<DIM,T,DataT>::HashTable( const vec_t &min, const vec_t &max, const vec_t &cellSize, uint32_t tableSize )
: mCellSize( cellSize ), mHashTableSize( tableSize )
{
	for( size_t i = 0; i < mHashTableSize; i++ )
		mHashTable.push_back( std::vector<Node*>() );
}
template<uint8_t DIM, class T, class DataT>
HashTable<DIM,T,DataT>::~HashTable()
{
	clear();
}
	
template<uint8_t DIM, class T, class DataT>
void HashTable<DIM,T,DataT>::insert( const vec_t &position, const DataT &data )
{
	uint32_t hash = HashTableTraits<DIM,T,DataT>::getHash( position, mCellSize, mHashTableSize );
        mHashTable[hash].emplace_back( new Node( position, data ) );
}
template<uint8_t DIM, class T, class DataT>
void HashTable<DIM,T,DataT>::clear()
{
	for( auto& cell : mHashTable ) {
		for( auto& node : cell ) {
			if( node )
				delete node;
		}
		cell.clear();
	}
}
template<uint8_t DIM, class T, class DataT>
size_t HashTable<DIM,T,DataT>::size() const
{
	size_t size = 0;
	for( const auto& cell : mHashTable ) {
		size += cell.size();
	}
	return size;
}
	
// TODO: Must be a better way to do this
template<uint8_t DIM, class T, class DataT>
typename HashTable<DIM,T,DataT>::Node* HashTable<DIM,T,DataT>::nearestNeighborSearch( const vec_t &position, T *distanceSq ) const
{
	// Grow search radius until found something
	// TODO: !!! Might grow forever !!!
	std::vector<typename HashTable<DIM,T,DataT>::NodePair> results;
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

template<uint8_t DIM, class T, class DataT>
std::vector<typename HashTable<DIM,T,DataT>::NodePair> HashTable<DIM,T,DataT>::rangeSearch( const vec_t &position, T radius ) const
{
	vec_t radiusVec = vec_t( radius );
	vec_t min       = glm::clamp( position - radiusVec, mMin, mMax + vec_t( 1 ) );
	vec_t max       = glm::clamp( position + radiusVec, mMin, mMax + vec_t( 1 ) );
	std::vector<typename HashTable<DIM,T,DataT>::NodePair> results;
	HashTableTraits<DIM,T,DataT>::rangeSearch( &results, mHashTable, position, radius, min, max + vec_t( mCellSize ), mCellSize, mHashTableSize );
	return results;
}

template<uint8_t DIM, class T, class DataT>
void HashTable<DIM,T,DataT>::rangeSearch( const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const
{
	vec_t radiusVec = vec_t( radius );
	vec_t min       = glm::clamp( position - radiusVec, mMin, mMax + vec_t( 1 ) );
	vec_t max       = glm::clamp( position + radiusVec, mMin, mMax + vec_t( 1 ) );
	HashTableTraits<DIM,T,DataT>::rangeSearch( visitor, mHashTable, position, radius, min, max + vec_t( mCellSize ), mCellSize, mHashTableSize );
}
	
//! Represents a 2D float Spatial Hash Table partitioning structure
template<class DataT> using HashTable2_t	= HashTable<2,float,DataT>;
//! Represents a 3D float Spatial Hash Table partitioning structure
template<class DataT> using HashTable3_t	= HashTable<3,float,DataT>;
//! Represents a 2D double Spatial Hash Table partitioning structure
template<class DataT> using dHashTable2_t	= HashTable<2,double,DataT>;
//! Represents a 3D double Spatial Hash Table partitioning structure
template<class DataT> using dHashTable3_t	= HashTable<3,double,DataT>;
	
//! Represents a 2D float Spatial Hash Table partitioning structure with uint32_t data
using HashTable2	= HashTable<2,float,uint32_t>;
//! Represents a 3D float Spatial Hash Table partitioning structure with uint32_t data
using HashTable3	= HashTable<3,float,uint32_t>;
//! Represents a 2D double Spatial Hash Table partitioning structure with uint32_t data
using dHashTable2	= HashTable<2,double,uint32_t>;
//! Represents a 3D double Spatial Hash Table partitioning structure with uint32_t data
using dHashTable3	= HashTable<3,double,uint32_t>;
	
};