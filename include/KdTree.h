/*
 KdTree - Space Partitioning algorithms for Cinder
 
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
	
//! Represents a K-D Tree space partitioning structure
template<uint8_t DIM, class T, class DataT>
class KdTree {
public:
	using vec_t = typename ci::VECDIM<DIM, T>::TYPE;
	
	//! Inserts a new point in the tree with optional user data
	void insert( const vec_t &position, const DataT &data = DataT() );
	//! Removes all the nodes from the structure
	void clear();
	//! Returns the size of the KdTree
	size_t size() const;
	
	//! Represents a single element of the KdTree
	class Node {
	public:
		//! Returns the position of the node
		vec_t getPosition() const { return mPosition; }
		//! Returns the user data
		const DataT& getData() const { return mData; }
		
		Node( const vec_t &position, int axis, const DataT &data );
		~Node();
	protected:
		vec_t	mPosition;
		int	mAxis;
		Node*	mLeft;
		Node*	mRight;
		DataT	mData;
		friend class KdTree;
	};
	
	using NodePair = std::pair<Node*,T>;
	
	//! Returns a pointer to the nearest Node with its square distance to the position
	Node*			nearestNeighborSearch( const vec_t &position, T *distanceSq = nullptr ) const;
	//! Returns a vector of Nodes within a radius along with their square distances to the position
	std::vector<NodePair>	rangeSearch( const vec_t &position, T radius ) const;
	//! Returns a vector of Nodes within a radius along with their square distances to the position
	void			rangeSearch( const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const;
	
	KdTree();
	~KdTree();
protected:
	struct HyperRect {
		HyperRect();
		HyperRect( const vec_t &min, const vec_t &max );
		
		//! Includes the position in the HyperRect
		void extend( const vec_t &position );
		//! Returns the distance squared between the position and the HyperRect
		T distance2( const vec_t &position ) const;
		
		vec_t mMin, mMax;
	};
	
	void insertImpl( Node **node, const vec_t &position, const DataT &data, int axis );
	void nearestNeighborSearchImpl( Node *node, HyperRect *rect, const vec_t &position, Node **result, T *resultDistanceSq ) const;
	void rangeSearchImpl( Node *node, const vec_t &position, T radius, std::vector<NodePair> *results ) const;
	void rangeSearchImpl( Node *node, const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const;
	size_t sizeImpl( Node *node ) const;
	
	Node*		mRoot;
	HyperRect	mHyperRect;
};
	
	
	
// MARK: KDTree Impl.
	
// https://en.wikipedia.org/wiki/K-d_tree
// https://github.com/jtsiomb/kdtree
// https://github.com/mikolalysenko/static-kdtree
// http://www.flipcode.com/archives/Raytracing_Topics_Techniques-Part_7_Kd-Trees_and_More_Speed.shtml
	
// KdTree::HyperRect
template<uint8_t DIM, class T, class DataT>
KdTree<DIM,T,DataT>::HyperRect::HyperRect()
: mMin( std::numeric_limits<T>::max() ), mMax( std::numeric_limits<T>::min() )
{
}
template<uint8_t DIM, class T, class DataT>
KdTree<DIM,T,DataT>::HyperRect::HyperRect( const vec_t &min, const vec_t &max )
: mMin( min ), mMax( max )
{
}
template<uint8_t DIM, class T, class DataT>
void KdTree<DIM,T,DataT>::HyperRect::extend( const vec_t &position )
{
	mMin = glm::min( mMin, position );
	mMax = glm::max( mMax, position );
}
template<uint8_t DIM, class T, class DataT>
T KdTree<DIM,T,DataT>::HyperRect::distance2( const vec_t &position ) const
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
template<uint8_t DIM, class T, class DataT>
KdTree<DIM,T,DataT>::Node::Node( const vec_t &position, int axis, const DataT &data )
: mPosition( position ), mAxis( axis ), mData( data ), mLeft( nullptr ), mRight( nullptr )
{
}
template<uint8_t DIM, class T, class DataT>
KdTree<DIM,T,DataT>::Node::~Node()
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
template<uint8_t DIM, class T, class DataT>
KdTree<DIM,T,DataT>::KdTree()
: mRoot( nullptr )
{
}
template<uint8_t DIM, class T, class DataT>
KdTree<DIM,T,DataT>::~KdTree()
{
	if( mRoot ) {
		delete mRoot;
		mRoot = nullptr;
	}
}

template<uint8_t DIM, class T, class DataT>
void KdTree<DIM,T,DataT>::insertImpl( Node **nptr, const vec_t &position, const DataT &data, int axis )
{
	using Node_t = typename KdTree<DIM,T,DataT>::Node;
	
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

template<uint8_t DIM, class T, class DataT>
void KdTree<DIM,T,DataT>::insert( const vec_t &position, const DataT &data )
{
	insertImpl( &mRoot, position, data, 0 );
	mHyperRect.extend( position );
}
template<uint8_t DIM, class T, class DataT>
void KdTree<DIM,T,DataT>::clear()
{
	if( mRoot ) {
		delete mRoot;
		mRoot = nullptr;
	}
}

template<uint8_t DIM, class T, class DataT>
size_t KdTree<DIM,T,DataT>::sizeImpl( Node *node ) const
{
	size_t size = 0;
	if( node ) {
		size++;
		if( node->mLeft )
			size += sizeImpl( node->mLeft );
		if( node->mRight )
			size += sizeImpl( node->mRight );
	}
	return size;
}

template<uint8_t DIM, class T, class DataT>
size_t KdTree<DIM,T,DataT>::size() const
{
	return sizeImpl( mRoot );
}
	
template<uint8_t DIM, class T, class DataT>
typename KdTree<DIM,T,DataT>::Node* KdTree<DIM,T,DataT>::nearestNeighborSearch( const vec_t &position, T *distanceSq ) const
{
	Node* result	= mRoot;
	T dSq			= glm::distance2( position, result->mPosition );
	HyperRect rect	= mHyperRect;
	nearestNeighborSearchImpl( mRoot, &rect, position, &result, &dSq );
	if( distanceSq )
		*distanceSq = dSq;
	
	return result;
}

template<uint8_t DIM, class T, class DataT>
void KdTree<DIM,T,DataT>::nearestNeighborSearchImpl( Node *node, HyperRect *rect, const vec_t &position, Node **result, T *resultDistanceSq ) const
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

template<uint8_t DIM, class T, class DataT>
std::vector<typename KdTree<DIM,T,DataT>::NodePair> KdTree<DIM,T,DataT>::rangeSearch( const vec_t &position, T radius ) const
{
	std::vector<NodePair> results;
	rangeSearchImpl( mRoot, position, radius, &results );
	return results;
}
template<uint8_t DIM, class T, class DataT>
void KdTree<DIM,T,DataT>::rangeSearchImpl( Node *node, const vec_t &position, T radius, std::vector<typename KdTree<DIM,T,DataT>::NodePair> *results ) const
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

template<uint8_t DIM, class T, class DataT>
void KdTree<DIM,T,DataT>::rangeSearch( const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const
{
	rangeSearchImpl( mRoot, position, radius, visitor );
}
template<uint8_t DIM, class T, class DataT>
void KdTree<DIM,T,DataT>::rangeSearchImpl( Node *node, const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const
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
	
//! Represents a 2D float K-D Tree space partitioning structure
template<class DataT> using KdTree2_t	= KdTree<2,float,DataT>;
//! Represents a 3D float K-D Tree space partitioning structure
template<class DataT> using KdTree3_t	= KdTree<3,float,DataT>;
//! Represents a 2D double K-D Tree space partitioning structure
template<class DataT> using dKdTree2_t	= KdTree<2,double,DataT>;
//! Represents a 3D double K-D Tree space partitioning structure
template<class DataT> using dKdTree3_t	= KdTree<3,double,DataT>;
	
//! Represents a 2D float K-D Tree space partitioning structure with uint32_t data
using KdTree2	= KdTree<2,float,uint32_t>;
//! Represents a 3D float K-D Tree space partitioning structure with uint32_t data
using KdTree3	= KdTree<3,float,uint32_t>;
//! Represents a 2D double K-D Tree space partitioning structure with uint32_t data
using dKdTree2	= KdTree<2,double,uint32_t>;
//! Represents a 3D double K-D Tree space partitioning structure with uint32_t data
using dKdTree3	= KdTree<3,double,uint32_t>;
	
};

namespace sp = SpacePartitioning;