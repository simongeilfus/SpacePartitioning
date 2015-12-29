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

#pragma once

#include <vector>
#include "cinder/Vector.h"

namespace SpacePartitioning {
	
//! Represents the base space partitioning structure
template<uint8_t DIM, class T>
class Structure {
public:
	virtual ~Structure() {}
	using vec_t = typename ci::VECDIM<DIM, T>::TYPE;
	using ivec_t = typename ci::VECDIM<DIM, int>::TYPE;
	
	//! Inserts a new point in the structure with an optional pointer to user data
	virtual void insert( const vec_t &position, void *data = nullptr ) = 0;
	//! Removes all the nodes from the structure
	virtual void clear() = 0;
};
	
	
//! Represents a K-D Tree space partitioning structure
template<uint8_t DIM, class T>
class KdTree : public Structure<DIM,T> {
public:
	using vec_t = typename Structure<DIM,T>::vec_t;
	
	//! Inserts a new point in the tree with an optional pointer to user data
	void insert( const vec_t &position, void *data = nullptr ) override;
	//! Removes all the nodes from the structure
	void clear() override;
	//! Returns the size of the KdTree in terms of bytes
	size_t size() const;
	
	//! Represents a single element of the KdTree
	class Node {
	public:
		//! Returns the position of the node
		vec_t getPosition() const { return mPosition; }
		//! Returns the user data pointer
		void* getData() const { return mData; }
		
		Node( const vec_t &position, int axis, void *data );
		~Node();
	protected:
		vec_t	mPosition;
		int	mAxis;
		Node*	mLeft;
		Node*	mRight;
		void*	mData;
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
	
	void insertImpl( Node **node, const vec_t &position, void *data, int axis );
	void nearestNeighborSearchImpl( Node *node, HyperRect *rect, const vec_t &position, Node **result, T *resultDistanceSq ) const;
	void rangeSearchImpl( Node *node, const vec_t &position, T radius, std::vector<NodePair> *results ) const;
	void rangeSearchImpl( Node *node, const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const;
	size_t sizeImpl( Node *node ) const;
	
	Node*		mRoot;
	HyperRect	mHyperRect;
};
	
//! Represents a Grid / Bin-lattice space partitioning structure
template<uint8_t DIM, class T>
class Grid : public Structure<DIM,T> {
public:
	using vec_t = typename Structure<DIM,T>::vec_t;
	using ivec_t = typename Structure<DIM,int>::vec_t;
	
	//! Constructs an unbounded Grid, insertion will be much slower than the bounded version
	Grid( uint32_t k = 3 );
	//! Constructs a Grid with bounds, this should be the prefered constructor in terms of insertion performance
	Grid( const vec_t &min, const vec_t &max, uint32_t k = 3 );
	
	//! Inserts a new point in the Grid with an optional pointer to user data
	void insert( const vec_t &position, void *data = nullptr ) override;
	//! Removes all the nodes from the Grid structure
	void clear() override;
	//! Returns the size of the Grid in terms of bytes
	size_t size() const;
	
	//! Represents a single element of the Grid
	class Node {
	public:
		//! Returns the position of the node
		vec_t getPosition() const { return mPosition; }
		//! Returns the user data pointer
		void* getData() const { return mData; }
		
		Node( const vec_t &position, void *data );
	protected:
		vec_t mPosition;
		void* mData;
		friend class Grid;
	};
	
	using NodePair = std::pair<Node*,T>;
	using Vector = typename std::vector<std::vector<Node*>>;
	
	//! Returns a pointer to the nearest Node with its square distance to the position
	Node*			nearestNeighborSearch( const vec_t &position, T *distanceSq = nullptr ) const;
	//! Returns a vector of Nodes within a radius along with their square distances to the position
	std::vector<NodePair>	rangeSearch( const vec_t &position, T radius ) const;
	//! Returns a vector of Nodes within a radius along with their square distances to the position
	void			rangeSearch( const vec_t &position, T radius, const std::function<void(Node*,T)> &visitor ) const;
	
	~Grid();
protected:
	void resize( uint32_t k );
	void resize( const vec_t &min, const vec_t &max );
	void resize( const vec_t &min, const vec_t &max, uint32_t k );
	void insert( Node *node );
	
	Vector		mGrid;
	ivec_t		mNumCells;
	vec_t		mMin, mMax, mOffset;
	uint32_t	mK, mCellSize;
};
	
class GridOutOfBoundsException : public ci::Exception {
public:
	GridOutOfBoundsException() : ci::Exception( "Out Of Bounds" ) {}
};
	
//! Represents a Spatial Hash Table partitioning structure
template<uint8_t DIM, class T>
class HashTable : public Structure<DIM,T> {
public:
	using vec_t = typename Structure<DIM,T>::vec_t;
	using ivec_t = typename Structure<DIM,int>::vec_t;
	
	HashTable();
	~HashTable();
	
	//! Inserts a new point in the HashTable with an optional pointer to user data
	void insert( const vec_t &position, void *data = nullptr ) override;
	//! Removes all the nodes from the HashTable structure
	void clear() override;
	//! Returns the size of the HashTable in terms of bytes
	size_t size() const;
	
	//! Represents a single element of the HashTable
	class Node {
	public:
		//! Returns the position of the node
		vec_t getPosition() const { return mPosition; }
		//! Returns the user data pointer
		void* getData() const { return mData; }
		
		Node( const vec_t &position, void *data );
	protected:
		vec_t mPosition;
		void* mData;
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
};

// TODO: Implement BSPTrees
//! Represents a Binary Space Partitioning Tree
class BSPTree {
public:
};
	
// TODO: Implement Octrees
//! Represents a Octree
class OctTree {
public:
};
	

typedef Structure<2, float>	Structure2;
typedef Structure<3, float>	Structure3;
typedef Structure<4, float>	Structure4;
typedef Structure<2, double>	dStructure2;
typedef Structure<3, double>	dStructure3;
typedef Structure<4, double>	dStructure4;
	
typedef KdTree<2, float>	KdTree2;
typedef KdTree<3, float>	KdTree3;
typedef KdTree<4, float>	KdTree4;
typedef KdTree<2, double>	dKdTree2;
typedef KdTree<3, double>	dKdTree3;
typedef KdTree<4, double>	dKdTree4;
	
typedef Grid<2, float>		Grid2;
typedef Grid<3, float>		Grid3;
typedef Grid<2, double>		dGrid2;
typedef Grid<3, double>		dGrid3;
};

namespace sp = SpacePartitioning;