/*
 Bounding Volume Hierarchy - Space Partitioning algorithms for Cinder
 
 Copyright (c) 2016, Simon Geilfus, All rights reserved.
 Non-recursive FlatTree Code adapated from Brandon Pelfrey Fast-BVH (https://github.com/brandonpelfrey/Fast-BVH/)
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
#include <stack>
#include <limits>
#include <type_traits>

#include "cinder/AxisAlignedBox.h"
#include "cinder/Ray.h"

namespace SpacePartitioning {

//! Represents a Bounding Volume Hierarchy for fast Ray-Object intersection tests. T needs to implement "ci::AxisAlignedBox getBounds() const", "vec3 getCentroid() const" and "bool intersect( const Ray& r, float* dist ) const" or you need to specialize BVHObjectTraits with the relevant functions
template<class T>
class BVH {
public:	
	//! Constructs an empty Bounding Volume Hierarchy object
	BVH( uint32_t leafSize = 4 );
	//! Constructs a Bounding Volume Hierarchy object from a set of objects pointers
	BVH( std::vector<T> *objects, uint32_t leafSize = 4 );
		
	//! Returns the list of objects found within a radius around a position
	std::deque<T*>	rangeSearch( const ci::vec3 &position, float radius );
	//! Returns the list of objects found within a radius around a position
	void			rangeSearch( const ci::vec3 &position, float radius, const std::function<bool(T*)> &rangeVisitor );
	//! Returns the list of objects found within the range of a Sphere
	std::deque<T*>	rangeSearch( const ci::Sphere &range );
	//! Returns the list of objects found within the range of a Sphere
	void			rangeSearch( const ci::Sphere &range, const std::function<bool(T*)> &rangeVisitor );
	//! Returns the list of objects found within the bounds of an AxisAlignedBox
	std::deque<T*>	rangeSearch( const ci::AxisAlignedBox &range );
	//! Returns the list of objects found within the bounds of an AxisAlignedBox
	void			rangeSearch( const ci::AxisAlignedBox &range, const std::function<bool(T*)> &rangeVisitor );
	
	//! Represents the result of a raycasting test
	class RaycastResult {
	public:
		//! Returns the distance of the collision along the ray
		float		getDistance() const { return mDistance; }
		//! Returns the intersection position
		ci::vec3	getPosition() const { return mPosition; }
		//! Returns the object that intersects with the ray
		const T*	getObject() const { return mObject; }
	protected:
		float		mDistance;
		ci::vec3	mPosition;
		T*			mObject;
		friend class BVH;
	};

	//! Returns whether an intersection was found and the resulting distance, position and object hit by the ray
	bool raycast( const ci::Ray &ray, RaycastResult* result, bool earlyExit = false ) const;

protected:
	void build();
	
	struct Node {
	  ci::AxisAlignedBox	mBounds;
	  uint32_t				mStart;
	  uint32_t				mNumObjects;
	  int32_t				mRightOffset;
	};
	struct BuildNode {
		BuildNode( int32_t parent, uint32_t start, uint32_t end ) : mParentIndex( parent ), mStartIndex( start ), mEndIndex( end ) {}
		uint32_t	mStartIndex;
		uint32_t	mEndIndex;
		int32_t		mParentIndex;
	};
	struct TraversalNode {
		TraversalNode( int index, float minDist ) : mIndex( index ), mMinDist( minDist ) {}
		uint32_t	mIndex;
		float		mMinDist;
	};
	
	uint32_t			mNumNodes;
	uint32_t			mNumLeafs;
	uint32_t			mLeafSize;
	std::vector<Node>	mNodes;
	std::vector<T>		*mObjects;
};

// MARK: BVH Impl.

template<class T>
BVH<T>::BVH( uint32_t leafSize )
: mNumNodes( 0 ), mNumLeafs( 0 ), mLeafSize( leafSize ), mObjects( nullptr )
{
}

template<class T>
BVH<T>::BVH( std::vector<T>* objects, uint32_t leafSize )
: mNumNodes( 0 ), mNumLeafs( 0 ), mLeafSize( leafSize ), mObjects( objects )
{
	build();
}

namespace details {

	//! Detects if a class has a ci::AxisAlignedBox getBounds() function 
	template<typename C>
	struct BVHObjectHasGetBounds {
	private:
		template<typename T> static constexpr auto check(T*) -> typename std::is_same<decltype( std::declval<T>().getBounds() ),ci::AxisAlignedBox>::type;
		template<typename> static constexpr std::false_type check(...);
		typedef decltype(check<C>(0)) type;
	public:
		static constexpr bool value = type::value;
	};
	//! Detects if a class has a ci::vec3 getCentroid() function
	template<typename C>
	struct BVHObjectHasGetCentroid {
	private:
		template<typename T> static constexpr auto check(T*) -> typename std::is_same<decltype( std::declval<T>().getCentroid() ),ci::vec3>::type;
		template<typename> static constexpr std::false_type check(...);
		typedef decltype(check<C>(0)) type;
	public:
		static constexpr bool value = type::value;
	};
	//! Detects if a class has a bool intersect( const ci::Ray &, float* ) function
	template<typename C>
	struct BVHObjectHasIntersect {
	private:
		template<typename T> static constexpr auto check(T*) -> typename std::is_same<decltype( std::declval<T>().intersect( std::declval<const ci::Ray&>(), std::declval<float*>() ) ),bool>::type;
		template<typename> static constexpr std::false_type check(...);
		typedef decltype(check<C>(0)) type;
	public:
		static constexpr bool value = type::value;
	};
	
	//! Traits class used if one of the 3 methods are not found in the BVH template type
	template<class T> 
	class BVHObjectTraits {
	public:
		static ci::AxisAlignedBox getBounds( const T &obj ) { static_assert( std::false_type::value, "If the BVH template type doesn't have a \"getBounds\" member function, you need to specialize SpacePartitioning::details::BVHObjectTraits<> to provide an alternative." ); }
		static ci::vec3 getCentroid( const T &obj ) { static_assert( std::false_type::value, "If the BVH template type doesn't have a \"getCentroid\" member function, you need to specialize SpacePartitioning::details::BVHObjectTraits<> to provide an alternative." ); }
		static bool intersect( const T &obj, const ci::Ray &ray, float *dist ) { static_assert( std::false_type::value, "If the BVH template type doesn't have a \"intersect\" member function, you need to specialize SpacePartitioning::details::BVHObjectTraits<> to provide an alternative." ); }
	};
	
	template<typename T, typename std::enable_if<BVHObjectHasGetBounds<T>::value,int>::type = 0>
	inline ci::AxisAlignedBox BVHObjectGetBounds( const T &obj ) { return obj.getBounds(); }
	template<typename T, typename std::enable_if<!BVHObjectHasGetBounds<T>::value,int>::type = 0>
	inline ci::AxisAlignedBox BVHObjectGetBounds( const T &obj ) { return BVHObjectTraits<T>::getBounds( obj ); }
	
	template<typename T, typename std::enable_if<BVHObjectHasGetCentroid<T>::value,int>::type = 0>
	inline ci::vec3 BVHObjectGetCentroid( const T &obj ) { return obj.getCentroid(); }
	template<typename T, typename std::enable_if<!BVHObjectHasGetCentroid<T>::value,int>::type = 0>
	inline ci::vec3 BVHObjectGetCentroid( const T &obj ) { return BVHObjectTraits<T>::getCentroid( obj ); }
	
	template<typename T, typename std::enable_if<BVHObjectHasIntersect<T>::value,int>::type = 0>
	inline bool BVHObjectIntersect( const T &obj, const ci::Ray &ray, float *dist ) { return obj.intersect( ray, dist ); }
	template<typename T, typename std::enable_if<!BVHObjectHasIntersect<T>::value,int>::type = 0>
	inline bool BVHObjectIntersect( const T &obj, const ci::Ray &ray, float *dist ) { return BVHObjectTraits<T>::intersect( obj, ray, dist ); }
}

template<class T>
void BVH<T>::build()
{
	// create the working structures
	Node node;
	std::stack<BuildNode> stack;
	std::vector<Node> nodes;
	nodes.reserve( mObjects->size() * 2 );
	
	// push the root node
	BuildNode root( -1, 0, static_cast<uint32_t>( mObjects->size() ) );
	stack.push( root );

	while( ! stack.empty() ) {
		// get the next item from the stack
		BuildNode buildNode = stack.top();
		stack.pop();
		uint32_t startIndex = buildNode.mStartIndex;
		uint32_t endIndex = buildNode.mEndIndex;
		uint32_t numObjects = endIndex - startIndex;

		node.mStart = startIndex;
		node.mNumObjects = numObjects;
		node.mRightOffset = 2;
		mNumNodes++;

		// get the bounding box and centroid for the current node
		ci::AxisAlignedBox boundingBox( details::BVHObjectGetBounds( (*mObjects)[startIndex] ) );
		ci::AxisAlignedBox boundingCentroid( details::BVHObjectGetCentroid( (*mObjects)[startIndex] ), details::BVHObjectGetCentroid( (*mObjects)[startIndex] ) );
		for( uint32_t i = startIndex + 1; i < endIndex; ++i ) {
			boundingBox.include( details::BVHObjectGetBounds( (*mObjects)[i] ) );
			boundingCentroid.include( details::BVHObjectGetCentroid( (*mObjects)[i] ) );
		}
		node.mBounds = boundingBox;

		// current node is a leaf
		if( numObjects <= mLeafSize ) {
			node.mRightOffset = 0;
			mNumLeafs++;
		}

		nodes.push_back( node );

		// if this is not the root find out the node's right child
		if( buildNode.mParentIndex != -1 ) {
			nodes[buildNode.mParentIndex].mRightOffset--;
			if( nodes[buildNode.mParentIndex].mRightOffset == 0 ) {
				nodes[buildNode.mParentIndex].mRightOffset = mNumNodes - 1 - buildNode.mParentIndex;
			}
		}

		// subdivide only if the current node is a leaf
		if( node.mRightOffset != 0 ) {
			// find on which axis to make the split
			uint32_t splitAxis = 0;
			if( boundingCentroid.getExtents().y > boundingCentroid.getExtents().x ) splitAxis = 1;
			if( boundingCentroid.getExtents().z > boundingCentroid.getExtents().y ) splitAxis = 2;
			float splitAxisCenter = 0.5f * ( boundingCentroid.getMin()[splitAxis] + boundingCentroid.getMax()[splitAxis] );

			// find the center split index
			uint32_t splitIndex = startIndex;
			for( uint32_t i = startIndex; i < endIndex; ++i ) {
				if( details::BVHObjectGetCentroid( (*mObjects)[i] )[splitAxis] < splitAxisCenter ) {
					std::swap( (*mObjects)[i], (*mObjects)[splitIndex] );
					++splitIndex;
				}
			}
			if( splitIndex == startIndex || splitIndex == endIndex ) {
				splitIndex = startIndex + ( endIndex - startIndex ) / 2;
			}

			// push left & right children to the stack
			stack.push( BuildNode( mNumNodes - 1, startIndex, splitIndex ) );
			stack.push( BuildNode( mNumNodes - 1, splitIndex, endIndex ) );
		}
	}

	// Copy the temp node data to a flat array
	for( size_t i = 0; i < mNumNodes; ++i ) { 
		mNodes.push_back( nodes[i] );
	}
}

template<class T>
bool BVH<T>::raycast( const ci::Ray &ray, RaycastResult* result, bool earlyExit ) const
{
	// return false if BVH has not been initialized
	if( mNodes.empty() ) {
		return false;
	}

	// initialize the working structure and temp values
	std::stack<TraversalNode> stack;
	float intersectionDist;
	float aabIntersectDist[4];
	uint32_t closer, farther;
	result->mDistance = std::numeric_limits<float>::max();
	result->mObject = nullptr;

	// push the root node
	stack.push( TraversalNode( 0, std::numeric_limits<float>::lowest() ) );

	while( ! stack.empty() ) {
		// pop the current node
		const TraversalNode &traversalNode = stack.top();
		const Node &node( mNodes[traversalNode.mIndex] );
		stack.pop();
		
		// if this node is closer than the closest found intersection
		if( traversalNode.mMinDist <= result->mDistance ) {
			// if the node is a leaf check its objects for intersection
			if( node.mRightOffset == 0 ) {
				for( uint32_t i = 0; i < node.mNumObjects; ++i ) {
					const T& obj = (*mObjects)[node.mStart+i];
					if( details::BVHObjectIntersect( obj, ray, &intersectionDist ) ) {
						result->mObject = &(*mObjects)[node.mStart+i];

						// exit early if we're not interested in the closest intersection
						if( earlyExit ) {
							result->mDistance = intersectionDist;
							result->mPosition = ray.calcPosition( intersectionDist );
							return true;
						}
						// otherwise keep the intersection results
						if( intersectionDist < result->mDistance ) {
							result->mDistance = intersectionDist;
						}
					}
				}
			} 
			else {
				// check if intersecting with both bounding boxes
				bool hit0 = mNodes[traversalNode.mIndex+1].mBounds.intersect( ray, &aabIntersectDist[0], &aabIntersectDist[1] ) > 0;
				bool hit1 = mNodes[traversalNode.mIndex+node.mRightOffset].mBounds.intersect( ray, &aabIntersectDist[2], &aabIntersectDist[3] ) > 0;
				if( hit0 && hit1 ) {
					// find out which node is farther
					closer = traversalNode.mIndex+1;
					farther = traversalNode.mIndex+node.mRightOffset;
					if( aabIntersectDist[2] < aabIntersectDist[0] ) {
						std::swap( aabIntersectDist[0], aabIntersectDist[2] );
						std::swap( aabIntersectDist[1], aabIntersectDist[3] );
						std::swap( closer, farther );
					}
					// and push it first
					stack.push( TraversalNode( farther, aabIntersectDist[2] ) );
					stack.push( TraversalNode( closer, aabIntersectDist[0] ) );
				}
				else if( hit0 ) {
					stack.push( TraversalNode( traversalNode.mIndex + 1, aabIntersectDist[0] ) );
				}
				else if( hit1 ) {
					stack.push( TraversalNode( traversalNode.mIndex + node.mRightOffset, aabIntersectDist[2] ) );
				}
			}
		}
	}

	// calculate the intersection position
	if( result->mObject != nullptr ) {
		result->mPosition = ray.calcPosition( result->mDistance );
	}

	return result->mObject != nullptr;
}

template<class T>
std::deque<T*> BVH<T>::rangeSearch( const ci::vec3 &position, float radius )
{
	return rangeSearch( ci::Sphere( position, radius ) );
}

template<class T>
void BVH<T>::rangeSearch( const ci::vec3 &position, float radius, const std::function<bool( T* )> &rangeVisitor )
{
	return rangeSearch( ci::Sphere( position, radius, rangeVisitor ) );
}

template<class T>
std::deque<T*> BVH<T>::rangeSearch( const ci::Sphere &sphere )
{
	std::deque<T*> results;
	
	// return an empty vector if BVH has not been initialized
	if( mNodes.empty() ) {
		return results;
	}

	// initialize the working structure and temp values
	std::stack<TraversalNode> stack;
	uint32_t closer, farther;
	float sqRadius = sphere.getRadius() * sphere.getRadius();
	
	// push the root node
	stack.push( TraversalNode( 0, std::numeric_limits<float>::lowest() ) );
	
	while( ! stack.empty() ) {
		// pop the current node
		const TraversalNode &traversalNode = stack.top();
		const Node &node( mNodes[traversalNode.mIndex] );
		stack.pop();
		
		// if the node is a leaf check its objects are within the range
		if( node.mRightOffset == 0 ) {
			for( uint32_t i = 0; i < node.mNumObjects; ++i ) {
				const T& obj = (*mObjects)[node.mStart+i];
				if( glm::distance2( obj.getCentroid(), sphere.getCenter() ) < sqRadius ) {
					results.emplace_back( &(*mObjects)[node.mStart+i] );
				}
			}
		} 
		else {
			// check if intersecting with both bounding boxes
			bool hit0 = mNodes[traversalNode.mIndex+1].mBounds.intersects( sphere );
			bool hit1 = mNodes[traversalNode.mIndex+node.mRightOffset].mBounds.intersects( sphere );
			if( hit0 && hit1 ) {
				// find out which node is farther
				closer = traversalNode.mIndex+1;
				farther = traversalNode.mIndex+node.mRightOffset;
				if( glm::distance2( mNodes[traversalNode.mIndex+node.mRightOffset].mBounds.getCenter(), sphere.getCenter() ) < glm::distance2( mNodes[traversalNode.mIndex+1].mBounds.getCenter(), sphere.getCenter() ) ) {
					std::swap( closer, farther );
				}
				// and push it first
				stack.push( TraversalNode( farther, 0.0f ) );
				stack.push( TraversalNode( closer, 0.0f ) );
			}
			else if( hit0 ) {
				stack.push( TraversalNode( traversalNode.mIndex + 1, 0.0f ) );
			}
			else if( hit1 ) {
				stack.push( TraversalNode( traversalNode.mIndex + node.mRightOffset, 0.0f ) );
			}
		}
		
	}

	return results;
}

template<class T>
void BVH<T>::rangeSearch( const ci::Sphere &sphere, const std::function<bool( T* )> &rangeVisitor )
{
	// return an empty vector if BVH has not been initialized
	if( mNodes.empty() ) {
		return;
	}

	// initialize the working structure and temp values
	std::stack<TraversalNode> stack;
	uint32_t closer, farther;
	float sqRadius = sphere.getRadius() * sphere.getRadius();
	
	// push the root node
	stack.push( TraversalNode( 0, std::numeric_limits<float>::lowest() ) );
	
	while( ! stack.empty() ) {
		// pop the current node
		const TraversalNode &traversalNode = stack.top();
		const Node &node( mNodes[traversalNode.mIndex] );
		stack.pop();
		
		// if the node is a leaf check its objects are within the range
		if( node.mRightOffset == 0 ) {
			for( uint32_t i = 0; i < node.mNumObjects; ++i ) {
				const T& obj = (*mObjects)[node.mStart+i];
				if( glm::distance2( obj.getCentroid(), sphere.getCenter() ) < sqRadius ) {
					if( rangeVisitor( &(*mObjects)[node.mStart+i] ) ) {
						return;
					}
				}
			}
		} 
		else {
			// check if intersecting with both bounding boxes
			bool hit0 = mNodes[traversalNode.mIndex+1].mBounds.intersects( sphere );
			bool hit1 = mNodes[traversalNode.mIndex+node.mRightOffset].mBounds.intersects( sphere );
			if( hit0 && hit1 ) {
				// find out which node is farther
				closer = traversalNode.mIndex+1;
				farther = traversalNode.mIndex+node.mRightOffset;
				if( glm::distance2( mNodes[traversalNode.mIndex+node.mRightOffset].mBounds.getCenter(), sphere.getCenter() ) < glm::distance2( mNodes[traversalNode.mIndex+1].mBounds.getCenter(), sphere.getCenter() ) ) {
					std::swap( closer, farther );
				}
				// and push it first
				stack.push( TraversalNode( farther, 0.0f ) );
				stack.push( TraversalNode( closer, 0.0f ) );
			}
			else if( hit0 ) {
				stack.push( TraversalNode( traversalNode.mIndex + 1, 0.0f ) );
			}
			else if( hit1 ) {
				stack.push( TraversalNode( traversalNode.mIndex + node.mRightOffset, 0.0f ) );
			}
		}
		
	}
}

template<class T>
std::deque<T*> BVH<T>::rangeSearch( const ci::AxisAlignedBox &range )
{
	std::deque<T*> results;
	
	// return an empty vector if BVH has not been initialized
	if( mNodes.empty() ) {
		return results;
	}

	// initialize the working structure and temp values
	std::stack<TraversalNode> stack;
	uint32_t closer, farther;
	
	// push the root node
	stack.push( TraversalNode( 0, std::numeric_limits<float>::lowest() ) );
	
	while( ! stack.empty() ) {
		// pop the current node
		const TraversalNode &traversalNode = stack.top();
		const Node &node( mNodes[traversalNode.mIndex] );
		stack.pop();
		
		// if the node is a leaf check its objects are within the range
		if( node.mRightOffset == 0 ) {
			for( uint32_t i = 0; i < node.mNumObjects; ++i ) {
				const T& obj = (*mObjects)[node.mStart+i];
				if( obj.getBounds().intersects( range ) ) {
					results.emplace_back( &(*mObjects)[node.mStart+i] );
				}
			}
		} 
		else {
			// check if intersecting with both bounding boxes
			bool hit0 = mNodes[traversalNode.mIndex+1].mBounds.intersects( range );
			bool hit1 = mNodes[traversalNode.mIndex+node.mRightOffset].mBounds.intersects( range );
			if( hit0 && hit1 ) {
				// find out which node is farther
				closer = traversalNode.mIndex+1;
				farther = traversalNode.mIndex+node.mRightOffset;
				if( glm::distance2( mNodes[traversalNode.mIndex+node.mRightOffset].mBounds.getCenter(), range.getCenter() ) < glm::distance2( mNodes[traversalNode.mIndex+1].mBounds.getCenter(), range.getCenter() ) ) {
					std::swap( closer, farther );
				}
				// and push it first
				stack.push( TraversalNode( farther, 0.0f ) );
				stack.push( TraversalNode( closer, 0.0f ) );
			}
			else if( hit0 ) {
				stack.push( TraversalNode( traversalNode.mIndex + 1, 0.0f ) );
			}
			else if( hit1 ) {
				stack.push( TraversalNode( traversalNode.mIndex + node.mRightOffset, 0.0f ) );
			}
		}
		
	}

	return results;
}

template<class T>
void BVH<T>::rangeSearch( const ci::AxisAlignedBox &range, const std::function<bool( T* )> &rangeVisitor )
{
	// return an empty vector if BVH has not been initialized
	if( mNodes.empty() ) {
		return;
	}

	// initialize the working structure and temp values
	std::stack<TraversalNode> stack;
	uint32_t closer, farther;
	
	// push the root node
	stack.push( TraversalNode( 0, std::numeric_limits<float>::lowest() ) );
	
	while( ! stack.empty() ) {
		// pop the current node
		const TraversalNode &traversalNode = stack.top();
		const Node &node( mNodes[traversalNode.mIndex] );
		stack.pop();
		
		// if the node is a leaf check its objects are within the range
		if( node.mRightOffset == 0 ) {
			for( uint32_t i = 0; i < node.mNumObjects; ++i ) {
				const T& obj = (*mObjects)[node.mStart+i];
				if( obj.getBounds().intersects( range ) ) {
					if( rangeVisitor( &(*mObjects)[node.mStart+i] ) ) {
						return;
					}
				}
			}
		} 
		else {
			// check if intersecting with both bounding boxes
			bool hit0 = mNodes[traversalNode.mIndex+1].mBounds.intersects( range );
			bool hit1 = mNodes[traversalNode.mIndex+node.mRightOffset].mBounds.intersects( range );
			if( hit0 && hit1 ) {
				// find out which node is farther
				closer = traversalNode.mIndex+1;
				farther = traversalNode.mIndex+node.mRightOffset;
				if( glm::distance2( mNodes[traversalNode.mIndex+node.mRightOffset].mBounds.getCenter(), range.getCenter() ) < glm::distance2( mNodes[traversalNode.mIndex+1].mBounds.getCenter(), range.getCenter() ) ) {
					std::swap( closer, farther );
				}
				// and push it first
				stack.push( TraversalNode( farther, 0.0f ) );
				stack.push( TraversalNode( closer, 0.0f ) );
			}
			else if( hit0 ) {
				stack.push( TraversalNode( traversalNode.mIndex + 1, 0.0f ) );
			}
			else if( hit1 ) {
				stack.push( TraversalNode( traversalNode.mIndex + node.mRightOffset, 0.0f ) );
			}
		}
		
	}
}

};

namespace sp = SpacePartitioning;