// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "dynamic_tree.h"

#include "aabb.h"
#include "algorithm.h"
#include "core.h"
#include "platform.h"
#include "simd.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

_Static_assert( B3_ROOT_NODE == 0, "bad root" );
_Static_assert( sizeof( b3TreeNode ) == 32, "expected size" );
_Static_assert( sizeof( b3TreeProxy ) == 24, "expected size" );

// The free sibling pair list is chained through the parent index of their first node. A pair comes from the
// free list when there is one and from the bump pointer otherwise. A rebuild empties the free list, so
// allocations can just bump.
static int b3AllocateSiblingPair( b3DynamicTree* tree )
{
	// Allocate from free list.
	if ( tree->pairFreeList != B3_NULL_INDEX )
	{
		int pair = tree->pairFreeList;
		tree->pairFreeList = tree->parents[pair];
		return pair;
	}

	// Extend the bump allocator if needed.
	if ( tree->nodeEnd + 2 > tree->nodeCapacity )
	{
		int oldCapacity = tree->nodeCapacity;
		int newCapacity = oldCapacity + ( oldCapacity >> 1 );
		newCapacity += newCapacity & 1;
		tree->nodes = B3_GROW_ZERO( tree->nodes, oldCapacity, newCapacity );
		tree->parents = B3_GROW_ZERO( tree->parents, oldCapacity, newCapacity );
		tree->nodeCapacity = newCapacity;

		// The spare has to match, the rebuild allocates it again.
		b3Free( tree->swapNodes, oldCapacity * sizeof( b3TreeNode ) );
		tree->swapNodes = NULL;
	}

	int pair = tree->nodeEnd;
	tree->nodeEnd += 2;
	return pair;
}

static void b3FreePair( b3DynamicTree* tree, int pair )
{
	B3_ASSERT( ( pair & 1 ) == 0 && 2 <= pair && pair < tree->nodeEnd );
	tree->nodes[pair] = b3MakeEmptyNode();
	tree->nodes[pair + 1] = b3MakeEmptyNode();
	tree->parents[pair] = tree->pairFreeList;
	tree->parents[pair + 1] = B3_NULL_INDEX;
	tree->pairFreeList = pair;
}

b3DynamicTree b3DynamicTree_Create( int proxyCapacity )
{
	int capacity = b3MaxInt( proxyCapacity, 16 );

	// Intentionally _not_ initialized with brace initialization, which can leave
	// uninitialized gaps.
	b3DynamicTree tree;

	// memset needed for deterministic serialization
	memset( &tree, 0, sizeof( b3DynamicTree ) );

	tree.version = B3_DYNAMIC_TREE_VERSION;

	// A tree of n proxies has 2n - 1 nodes plus the empty node beside the root
	tree.nodeCapacity = 2 * capacity;
	tree.nodes = (b3TreeNode*)b3AllocZero( tree.nodeCapacity * sizeof( b3TreeNode ) );
	tree.parents = (int32_t*)b3AllocZero( tree.nodeCapacity * sizeof( int32_t ) );
	tree.pairFreeList = B3_NULL_INDEX;

	// The root and the empty node always exist
	tree.nodes[B3_ROOT_NODE] = b3MakeEmptyNode();
	tree.nodes[B3_ROOT_NODE + 1] = b3MakeEmptyNode();
	tree.parents[B3_ROOT_NODE] = B3_NULL_INDEX;
	tree.parents[B3_ROOT_NODE + 1] = B3_NULL_INDEX;

	// This is the bump index.
	tree.nodeEnd = 2;
	tree.dfsOrdered = true;

	tree.proxyCapacity = capacity;
	tree.proxyCount = 0;
	tree.proxies = (b3TreeProxy*)b3AllocZero( tree.proxyCapacity * sizeof( b3TreeProxy ) );

	// Build a linked list for the free list.
	for ( int i = 0; i < tree.proxyCapacity - 1; ++i )
	{
		tree.proxies[i].node = B3_NULL_INDEX;
		tree.proxies[i].next = i + 1;
	}

	tree.proxies[tree.proxyCapacity - 1].node = B3_NULL_INDEX;
	tree.proxies[tree.proxyCapacity - 1].next = B3_NULL_INDEX;
	tree.proxyFreeList = 0;

	tree.swapNodes = NULL;
	tree.leafIndices = NULL;
	tree.leafNodes = NULL;
	tree.leafBoxes = NULL;
	tree.leafCenters = NULL;
	tree.binIndices = NULL;
	tree.rebuildCapacity = 0;

	return tree;
}

void b3DynamicTree_Destroy( b3DynamicTree* tree )
{
	b3Free( tree->nodes, tree->nodeCapacity * sizeof( b3TreeNode ) );
	b3Free( tree->parents, tree->nodeCapacity * sizeof( int32_t ) );
	b3Free( tree->proxies, tree->proxyCapacity * sizeof( b3TreeProxy ) );
	b3Free( tree->swapNodes, tree->nodeCapacity * sizeof( b3TreeNode ) );
	b3Free( tree->leafIndices, tree->rebuildCapacity * sizeof( int32_t ) );
	b3Free( tree->leafNodes, tree->rebuildCapacity * sizeof( b3TreeNode ) );
	b3Free( tree->leafBoxes, tree->rebuildCapacity * sizeof( b3AABB ) );
	b3Free( tree->leafCenters, tree->rebuildCapacity * sizeof( b3Vec3 ) );
	b3Free( tree->binIndices, tree->rebuildCapacity * sizeof( int32_t ) );

	memset( tree, 0, sizeof( b3DynamicTree ) );
}

// Allocate a proxy from the pool. Grow the pool if necessary.
static int b3AllocateProxy( b3DynamicTree* tree )
{
	// Expand the pool as needed.
	if ( tree->proxyFreeList == B3_NULL_INDEX )
	{
		B3_ASSERT( tree->proxyCount == tree->proxyCapacity );

		// The free list is empty. Rebuild a bigger pool.
		// A restored pool can have capacity 1, so guarantee strict growth.
		int oldCapacity = tree->proxyCapacity;
		tree->proxyCapacity += b3MaxInt( oldCapacity >> 1, 1 );
		tree->proxies = B3_GROW_ZERO( tree->proxies, oldCapacity, tree->proxyCapacity );

		// Build a linked list for the free list.
		for ( int i = oldCapacity; i < tree->proxyCapacity - 1; ++i )
		{
			tree->proxies[i].node = B3_NULL_INDEX;
			tree->proxies[i].next = i + 1;
		}

		tree->proxies[tree->proxyCapacity - 1].node = B3_NULL_INDEX;
		tree->proxies[tree->proxyCapacity - 1].next = B3_NULL_INDEX;
		tree->proxyFreeList = oldCapacity;
	}

	// Peel a proxy off the free list.
	int proxyIndex = tree->proxyFreeList;
	tree->proxyFreeList = tree->proxies[proxyIndex].next;
	memset( tree->proxies + proxyIndex, 0, sizeof( b3TreeProxy ) );
	tree->proxies[proxyIndex].node = B3_NULL_INDEX;
	tree->proxies[proxyIndex].next = B3_NULL_INDEX;
	++tree->proxyCount;
	return proxyIndex;
}

// Return a proxy to the pool.
static void b3FreeProxy( b3DynamicTree* tree, int proxyId )
{
	B3_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	B3_ASSERT( 0 < tree->proxyCount );
	tree->proxies[proxyId].node = B3_NULL_INDEX;
	tree->proxies[proxyId].next = tree->proxyFreeList;
	tree->proxyFreeList = proxyId;
	--tree->proxyCount;
}

static inline int b3GetNodeHeight( const b3TreeNode* node )
{
	return b3IsLeaf( node ) ? 0 : node->height;
}

// The internal node above a children pair
static inline b3TreeNode b3MakeInternalNode( const b3TreeNode* nodes, int pair )
{
	const b3TreeNode* c1 = nodes + pair;
	const b3TreeNode* c2 = nodes + pair + 1;

	b3TreeNode node = { 0 };
	b3StoreAABBV( &node.aabb, b3UnionPairV( nodes + pair ), true );
	node.flagIndex = (uint32_t)pair | ( ( c1->flagIndex | c2->flagIndex ) & B3_MOVED_NODE );
	node.height = 1 + b3MaxInt( b3GetNodeHeight( c1 ), b3GetNodeHeight( c2 ) );
	return node;
}

static inline b3TreeNode b3MakeLeafNode( b3AABB aabb, int proxyId, uint64_t userData, bool moved )
{
	b3TreeNode node = { 0 };
	node.aabb = aabb;
	node.flagIndex = (uint32_t)proxyId | B3_LEAF_NODE | ( moved ? B3_MOVED_NODE : 0 );
	node.shapeIndex = (int32_t)(uint32_t)userData;
	return node;
}

// A node landed at a new index, so tell what hangs below it
static inline void b3LinkChildren( b3DynamicTree* tree, int nodeIndex )
{
	const b3TreeNode* node = tree->nodes + nodeIndex;
	if ( b3IsLeaf( node ) )
	{
		tree->proxies[b3GetProxyId( node )].node = nodeIndex;
	}
	else
	{
		int pair = b3GetLeftChild( node );
		tree->parents[pair] = nodeIndex;
		tree->parents[pair + 1] = nodeIndex;
	}
}

// The sweep refit visits indices from high to low, so it needs every child above its parent
static inline bool b3IsNodeOrdered( const b3TreeNode* nodes, int nodeIndex )
{
	const b3TreeNode* node = nodes + nodeIndex;
	return b3IsLeaf( node ) || nodeIndex < b3GetLeftChild( node );
}

// Greedy algorithm for sibling selection using the SAH
// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
// 1: make a new parent for A and D : E-(A-(B,C), D)
// 2: associate D with B
//   a: B is a leaf : A-(E-(B,D), C)
//   b: B is an internal node: A-(B{D},C)
// 3: associate D with C
//   a: C is a leaf : A-(B, E-(C,D))
//   b: C is an internal node: A-(B, C{D})
// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.

// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
// cost of sibling H = area(union(H, D)) + increased area of ancestors

// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
// case1: D becomes a sibling of B
// case2: D becomes a descendant of B along with a new internal node of area(D).
static int b3FindBestSibling( const b3DynamicTree* tree, b3AABB boxD )
{
	b3Vec3 centerD = b3AABB_Center( boxD );
	float areaD = b3Perimeter( boxD );

	const b3TreeNode* nodes = tree->nodes;
	int nodeIndex = B3_ROOT_NODE;

	if ( b3IsLeaf( nodes + nodeIndex ) )
	{
		return nodeIndex;
	}

	b3AABB rootBox = nodes[nodeIndex].aabb;

	// Area of current node
	float areaBase = b3Perimeter( rootBox );

	// Area of inflated node
	float directCost = b3Perimeter( b3AABB_Union( rootBox, boxD ) );
	float inheritedCost = 0.0f;

	int bestSibling = nodeIndex;
	float bestCost = directCost;

	// Descend the tree, following a single greedy path.
	for ( ;; )
	{
		int child1 = b3GetLeftChild( nodes + nodeIndex );
		int child2 = child1 + 1;

		// Cost of creating a new parent for this node and the new leaf
		float cost = directCost + inheritedCost;

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if ( cost < bestCost )
		{
			bestSibling = nodeIndex;
			bestCost = cost;
		}

		// Inheritance cost seen by children
		inheritedCost += directCost - areaBase;

		bool leaf1 = b3IsLeaf( nodes + child1 );
		bool leaf2 = b3IsLeaf( nodes + child2 );

		// Cost of descending into child 1
		float lowerCost1 = FLT_MAX;
		b3AABB box1 = nodes[child1].aabb;
		float directCost1 = b3Perimeter( b3AABB_Union( box1, boxD ) );
		float area1 = 0.0f;
		if ( leaf1 )
		{
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			float cost1 = directCost1 + inheritedCost;
			if ( cost1 < bestCost )
			{
				bestSibling = child1;
				bestCost = cost1;
			}
		}
		else
		{
			// Child 1 is an internal node
			area1 = b3Perimeter( box1 );

			// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
			// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
			// 2. A descendant of child1 could be the sibling with the lower bound cost of
			//       cost1 = inheritedCost + (directCost1 - area1) + areaD
			// This minimum here leads to the minimum of these two costs.
			lowerCost1 = inheritedCost + directCost1 + b3MinFloat( areaD - area1, 0.0f );
		}

		// Cost of descending into child 2
		float lowerCost2 = FLT_MAX;
		b3AABB box2 = nodes[child2].aabb;
		float directCost2 = b3Perimeter( b3AABB_Union( box2, boxD ) );
		float area2 = 0.0f;
		if ( leaf2 )
		{
			float cost2 = directCost2 + inheritedCost;
			if ( cost2 < bestCost )
			{
				bestSibling = child2;
				bestCost = cost2;
			}
		}
		else
		{
			area2 = b3Perimeter( box2 );
			lowerCost2 = inheritedCost + directCost2 + b3MinFloat( areaD - area2, 0.0f );
		}

		if ( leaf1 && leaf2 )
		{
			break;
		}

		// Can the cost possibly be decreased?
		if ( bestCost <= lowerCost1 && bestCost <= lowerCost2 )
		{
			break;
		}

		if ( lowerCost1 == lowerCost2 && leaf1 == false )
		{
			B3_ASSERT( lowerCost1 < FLT_MAX );
			B3_ASSERT( lowerCost2 < FLT_MAX );

			// No clear choice based on lower bound surface area. This can happen when both
			// children fully contain D. Fall back to node distance.
			b3Vec3 d1 = b3Sub( b3AABB_Center( box1 ), centerD );
			b3Vec3 d2 = b3Sub( b3AABB_Center( box2 ), centerD );
			lowerCost1 = b3LengthSquared( d1 );
			lowerCost2 = b3LengthSquared( d2 );
		}

		// Descend
		if ( lowerCost1 < lowerCost2 && leaf1 == false )
		{
			nodeIndex = child1;
			areaBase = area1;
			directCost = directCost1;
		}
		else
		{
			nodeIndex = child2;
			areaBase = area2;
			directCost = directCost2;
		}
	}

	return bestSibling;
}

// Swap a child of A with a grandchild under the other child.
// Example swapping B with G:
// (A (B (D E) C (F G)) -> (A (G) C (F B (D E))
static void b3SwapNodes( b3DynamicTree* tree, int iDown, int iUp )
{
	b3TreeNode* nodes = tree->nodes;
	B3_SWAP( nodes[iDown], nodes[iUp] );
	b3LinkChildren( tree, iDown );
	b3LinkChildren( tree, iUp );

	if ( b3IsNodeOrdered( nodes, iDown ) == false || b3IsNodeOrdered( nodes, iUp ) == false )
	{
		tree->dfsOrdered = false;
	}

	// The sibling of the node that went down holds a different subtree now
	int iC = iDown ^ 1;
	nodes[iC] = b3MakeInternalNode( nodes, b3GetLeftChild( nodes + iC ) );
}

// Perform a left or right rotation if node A is imbalanced.
// Tree: (A (B (D E) C (F G))
static void b3RotateNodes( b3DynamicTree* tree, int iA )
{
	b3TreeNode* nodes = tree->nodes;
	const b3TreeNode* A = nodes + iA;
	B3_ASSERT( b3IsLeaf( A ) == false );

	int iB = b3GetLeftChild( A );
	int iC = iB + 1;
	const b3TreeNode* B = nodes + iB;
	const b3TreeNode* C = nodes + iC;

	bool leafB = b3IsLeaf( B );
	bool leafC = b3IsLeaf( C );
	if ( leafB && leafC )
	{
		return;
	}

	int bestDown = B3_NULL_INDEX;
	int bestUp = B3_NULL_INDEX;
	float bestDelta = 0.0f;

	if ( leafC == false )
	{
		// Swap B with F or G
		int iF = b3GetLeftChild( C );
		int iG = iF + 1;
		float areaC = b3Perimeter( C->aabb );

		// B <-> F then C (B G)
		float deltaBF = b3Perimeter( b3UnionV( B->aabb, nodes[iG].aabb ) ) - areaC;
		if ( deltaBF < bestDelta )
		{
			bestDown = iB;
			bestUp = iF;
			bestDelta = deltaBF;
		}

		// B <-> G then C (F B)
		float deltaBG = b3Perimeter( b3UnionV( B->aabb, nodes[iF].aabb ) ) - areaC;
		if ( deltaBG < bestDelta )
		{
			bestDown = iB;
			bestUp = iG;
			bestDelta = deltaBG;
		}
	}

	if ( leafB == false )
	{
		// Swap C with D or E
		int iD = b3GetLeftChild( B );
		int iE = iD + 1;
		float areaB = b3Perimeter( B->aabb );

		// C <-> D then B (C E)
		float deltaCD = b3Perimeter( b3UnionV( C->aabb, nodes[iE].aabb ) ) - areaB;
		if ( deltaCD < bestDelta )
		{
			bestDown = iC;
			bestUp = iD;
			bestDelta = deltaCD;
		}

		// C <-> E then B (D C)
		float deltaCE = b3Perimeter( b3UnionV( C->aabb, nodes[iD].aabb ) ) - areaB;
		if ( deltaCE < bestDelta )
		{
			bestDown = iC;
			bestUp = iE;
			bestDelta = deltaCE;
		}
	}

	if ( bestDown != B3_NULL_INDEX )
	{
		b3SwapNodes( tree, bestDown, bestUp );
	}
}

static void b3InsertLeaf( b3DynamicTree* tree, b3AABB aabb, int proxyId, bool moved, bool shouldRotate )
{
	b3TreeProxy* proxy = tree->proxies + proxyId;
	b3TreeNode leaf = b3MakeLeafNode( aabb, proxyId, proxy->userData, moved );

	if ( b3IsEmptyNode( tree->nodes + B3_ROOT_NODE ) )
	{
		tree->nodes[B3_ROOT_NODE] = leaf;
		proxy->node = B3_ROOT_NODE;
		return;
	}

	int sibling = b3FindBestSibling( tree, aabb );

	// The sibling's position becomes the new parent. The sibling moves down into a new pair
	// beside the leaf, which puts it below its own children when it is internal.
	int pair = b3AllocateSiblingPair( tree );
	b3TreeNode* nodes = tree->nodes;
	int32_t* parents = tree->parents;

	nodes[pair] = nodes[sibling];
	nodes[pair + 1] = leaf;
	parents[pair] = sibling;
	parents[pair + 1] = sibling;
	b3LinkChildren( tree, pair );
	proxy->node = pair + 1;

	nodes[sibling] = b3MakeInternalNode( nodes, pair );

	if ( b3IsNodeOrdered( nodes, sibling ) == false || b3IsNodeOrdered( nodes, pair ) == false )
	{
		tree->dfsOrdered = false;
	}

	// Walk back up the tree refitting ancestors, the root included.
	int index = sibling;
	while ( index != B3_NULL_INDEX )
	{
		if ( shouldRotate )
		{
			b3RotateNodes( tree, index );
		}

		nodes[index] = b3MakeInternalNode( nodes, b3GetLeftChild( nodes + index ) );
		index = parents[index];
	}
}

static void b3RemoveLeaf( b3DynamicTree* tree, int proxyId )
{
	b3TreeNode* nodes = tree->nodes;
	int32_t* parents = tree->parents;

	int leaf = tree->proxies[proxyId].node;
	B3_ASSERT( 0 <= leaf && leaf < tree->nodeEnd );
	B3_ASSERT( b3IsLeaf( nodes + leaf ) && b3GetProxyId( nodes + leaf ) == proxyId );

	if ( leaf == B3_ROOT_NODE )
	{
		nodes[B3_ROOT_NODE] = b3MakeEmptyNode();
		return;
	}

	// The sibling takes the parent's position and the pair is freed. This keeps the order,
	// the sibling only moves down.
	int parent = parents[leaf];
	nodes[parent] = nodes[leaf ^ 1];
	b3LinkChildren( tree, parent );
	b3FreePair( tree, leaf & ~1 );

	// Update ancestors.
	int index = parents[parent];
	while ( index != B3_NULL_INDEX )
	{
		nodes[index] = b3MakeInternalNode( nodes, b3GetLeftChild( nodes + index ) );
		index = parents[index];
	}
}

// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
// the node pool.
int b3CreateTreeProxyInternal( b3DynamicTree* tree, b3AABB aabb, uint64_t categoryBits, uint64_t userData, bool markMoved )
{
	B3_VALIDATE( b3IsValidAABB( aabb ) );

	int proxyId = b3AllocateProxy( tree );

	b3TreeProxy* proxy = tree->proxies + proxyId;
	proxy->categoryBits = categoryBits;
	proxy->userData = userData;

	bool shouldRotate = true;
	b3InsertLeaf( tree, aabb, proxyId, markMoved, shouldRotate );

	return proxyId;
}

int b3DynamicTree_CreateProxy( b3DynamicTree* tree, b3AABB aabb, uint64_t categoryBits, uint64_t userData )
{
	return b3CreateTreeProxyInternal( tree, aabb, categoryBits, userData, false );
}

void b3DynamicTree_DestroyProxy( b3DynamicTree* tree, int proxyId )
{
	B3_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );

	b3RemoveLeaf( tree, proxyId );
	b3FreeProxy( tree, proxyId );
}

int b3DynamicTree_GetProxyCount( const b3DynamicTree* tree )
{
	return tree->proxyCount;
}

void b3DynamicTree_MoveProxyInternal( b3DynamicTree* tree, int proxyId, b3AABB aabb, bool markMoved )
{
	B3_VALIDATE( b3IsValidAABB( aabb ) );
	B3_VALIDATE( aabb.upperBound.x - aabb.lowerBound.x < B3_HUGE );
	B3_VALIDATE( aabb.upperBound.y - aabb.lowerBound.y < B3_HUGE );
	B3_VALIDATE( aabb.upperBound.z - aabb.lowerBound.z < B3_HUGE );
	B3_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );

	b3RemoveLeaf( tree, proxyId );

	bool shouldRotate = false;
	b3InsertLeaf( tree, aabb, proxyId, markMoved, shouldRotate );
}

void b3DynamicTree_MoveProxy( b3DynamicTree* tree, int proxyId, b3AABB aabb )
{
	b3DynamicTree_MoveProxyInternal( tree, proxyId, aabb, false );
}

void b3DynamicTree_EnlargeProxy( b3DynamicTree* tree, int proxyId, b3AABB aabb )
{
	B3_VALIDATE( b3IsValidAABB( aabb ) );
	B3_VALIDATE( aabb.upperBound.x - aabb.lowerBound.x < B3_HUGE );
	B3_VALIDATE( aabb.upperBound.y - aabb.lowerBound.y < B3_HUGE );
	B3_VALIDATE( aabb.upperBound.z - aabb.lowerBound.z < B3_HUGE );
	B3_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );

	b3TreeNode* nodes = tree->nodes;
	const int32_t* parents = tree->parents;

	int index = tree->proxies[proxyId].node;
	b3TreeNode* node = nodes + index;
	B3_ASSERT( b3IsLeaf( node ) );

	// Caller must ensure this
	B3_VALIDATE( b3AABB_Contains( node->aabb, aabb ) == false );

	node->aabb = aabb;
	node->flagIndex |= B3_MOVED_NODE;

	index = parents[index];
	while ( index != B3_NULL_INDEX )
	{
		node = nodes + index;
		bool changed = b3EnlargeAABB( &node->aabb, aabb );

		// This is marked to ensure the root is marked in this loop or the one below.
		node->flagIndex |= B3_MOVED_NODE;

		index = parents[index];

		if ( changed == false )
		{
			break;
		}
	}

	// Mark all the way up to the root.
	while ( index != B3_NULL_INDEX )
	{
		node = nodes + index;
		if ( node->flagIndex & B3_MOVED_NODE )
		{
			// Early out because this ancestor was previously ascended and marked as moved.
			break;
		}

		node->flagIndex |= B3_MOVED_NODE;
		index = parents[index];
	}
}

void b3DynamicTree_SetCategoryBits( b3DynamicTree* tree, int proxyId, uint64_t categoryBits )
{
	B3_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	tree->proxies[proxyId].categoryBits = categoryBits;
}

uint64_t b3DynamicTree_GetCategoryBits( b3DynamicTree* tree, int proxyId )
{
	B3_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	return tree->proxies[proxyId].categoryBits;
}

int b3DynamicTree_GetHeight( const b3DynamicTree* tree )
{
	return b3GetNodeHeight( tree->nodes + B3_ROOT_NODE );
}

// The area ratio is the thing that SAH seeks to minimize. SAH
// cannot do anything about leaf boxes or the root box. It seeks
// to minimize the area of all non-root internal nodes. Divide this
// by the root area to make the metric non-dimensional.
// So this becomes a meaningful measure of tree quality.
float b3DynamicTree_GetAreaRatio( const b3DynamicTree* tree )
{
	if ( tree->proxyCount == 0 )
	{
		return 0.0f;
	}

	const b3TreeNode* nodes = tree->nodes;
	float rootArea = b3Perimeter( nodes[B3_ROOT_NODE].aabb );
	if ( rootArea <= 0.0f )
	{
		return 0.0f;
	}

	// Free nodes and the empty node are leaf tagged.
	float internalArea = 0.0f;
	int nodeEnd = tree->nodeEnd;
	for ( int i = 2; i < nodeEnd; ++i )
	{
		if ( b3IsLeaf( nodes + i ) == false )
		{
			internalArea += b3Perimeter( nodes[i].aabb );
		}
	}

	return internalArea / rootArea;
}

b3AABB b3DynamicTree_GetRootBounds( const b3DynamicTree* tree )
{
	if ( tree->proxyCount == 0 )
	{
		return B3_LITERAL( b3AABB ){ b3Vec3_zero, b3Vec3_zero };
	}

	return tree->nodes[B3_ROOT_NODE].aabb;
}

#if B3_ENABLE_VALIDATION

static int b3ValidateSubtree( const b3DynamicTree* tree, int nodeIndex, int* leafCount )
{
	B3_ASSERT( 0 <= nodeIndex && nodeIndex < tree->nodeEnd );
	const b3TreeNode* node = tree->nodes + nodeIndex;
	B3_ASSERT( b3IsEmptyNode( node ) == false );
	B3_ASSERT( b3IsValidAABB( node->aabb ) );

	if ( b3IsLeaf( node ) )
	{
		int proxyId = b3GetProxyId( node );
		B3_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
		B3_ASSERT( tree->proxies[proxyId].node == nodeIndex );
		B3_ASSERT( (int32_t)(uint32_t)tree->proxies[proxyId].userData == node->shapeIndex );
		*leafCount += 1;
		return 0;
	}

	int pair = b3GetLeftChild( node );
	B3_ASSERT( ( pair & 1 ) == 0 && 2 <= pair && pair < tree->nodeEnd );
	B3_ASSERT( tree->parents[pair] == nodeIndex );
	B3_ASSERT( tree->parents[pair + 1] == nodeIndex );
	B3_ASSERT( tree->dfsOrdered == false || nodeIndex < pair );

	const b3TreeNode* c1 = tree->nodes + pair;
	const b3TreeNode* c2 = tree->nodes + pair + 1;
	B3_ASSERT( b3AABB_Contains( node->aabb, c1->aabb ) );
	B3_ASSERT( b3AABB_Contains( node->aabb, c2->aabb ) );
	B3_ASSERT( b3IsNodeMoved( node ) == ( b3IsNodeMoved( c1 ) || b3IsNodeMoved( c2 ) ) );

	// A bad tree can stack overflow, but that is validation on its own.
	int height1 = b3ValidateSubtree( tree, pair, leafCount );
	int height2 = b3ValidateSubtree( tree, pair + 1, leafCount );
	int height = 1 + b3MaxInt( height1, height2 );
	B3_ASSERT( node->height == height );
	return height;
}

#endif

void b3DynamicTree_Validate( const b3DynamicTree* tree )
{
#if B3_ENABLE_VALIDATION
	B3_ASSERT( 2 <= tree->nodeEnd && tree->nodeEnd <= tree->nodeCapacity );
	B3_ASSERT( ( tree->nodeEnd & 1 ) == 0 );
	B3_ASSERT( tree->parents[B3_ROOT_NODE] == B3_NULL_INDEX );
	B3_ASSERT( b3IsEmptyNode( tree->nodes + B3_ROOT_NODE + 1 ) );

	int freePairCount = 0;
	int pair = tree->pairFreeList;
	while ( pair != B3_NULL_INDEX )
	{
		B3_ASSERT( ( pair & 1 ) == 0 && 2 <= pair && pair < tree->nodeEnd );
		B3_ASSERT( b3IsEmptyNode( tree->nodes + pair ) );
		B3_ASSERT( b3IsEmptyNode( tree->nodes + pair + 1 ) );
		pair = tree->parents[pair];
		++freePairCount;
		B3_ASSERT( 2 * freePairCount < tree->nodeEnd );
	}

	// Validate proxy free list.
	int freeProxyCount = 0;
	int freeIndex = tree->proxyFreeList;
	while ( freeIndex != B3_NULL_INDEX )
	{
		B3_ASSERT( 0 <= freeIndex && freeIndex < tree->proxyCapacity );
		B3_ASSERT( tree->proxies[freeIndex].node == B3_NULL_INDEX );
		freeIndex = tree->proxies[freeIndex].next;
		++freeProxyCount;
	}
	B3_ASSERT( tree->proxyCount + freeProxyCount == tree->proxyCapacity );

	B3_ASSERT( tree->nodeEnd == 2 * b3MaxInt( tree->proxyCount, 1 ) + 2 * freePairCount );

	if ( tree->proxyCount == 0 )
	{
		B3_ASSERT( b3IsEmptyNode( tree->nodes + B3_ROOT_NODE ) );
		return;
	}

	int leafCount = 0;
	b3ValidateSubtree( tree, B3_ROOT_NODE, &leafCount );
	B3_ASSERT( leafCount == tree->proxyCount );

#else
	B3_UNUSED( tree );
#endif
}

void b3DynamicTree_ValidateNoMoved( const b3DynamicTree* tree )
{
#if B3_ENABLE_VALIDATION == 1
	const b3TreeNode* nodes = tree->nodes;
	int nodeEnd = tree->nodeEnd;
	for ( int i = 0; i < nodeEnd; ++i )
	{
		B3_ASSERT( b3IsNodeMoved( nodes + i ) == false );
	}
#else
	B3_UNUSED( tree );
#endif
}

int b3DynamicTree_GetByteCount( const b3DynamicTree* tree )
{
	size_t size = sizeof( b3DynamicTree );
	size += tree->nodeCapacity * sizeof( b3TreeNode );
	size += tree->nodeCapacity * sizeof( int32_t );
	size += tree->proxyCapacity * sizeof( b3TreeProxy );
	size += tree->swapNodes == NULL ? 0 : tree->nodeCapacity * sizeof( b3TreeNode );

	// leafIndices
	size += tree->rebuildCapacity * sizeof( int32_t );
	// leafNodes
	size += tree->rebuildCapacity * sizeof( b3TreeNode );
	// leafBoxes
	size += tree->rebuildCapacity * sizeof( b3AABB );
	// leafCenters
	size += tree->rebuildCapacity * sizeof( b3Vec3 );
	// binIndices
	size += tree->rebuildCapacity * sizeof( int32_t );

	return (int)size;
}

B3_FORCE_INLINE bool b3TestCategory( uint64_t categoryBits, uint64_t maskBits, bool requireAllBits )
{
	return requireAllBits ? ( categoryBits & maskBits ) == maskBits : ( categoryBits & maskBits ) != 0;
}

b3TreeStats b3DynamicTree_Query( const b3DynamicTree* tree, b3AABB aabb, uint64_t maskBits, bool requireAllBits,
								 b3TreeQueryCallbackFcn* callback, void* context )
{
	b3TreeStats result = { 0 };

	if ( tree->proxyCount == 0 )
	{
		return result;
	}

	const b3TreeNode* nodes = tree->nodes;

	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b3GetRootPair( nodes );

	b3AABBV boxv = b3LoadAABBV( &aabb );

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		result.nodeVisits += 1;

		for ( int i = 0; i < 2; ++i )
		{
			const b3TreeNode* node = nodes + pair + i;
			if ( b3OverlapNode( boxv, node ) == false )
			{
				continue;
			}

			if ( b3IsLeaf( node ) )
			{
				int proxyId = b3GetProxyId( node );
				const b3TreeProxy* proxy = tree->proxies + proxyId;
				if ( b3TestCategory( proxy->categoryBits, maskBits, requireAllBits ) )
				{
					bool proceed = callback( proxyId, proxy->userData, context );
					result.leafVisits += 1;

					if ( proceed == false )
					{
						return result;
					}
				}
			}
			else
			{
				if ( stackCount < B3_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = b3GetLeftChild( node );
				}
				else
				{
					B3_ASSERT( stackCount < B3_TREE_STACK_SIZE - 1 );
				}
			}
		}
	}

	return result;
}

B3_FORCE_INLINE float b3DistanceToNodeSqr( b3Vec3 point, const b3TreeNode* node )
{
	b3Vec3 r = b3Sub( point, b3Clamp( point, node->aabb.lowerBound, node->aabb.upperBound ) );
	return b3Dot( r, r );
}

struct b3QueryClosestItem
{
	int nodeIndex;
	float distanceToNodeSqr;
};

b3TreeStats b3DynamicTree_QueryClosest( const b3DynamicTree* tree, b3Vec3 point, uint64_t maskBits, bool requireAllBits,
										b3TreeQueryClosestCallbackFcn* callback, void* context, float* minDistanceSqr )
{
	b3TreeStats result = { 0 };

	if ( tree->proxyCount == 0 )
	{
		return result;
	}

	const b3TreeNode* nodes = tree->nodes;

	float minSqr = *minDistanceSqr;
	struct b3QueryClosestItem stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;

	int rootPair = b3GetRootPair( nodes );
	struct b3QueryClosestItem seed1 = {
		.nodeIndex = rootPair,
		.distanceToNodeSqr = b3DistanceToNodeSqr( point, nodes + rootPair ),
	};

	if ( rootPair == B3_ROOT_NODE )
	{
		stack[stackCount++] = seed1;
	}
	else
	{
		struct b3QueryClosestItem seed2 = {
			.nodeIndex = rootPair + 1,
			.distanceToNodeSqr = b3DistanceToNodeSqr( point, nodes + rootPair + 1 ),
		};

		// Ensure we iterate the closest child first as we pop off the stack
		if ( seed2.distanceToNodeSqr < seed1.distanceToNodeSqr )
		{
			stack[stackCount++] = seed1;
			stack[stackCount++] = seed2;
		}
		else
		{
			stack[stackCount++] = seed2;
			stack[stackCount++] = seed1;
		}
	}

	while ( stackCount > 0 )
	{
		struct b3QueryClosestItem item = stack[--stackCount];
		const b3TreeNode* node = nodes + item.nodeIndex;
		result.nodeVisits += 1;

		if ( item.distanceToNodeSqr >= minSqr )
		{
			continue;
		}

		if ( b3IsLeaf( node ) )
		{
			int proxyId = b3GetProxyId( node );
			const b3TreeProxy* proxy = tree->proxies + proxyId;
			if ( b3TestCategory( proxy->categoryBits, maskBits, requireAllBits ) == false )
			{
				continue;
			}

			// callback to user code with minimum distance squared so far and proxy id
			float dd = callback( minSqr, proxyId, proxy->userData, context );

			if ( dd < minSqr )
			{
				minSqr = dd;
			}

			result.leafVisits += 1;
			continue;
		}

		if ( stackCount + 2 > B3_TREE_STACK_SIZE )
		{
			B3_ASSERT( stackCount + 2 <= B3_TREE_STACK_SIZE );
			continue;
		}

		int pair = b3GetLeftChild( node );

		// Store the distance to node in the stack instead of recomputing after pop
		struct b3QueryClosestItem item1 = {
			.nodeIndex = pair,
			.distanceToNodeSqr = b3DistanceToNodeSqr( point, nodes + pair ),
		};

		struct b3QueryClosestItem item2 = {
			.nodeIndex = pair + 1,
			.distanceToNodeSqr = b3DistanceToNodeSqr( point, nodes + pair + 1 ),
		};

		// Ensure we iterate the closest child first as we pop off the stack
		if ( item2.distanceToNodeSqr < item1.distanceToNodeSqr )
		{
			stack[stackCount++] = item1;
			stack[stackCount++] = item2;
		}
		else
		{
			stack[stackCount++] = item2;
			stack[stackCount++] = item1;
		}
	}

	*minDistanceSqr = minSqr;

	return result;
}

// A lot of optimization work went into this. The children of a popped pair are both tested
// before either is followed, then the survivors are ordered so the closer one narrows first.
b3TreeStats b3DynamicTree_RayCast( const b3DynamicTree* tree, const b3RayCastInput* input, uint64_t maskBits, bool requireAllBits,
								   b3TreeRayCastCallbackFcn* callback, void* context )
{
	b3TreeStats result = { 0 };

	if ( tree->proxyCount == 0 )
	{
		return result;
	}

	b3Vec3 p1 = input->origin;
	b3Vec3 d = input->translation;

	b3V32 pv1 = b3LoadV( &p1.x );
	b3V32 dv = b3LoadV( &d.x );

	float maxFraction = input->maxFraction;

	b3Vec3 p2 = b3MulAdd( p1, maxFraction, d );

	// Build a bounding box for the segment.
	b3AABB segmentAABB = { b3Min( p1, p2 ), b3Max( p1, p2 ) };
	b3AABBV boxv = b3LoadAABBV( &segmentAABB );

	const b3TreeNode* nodes = tree->nodes;

	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b3GetRootPair( nodes );

	b3RayCastInput subInput = *input;

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		result.nodeVisits += 1;

		const b3TreeNode* hit[2];
		bool isLeaf[2];
		int hitCount = 0;
		for ( int i = 0; i < 2; ++i )
		{
			const b3TreeNode* node = nodes + pair + i;

			if ( b3OverlapNode( boxv, node ) == false )
			{
				continue;
			}

			b3AABB nodeAABB = node->aabb;
			b3V32 lower = b3LoadV( &nodeAABB.lowerBound.x );
			b3V32 upper = b3LoadV( &nodeAABB.upperBound.x );
			if ( b3TestBoundsRayOverlap( lower, upper, pv1, dv ) == false )
			{
				continue;
			}

			isLeaf[hitCount] = b3IsLeaf( node );
			hit[hitCount] = node;
			hitCount += 1;
		}

		if ( hitCount == 2 && isLeaf[0] == false && isLeaf[1] == false )
		{
			b3Vec3 center1 = b3AABB_Center( hit[0]->aabb );
			b3Vec3 center2 = b3AABB_Center( hit[1]->aabb );
			float d1 = b3DistanceSquared( center1, p1 );
			float d2 = b3DistanceSquared( center2, p1 );

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if ( d1 < d2 )
			{
				B3_SWAP( hit[0], hit[1] );
			}
		}

		for ( int i = 0; i < hitCount; ++i )
		{
			if ( isLeaf[i] )
			{
				int proxyId = b3GetProxyId( hit[i] );
				const b3TreeProxy* proxy = tree->proxies + proxyId;

				if ( b3TestCategory( proxy->categoryBits, maskBits, requireAllBits ) == false )
				{
					continue;
				}

				subInput.maxFraction = maxFraction;

				float value = callback( &subInput, proxyId, proxy->userData, context );
				result.leafVisits += 1;

				// The user may return -1 to indicate this shape should be skipped

				if ( value == 0.0f )
				{
					// The client has terminated the ray cast.
					return result;
				}

				if ( 0.0f < value && value <= maxFraction )
				{
					// Update segment bounding box.
					maxFraction = value;
					p2 = b3MulAdd( p1, maxFraction, d );
					segmentAABB.lowerBound = b3Min( p1, p2 );
					segmentAABB.upperBound = b3Max( p1, p2 );
					boxv = b3LoadAABBV( &segmentAABB );
				}
			}
			else
			{
				if ( stackCount < B3_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = b3GetLeftChild( hit[i] );
				}
				else
				{
					B3_ASSERT( stackCount < B3_TREE_STACK_SIZE - 1 );
				}
			}
		}
	}

	return result;
}

// Follows structure of ray cast with small tweaks to handle a swept box.
b3TreeStats b3DynamicTree_BoxCast( const b3DynamicTree* tree, const b3BoxCastInput* input, uint64_t maskBits, bool requireAllBits,
								   b3TreeBoxCastCallbackFcn* callback, void* context )
{
	b3TreeStats result = { 0 };

	if ( tree->proxyCount == 0 )
	{
		return result;
	}

	// The caller folds the shape radius and the world origin into the box
	b3AABB originAABB = input->box;

	b3Vec3 p1 = b3AABB_Center( originAABB );
	b3Vec3 extension = b3AABB_Extents( originAABB );

	b3Vec3 d = input->translation;

	b3V32 pv1 = b3LoadV( &p1.x );
	b3V32 dv = b3LoadV( &d.x );
	b3V32 ev = b3LoadV( &extension.x );

	float maxFraction = input->maxFraction;

	// Build total box for the cast
	b3Vec3 t = b3MulSV( maxFraction, input->translation );
	b3AABB totalAABB = {
		b3Min( originAABB.lowerBound, b3Add( originAABB.lowerBound, t ) ),
		b3Max( originAABB.upperBound, b3Add( originAABB.upperBound, t ) ),
	};
	b3AABBV boxv = b3LoadAABBV( &totalAABB );

	const b3TreeNode* nodes = tree->nodes;

	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b3GetRootPair( nodes );

	b3BoxCastInput subInput = *input;

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		result.nodeVisits += 1;

		const b3TreeNode* hit[2];
		bool isLeaf[2];
		int hitCount = 0;
		for ( int i = 0; i < 2; ++i )
		{
			const b3TreeNode* node = nodes + pair + i;

			if ( b3OverlapNode( boxv, node ) == false )
			{
				continue;
			}

			// radius extension is added to the node in this case
			b3AABB nodeAABB = node->aabb;
			b3V32 lower = b3SubV( b3LoadV( &nodeAABB.lowerBound.x ), ev );
			b3V32 upper = b3AddV( b3LoadV( &nodeAABB.upperBound.x ), ev );
			if ( b3TestBoundsRayOverlap( lower, upper, pv1, dv ) == false )
			{
				continue;
			}

			isLeaf[hitCount] = b3IsLeaf( node );
			hit[hitCount] = node;
			hitCount += 1;
		}

		if ( hitCount == 2 && isLeaf[0] == false && isLeaf[1] == false )
		{
			b3Vec3 center1 = b3AABB_Center( hit[0]->aabb );
			b3Vec3 center2 = b3AABB_Center( hit[1]->aabb );
			float d1 = b3DistanceSquared( center1, p1 );
			float d2 = b3DistanceSquared( center2, p1 );

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if ( d1 < d2 )
			{
				B3_SWAP( hit[0], hit[1] );
			}
		}

		for ( int i = 0; i < hitCount; ++i )
		{
			if ( isLeaf[i] )
			{
				int proxyId = b3GetProxyId( hit[i] );
				const b3TreeProxy* proxy = tree->proxies + proxyId;

				if ( b3TestCategory( proxy->categoryBits, maskBits, requireAllBits ) == false )
				{
					continue;
				}

				subInput.maxFraction = maxFraction;

				float value = callback( &subInput, proxyId, proxy->userData, context );
				result.leafVisits += 1;

				if ( value == 0.0f )
				{
					// The client has terminated the cast.
					return result;
				}

				if ( 0.0f < value && value < maxFraction )
				{
					maxFraction = value;
					t = b3MulSV( maxFraction, input->translation );
					totalAABB.lowerBound = b3Min( originAABB.lowerBound, b3Add( originAABB.lowerBound, t ) );
					totalAABB.upperBound = b3Max( originAABB.upperBound, b3Add( originAABB.upperBound, t ) );
					boxv = b3LoadAABBV( &totalAABB );
				}
			}
			else
			{
				if ( stackCount < B3_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = b3GetLeftChild( hit[i] );
				}
				else
				{
					B3_ASSERT( stackCount < B3_TREE_STACK_SIZE - 1 );
				}
			}
		}
	}

	return result;
}


// Median split == 0, Surface area heuristic == 1
#define B3_TREE_HEURISTIC 0

#if B3_TREE_HEURISTIC == 0

// Median split heuristic
static int b3PartitionMid( int* indices, b3Vec3* centers, int count )
{
	// Handle trivial case
	if ( count <= 2 )
	{
		return count / 2;
	}

	b3Vec3 lowerBound = centers[0];
	b3Vec3 upperBound = centers[0];

	for ( int i = 1; i < count; ++i )
	{
		lowerBound = b3Min( lowerBound, centers[i] );
		upperBound = b3Max( upperBound, centers[i] );
	}

	b3Vec3 d = b3Sub( upperBound, lowerBound );
	b3Vec3 c = b3MulSV( 0.5f, b3Add( lowerBound, upperBound ) );

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	int i1 = 0, i2 = count;
	if ( d.x >= d.y && d.x >= d.z )
	{
		float pivot = c.x;

		while ( i1 < i2 )
		{
			while ( i1 < i2 && centers[i1].x < pivot )
			{
				i1 += 1;
			};

			while ( i1 < i2 && centers[i2 - 1].x >= pivot )
			{
				i2 -= 1;
			};

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b3Vec3 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	else if ( d.y >= d.z )
	{
		float pivot = c.y;

		while ( i1 < i2 )
		{
			while ( i1 < i2 && centers[i1].y < pivot )
			{
				i1 += 1;
			};

			while ( i1 < i2 && centers[i2 - 1].y >= pivot )
			{
				i2 -= 1;
			};

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b3Vec3 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	else
	{
		float pivot = c.z;

		while ( i1 < i2 )
		{
			while ( i1 < i2 && centers[i1].z < pivot )
			{
				i1 += 1;
			};

			while ( i1 < i2 && centers[i2 - 1].z >= pivot )
			{
				i2 -= 1;
			};

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b3Vec3 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	B3_ASSERT( i1 == i2 );

	if ( i1 > 0 && i1 < count )
	{
		return i1;
	}

	return count / 2;
}

#else

#define B3_BIN_COUNT 8

typedef struct b3TreeBin
{
	b3AABB aabb;
	int count;
} b3TreeBin;

typedef struct b3TreePlane
{
	b3AABB leftAABB;
	b3AABB rightAABB;
	int leftCount;
	int rightCount;
} b3TreePlane;

// "On Fast Construction of SAH-based Bounding Volume Hierarchies" by Ingo Wald
// Returns the left child count
static int b3PartitionSAH( int* indices, int* binIndices, b3AABB* boxes, int count )
{
	B3_ASSERT( count > 0 );

	b3TreeBin bins[B3_BIN_COUNT];
	b3TreePlane planes[B3_BIN_COUNT - 1];

	b3Vec3 center = b3AABB_Center( boxes[0] );
	b3AABB centroidAABB;
	centroidAABB.lowerBound = center;
	centroidAABB.upperBound = center;

	for ( int i = 1; i < count; ++i )
	{
		center = b3AABB_Center( boxes[i] );
		centroidAABB.lowerBound = b3Min( centroidAABB.lowerBound, center );
		centroidAABB.upperBound = b3Max( centroidAABB.upperBound, center );
	}

	b3Vec3 d = b3Sub( centroidAABB.upperBound, centroidAABB.lowerBound );

	// Find longest axis
	int axisIndex;
	float invD;
	if ( d.x > d.y )
	{
		axisIndex = 0;
		invD = d.x;
	}
	else
	{
		axisIndex = 1;
		invD = d.y;
	}

	invD = invD > 0.0f ? 1.0f / invD : 0.0f;

	// Initialize bin bounds and count
	for ( int i = 0; i < B3_BIN_COUNT; ++i )
	{
		bins[i].aabb.lowerBound = (b3Vec3){ FLT_MAX, FLT_MAX };
		bins[i].aabb.upperBound = (b3Vec3){ -FLT_MAX, -FLT_MAX };
		bins[i].count = 0;
	}

	// Assign boxes to bins and compute bin boxes
	// TODO_ERIN optimize
	float binCount = B3_BIN_COUNT;
	float lowerBoundArray[2] = { centroidAABB.lowerBound.x, centroidAABB.lowerBound.y };
	float minC = lowerBoundArray[axisIndex];
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 c = b3AABB_Center( boxes[i] );
		float cArray[2] = { c.x, c.y };
		int binIndex = (int)( binCount * ( cArray[axisIndex] - minC ) * invD );
		binIndex = b3ClampInt( binIndex, 0, B3_BIN_COUNT - 1 );
		binIndices[i] = binIndex;
		bins[binIndex].count += 1;
		bins[binIndex].aabb = b3AABB_Union( bins[binIndex].aabb, boxes[i] );
	}

	int planeCount = B3_BIN_COUNT - 1;

	// Prepare all the left planes, candidates for left child
	planes[0].leftCount = bins[0].count;
	planes[0].leftAABB = bins[0].aabb;
	for ( int i = 1; i < planeCount; ++i )
	{
		planes[i].leftCount = planes[i - 1].leftCount + bins[i].count;
		planes[i].leftAABB = b3AABB_Union( planes[i - 1].leftAABB, bins[i].aabb );
	}

	// Prepare all the right planes, candidates for right child
	planes[planeCount - 1].rightCount = bins[planeCount].count;
	planes[planeCount - 1].rightAABB = bins[planeCount].aabb;
	for ( int i = planeCount - 2; i >= 0; --i )
	{
		planes[i].rightCount = planes[i + 1].rightCount + bins[i + 1].count;
		planes[i].rightAABB = b3AABB_Union( planes[i + 1].rightAABB, bins[i + 1].aabb );
	}

	// Find best split to minimize SAH
	float minCost = FLT_MAX;
	int bestPlane = 0;
	for ( int i = 0; i < planeCount; ++i )
	{
		float leftArea = b3Perimeter( planes[i].leftAABB );
		float rightArea = b3Perimeter( planes[i].rightAABB );
		int leftCount = planes[i].leftCount;
		int rightCount = planes[i].rightCount;

		float cost = leftCount * leftArea + rightCount * rightArea;
		if ( cost < minCost )
		{
			bestPlane = i;
			minCost = cost;
		}
	}

	// Partition node indices and boxes using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	int i1 = 0, i2 = count;
	while ( i1 < i2 )
	{
		while ( i1 < i2 && binIndices[i1] < bestPlane )
		{
			i1 += 1;
		};

		while ( i1 < i2 && binIndices[i2 - 1] >= bestPlane )
		{
			i2 -= 1;
		};

		if ( i1 < i2 )
		{
			// Swap indices
			{
				int temp = indices[i1];
				indices[i1] = indices[i2 - 1];
				indices[i2 - 1] = temp;
			}

			// Swap boxes
			{
				b3AABB temp = boxes[i1];
				boxes[i1] = boxes[i2 - 1];
				boxes[i2 - 1] = temp;
			}

			i1 += 1;
			i2 -= 1;
		}
	}
	B3_ASSERT( i1 == i2 );

	if ( i1 > 0 && i1 < count )
	{
		return i1;
	}
	else
	{
		return count / 2;
	}
}

#endif

// Temporary data used to track the rebuild of a tree node.
typedef struct b3RebuildItem
{
	// Where this node is written and the pair its children go in.
	int nodeIndex;
	int pair;
	int childCount;

	// Leaf indices
	int startIndex;
	int splitIndex;
	int endIndex;
} b3RebuildItem;

typedef struct b3CopyItem
{
	int oldPair;
	int newIndex;
} b3CopyItem;

// Bump allocate a pair of sibling nodes. Returns index to the first one.
static inline int b3BumpPair( b3DynamicTree* tree, int parent )
{
	int pair = tree->nodeEnd;
	B3_ASSERT( pair + 2 <= tree->nodeCapacity );
	tree->nodeEnd += 2;
	tree->parents[pair] = parent;
	tree->parents[pair + 1] = parent;
	return pair;
}

static inline void b3SetLeftChild( b3TreeNode* node, int pair )
{
	node->flagIndex = ( node->flagIndex & ~B3_NODE_INDEX_MASK ) | (uint32_t)pair;
}

// Copy a retained subtree from the old array into the rebuilt DFS array using
// a bump allocator.
static void b3CopySubtree( b3DynamicTree* tree, b3TreeNode node, int newIndex )
{
	const b3TreeNode* oldNodes = tree->nodes;
	b3TreeNode* newNodes = tree->swapNodes;
	b3TreeProxy* proxies = tree->proxies;

	if ( b3IsLeaf( &node ) )
	{
		newNodes[newIndex] = node;
		proxies[b3GetProxyId( &node )].node = newIndex;
		return;
	}

	b3CopyItem stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;

	int oldPair = b3GetLeftChild( &node );
	int newPair = b3BumpPair( tree, newIndex );
	b3SetLeftChild( &node, newPair );

	// Copy the subtree root. This can be a left or right sibling (even or odd).
	newNodes[newIndex] = node;

	for ( ;; )
	{
		b3TreeNode* pair = newNodes + newPair;

		// Copy the siblings.
		pair[0] = oldNodes[oldPair];
		pair[1] = oldNodes[oldPair + 1];

		if ( b3IsLeaf( pair + 1 ) )
		{
			// Hook up proxy on right sibling.
			int proxyId = b3GetProxyId( pair + 1 );
			proxies[proxyId].node = newPair + 1;
		}
		else
		{
			// Push the left child of the right sibling.
			if ( stackCount < B3_TREE_STACK_SIZE )
			{
				int leftChild = b3GetLeftChild( pair + 1 );
				stack[stackCount++] = B3_LITERAL( b3CopyItem ){ leftChild, newPair + 1 };
			}
			else
			{
				// todo fail gracefully if the stack runs out in release
				B3_ASSERT( stackCount < B3_TREE_STACK_SIZE );
			}
		}

		if ( b3IsLeaf( pair ) == false )
		{
			// Descend into left subtree.
			int leftIndex = newPair;
			oldPair = b3GetLeftChild( pair );
			newPair = b3BumpPair( tree, leftIndex );
			b3SetLeftChild( newNodes + leftIndex, newPair );
			continue;
		}

		// Hook up proxy on left sibling.
		proxies[b3GetProxyId( pair )].node = newPair;

		if ( stackCount == 0 )
		{
			break;
		}

		// Descend a right subtree that was previously pushed.
		b3CopyItem item = stack[--stackCount];
		oldPair = item.oldPair;
		newPair = b3BumpPair( tree, item.newIndex );
		b3SetLeftChild( newNodes + item.newIndex, newPair );
	}
}

static void b3PlaceLeaf( b3DynamicTree* tree, b3TreeNode leaf, int newIndex )
{
	if ( b3IsLeaf( &leaf ) )
	{
		tree->swapNodes[newIndex] = leaf;
		tree->proxies[b3GetProxyId( &leaf )].node = newIndex;
	}
	else
	{
		b3CopySubtree( tree, leaf, newIndex );
	}
}

static void b3BuildTree( b3DynamicTree* tree, int leafCount )
{
	b3TreeNode* nodes = tree->swapNodes;
	const b3TreeNode* leaves = tree->leafNodes;
	int* leafIndices = tree->leafIndices;

#if B3_TREE_HEURISTIC == 0
	b3Vec3* leafCenters = tree->leafCenters;
#else
	b3AABB* leafBoxes = tree->leafBoxes;
	int* binIndices = tree->binIndices;
#endif

	// Bump allocation into the swap nodes.
	tree->nodeEnd = 2;
	nodes[B3_ROOT_NODE + 1] = b3MakeEmptyNode();
	tree->parents[B3_ROOT_NODE] = B3_NULL_INDEX;
	tree->parents[B3_ROOT_NODE + 1] = B3_NULL_INDEX;

	if ( leafCount == 1 )
	{
		b3PlaceLeaf( tree, leaves[leafIndices[0]], B3_ROOT_NODE );
		return;
	}

	b3RebuildItem stack[B3_TREE_STACK_SIZE];
	int top = 0;

	stack[0].nodeIndex = B3_ROOT_NODE;
	stack[0].pair = b3BumpPair( tree, B3_ROOT_NODE );
	stack[0].childCount = -1;
	stack[0].startIndex = 0;
	stack[0].endIndex = leafCount;
#if B3_TREE_HEURISTIC == 0
	stack[0].splitIndex = b3PartitionMid( leafIndices, leafCenters, leafCount );
#else
	stack[0].splitIndex = b3PartitionSAH( leafIndices, binIndices, leafBoxes, leafCount );
#endif

	while ( true )
	{
		b3RebuildItem* item = stack + top;
		item->childCount += 1;

		if ( item->childCount == 2 )
		{
			// Both children written, so the parent node can be finalized.
			nodes[item->nodeIndex] = b3MakeInternalNode( nodes, item->pair );
			if ( top == 0 )
			{
				break;
			}

			top -= 1;
			continue;
		}

		int slot = item->childCount;
		int startIndex = slot == 0 ? item->startIndex : item->splitIndex;
		int endIndex = slot == 0 ? item->splitIndex : item->endIndex;
		int count = endIndex - startIndex;
		B3_ASSERT( count > 0 );

		int nodeIndex = item->pair + slot;
		if ( count == 1 )
		{
			b3PlaceLeaf( tree, leaves[leafIndices[startIndex]], nodeIndex );
			continue;
		}

		// todo fail gracefully if the stack runs out in release
		B3_ASSERT( top < B3_TREE_STACK_SIZE - 1 );

		top += 1;
		b3RebuildItem* newItem = stack + top;
		newItem->nodeIndex = nodeIndex;
		newItem->pair = b3BumpPair( tree, nodeIndex );
		newItem->childCount = -1;
		newItem->startIndex = startIndex;
		newItem->endIndex = endIndex;
		newItem->splitIndex = startIndex;
#if B3_TREE_HEURISTIC == 0
		newItem->splitIndex += b3PartitionMid( leafIndices + startIndex, leafCenters + startIndex, count );
#else
		newItem->splitIndex += b3PartitionSAH( leafIndices + startIndex, binIndices + startIndex, leafBoxes + startIndex, count );
#endif
	}
}

// Rebuild the stale parts of the tree. The entire tree is put into DFS order. This makes
// refitting much faster. This is done async with threading, so the cost is hidden.
// Not safe to access tree during this operation.
int b3DynamicTree_Rebuild( b3DynamicTree* tree, bool fullBuild )
{
	int proxyCount = tree->proxyCount;
	if ( proxyCount == 0 )
	{
		return 0;
	}

	// An unordered tree is rebuilt even when nothing moved, the sweep refit needs the order
	const b3TreeNode* nodes = tree->nodes;
	const b3TreeNode* root = nodes + B3_ROOT_NODE;
	if ( fullBuild == false && b3IsNodeMoved( root ) == false && tree->dfsOrdered )
	{
		return 0;
	}

	if ( tree->swapNodes == NULL )
	{
		tree->swapNodes = b3AllocZero( tree->nodeCapacity * sizeof( b3TreeNode ) );
	}

	if ( proxyCount > tree->rebuildCapacity )
	{
		int oldCapacity = tree->rebuildCapacity;
		int newCapacity = proxyCount + proxyCount / 2;

		b3Free( tree->leafIndices, oldCapacity * sizeof( int32_t ) );
		tree->leafIndices = (int32_t*)b3Alloc( newCapacity * sizeof( int32_t ) );

		b3Free( tree->leafNodes, oldCapacity * sizeof( b3TreeNode ) );
		tree->leafNodes = (b3TreeNode*)b3Alloc( newCapacity * sizeof( b3TreeNode ) );

#if B3_TREE_HEURISTIC == 0
		b3Free( tree->leafCenters, oldCapacity * sizeof( b3Vec3 ) );
		tree->leafCenters = (b3Vec3*)b3Alloc( newCapacity * sizeof( b3Vec3 ) );
#else
		b3Free( tree->leafBoxes, oldCapacity * sizeof( b3AABB ) );
		tree->leafBoxes = (b3AABB*)b3Alloc( newCapacity * sizeof( b3AABB ) );
		b3Free( tree->binIndices, oldCapacity * sizeof( int32_t ) );
		tree->binIndices = (int32_t*)b3Alloc( newCapacity * sizeof( int32_t ) );
#endif
		tree->rebuildCapacity = newCapacity;
	}

	int* leafIndices = tree->leafIndices;
	b3TreeNode* leaves = tree->leafNodes;
#if B3_TREE_HEURISTIC == 0
	b3Vec3* leafCenters = tree->leafCenters;
#else
	b3AABB* leafBoxes = tree->leafBoxes;
#endif

	// Gather build leaves. A marked internal node is descended and abandoned in the old array.
	// Everything else is a build leaf, kept subtrees included. A leaf root, or an unmarked root
	// on an unordered array, is the one build leaf.
	int leafCount = 0;
	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;
	if ( b3IsLeaf( root ) == false && ( fullBuild || b3IsNodeMoved( root ) ) )
	{
		stack[stackCount++] = b3GetLeftChild( root );
	}
	else
	{
		b3TreeNode node = *root;
		node.flagIndex &= ~B3_MOVED_NODE;
		leafIndices[0] = 0;
		leaves[0] = node;
#if B3_TREE_HEURISTIC == 0
		leafCenters[0] = b3AABB_Center( node.aabb );
#else
		leafBoxes[0] = node.aabb;
#endif
		leafCount = 1;
	}

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		for ( int i = 0; i < 2; ++i )
		{
			b3TreeNode node = nodes[pair + i];
			if ( b3IsLeaf( &node ) == false && ( fullBuild || b3IsNodeMoved( &node ) ) )
			{
				// todo fail gracefully if the stack runs out in release
				B3_ASSERT( stackCount < B3_TREE_STACK_SIZE );
				stack[stackCount++] = b3GetLeftChild( &node );
				continue;
			}

			node.flagIndex &= ~B3_MOVED_NODE;
			leafIndices[leafCount] = leafCount;
			leaves[leafCount] = node;
#if B3_TREE_HEURISTIC == 0
			leafCenters[leafCount] = b3AABB_Center( node.aabb );
#else
			leafBoxes[leafCount] = node.aabb;
#endif
			leafCount += 1;
		}
	}

	B3_ASSERT( 0 < leafCount && leafCount <= proxyCount );

	b3BuildTree( tree, leafCount );

	// The spare is now the tree and the old array is the spare. The build is dense, so the
	// stale tail of the old array sits above the end and is never read.
	B3_SWAP( tree->nodes, tree->swapNodes );
	tree->pairFreeList = B3_NULL_INDEX;
	tree->dfsOrdered = true;

	b3DynamicTree_Validate( tree );
	b3DynamicTree_ValidateNoMoved( tree );

	return leafCount;
}

// Set the moved flag on the ancestors of the proxy. Serial use case.
void b3DynamicTree_MarkProxyMovedSerial( b3DynamicTree* tree, int proxyId )
{
	B3_VALIDATE( 0 <= proxyId && proxyId < tree->proxyCapacity );

	b3TreeNode* nodes = tree->nodes;
	const int32_t* parents = tree->parents;

	int index = tree->proxies[proxyId].node;
	B3_VALIDATE( 0 <= index && index < tree->nodeEnd );
	B3_VALIDATE( b3IsLeaf( nodes + index ) );

	while ( index != B3_NULL_INDEX )
	{
		nodes[index].flagIndex |= B3_MOVED_NODE;
		index = parents[index];
	}
}

// Update a proxy AABB and flag the ancestors as moved. Thread-safe using atomics.
void b3DynamicTree_MarkProxyMoved( b3DynamicTree* tree, int proxyId, b3AABB aabb )
{
	B3_VALIDATE( 0 <= proxyId && proxyId < tree->proxyCapacity );

	b3TreeNode* nodes = tree->nodes;
	const int32_t* parents = tree->parents;

	int index = tree->proxies[proxyId].node;
	B3_VALIDATE( 0 <= index && index < tree->nodeEnd );

	b3TreeNode* node = nodes + index;
	B3_VALIDATE( b3IsLeaf( node ) );
	B3_VALIDATE( b3AABB_Contains( node->aabb, aabb ) == false );

	// This is not raced since it is the leaf.
	node->aabb = aabb;
	node->flagIndex |= B3_MOVED_NODE;

	index = parents[index];
	while ( index != B3_NULL_INDEX )
	{
		node = nodes + index;

		// Read first to avoid the FetchOr if possible.
		if ( b3AtomicLoadU32Raw( &node->flagIndex ) & B3_MOVED_NODE )
		{
			break;
		}

		uint32_t previousFlags = b3AtomicFetchOrU32( &node->flagIndex, B3_MOVED_NODE );
		if ( previousFlags & B3_MOVED_NODE )
		{
			// Ancestor already visited.
			break;
		}

		index = parents[index];
	}
}

// Clear the moved flags from the entire tree.
void b3DynamicTree_ClearMoved( b3DynamicTree* tree )
{
	b3TreeNode* nodes = tree->nodes;

	b3TreeNode* root = nodes + B3_ROOT_NODE;
	if ( b3IsNodeMoved( root ) == false )
	{
		return;
	}

	root->flagIndex &= ~B3_MOVED_NODE;
	if ( b3IsLeaf( root ) )
	{
		return;
	}

	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b3GetLeftChild( root );

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		for ( int i = 0; i < 2; ++i )
		{
			b3TreeNode* node = nodes + pair + i;
			if ( node->flagIndex & B3_MOVED_NODE )
			{
				node->flagIndex &= ~B3_MOVED_NODE;
				if ( b3IsLeaf( node ) == false )
				{
					if ( stackCount < B3_TREE_STACK_SIZE )
					{
						stack[stackCount++] = b3GetLeftChild( node );
					}
					else
					{
						// Bad stuff will happen if the moved flags don't get cleared.
						B3_ASSERT( stackCount < B3_TREE_STACK_SIZE );
					}
				}
			}
		}
	}
}

int b3DynamicTree_GatherMovedProxies( const b3DynamicTree* tree, int* proxyIds )
{
	const b3TreeNode* nodes = tree->nodes;
	const b3TreeNode* root = nodes + B3_ROOT_NODE;
	if ( b3IsNodeMoved( root ) == false )
	{
		return 0;
	}

	if ( b3IsLeaf( root ) )
	{
		proxyIds[0] = b3GetProxyId( root );
		return 1;
	}

	int count = 0;
	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b3GetLeftChild( root );

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		for ( int i = 0; i < 2; ++i )
		{
			const b3TreeNode* node = nodes + pair + i;
			if ( b3IsNodeMoved( node ) == false )
			{
				continue;
			}

			if ( b3IsLeaf( node ) )
			{
				proxyIds[count++] = b3GetProxyId( node );
			}
			else
			{
				if ( stackCount < B3_TREE_STACK_SIZE )
				{
					stack[stackCount++] = b3GetLeftChild( node );
				}
				else
				{
					B3_ASSERT( stackCount < B3_TREE_STACK_SIZE );
				}
			}
		}
	}

	return count;
}

// Slow refit for unit tests.
static void b3RefitSubtree( b3TreeNode* nodes, int rootIndex )
{
	b3TreeNode* root = nodes + rootIndex;
	if ( b3IsLeaf( root ) || b3IsNodeMoved( root ) == false )
	{
		return;
	}

	// Negative entries indicate deferred unions. A parent is pushed before its children so the
	// LIFO order guarantees both children are finished when the union runs.
	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;
	// ~0 == -1
	stack[stackCount++] = ~rootIndex;
	stack[stackCount++] = b3GetLeftChild( root );
	stack[stackCount++] = b3GetLeftChild( root ) + 1;

	while ( stackCount > 0 )
	{
		int item = stack[--stackCount];
		if ( item < 0 )
		{
			b3TreeNode* node = nodes + ~item;
			int pair = b3GetLeftChild( node );
			node->aabb = b3UnionV( nodes[pair].aabb, nodes[pair + 1].aabb );
			continue;
		}

		b3TreeNode* node = nodes + item;
		if ( b3IsLeaf( node ) || b3IsNodeMoved( node ) == false )
		{
			continue;
		}

		if ( stackCount + 3 <= B3_TREE_STACK_SIZE )
		{
			int pair = b3GetLeftChild( node );
			stack[stackCount++] = ~item;
			stack[stackCount++] = pair;
			stack[stackCount++] = pair + 1;
		}
		else
		{
			B3_ASSERT( stackCount + 3 <= B3_TREE_STACK_SIZE );
		}
	}
}

void b3DynamicTree_Refit( b3DynamicTree* tree )
{
	if ( b3HasTreeMoved( tree ) == false )
	{
		return;
	}

	b3TreeNode* nodes = tree->nodes;

	if ( tree->dfsOrdered == false )
	{
		// This only happens in unit tests.
		b3RefitSubtree( nodes, B3_ROOT_NODE );
		return;
	}

	for ( int pair = tree->nodeEnd - 2; pair >= 0; pair -= 2 )
	{
		b3TreeNode* node = nodes + pair;
		uint32_t flags1 = node[0].flagIndex;
		uint32_t flags2 = node[1].flagIndex;

		// Did either move?
		if ( ( ( flags1 | flags2 ) & B3_MOVED_NODE ) == 0 )
		{
			continue;
		}

		// Is the node internal and moved?
		bool refit1 = ( flags1 & ( B3_LEAF_NODE | B3_MOVED_NODE ) ) == B3_MOVED_NODE;
		bool refit2 = ( flags2 & ( B3_LEAF_NODE | B3_MOVED_NODE ) ) == B3_MOVED_NODE;

		// If the node needs a refit then get the child index. Otherwise use a valid dummy index.
		// The store function won't actually modify the node at the dummy index. This
		// avoids an unpredictable branch. SIMD FTW.
		int children1 = refit1 ? (int)( flags1 & B3_NODE_INDEX_MASK ) : pair;
		int children2 = refit2 ? (int)( flags2 & B3_NODE_INDEX_MASK ) : pair;

		// Conditionally store the union.
		b3StoreAABBV( &node[0].aabb, b3UnionPairV( nodes + children1 ), refit1 );
		b3StoreAABBV( &node[1].aabb, b3UnionPairV( nodes + children2 ), refit2 );
	}
}

static FILE* b3OpenTreeFile( const char* fileName, const char* mode )
{
	FILE* file = NULL;

#if defined( _MSC_VER )
	errno_t e = fopen_s( &file, fileName, mode );
	if ( e != 0 )
	{
		return NULL;
	}
#else
	file = fopen( fileName, mode );
	if ( file == NULL )
	{
		return NULL;
	}
#endif

	return file;
}

void b3DynamicTree_Save( const b3DynamicTree* tree, const char* fileName )
{
	if ( tree->nodes == NULL || tree->parents == NULL || tree->proxies == NULL )
	{
		return;
	}

	FILE* file = b3OpenTreeFile( fileName, "wb" );
	if ( file == NULL )
	{
		return;
	}

	// Copy to allow setting some fields to zero
	b3DynamicTree temp = *tree;

	// Zero pointers and temp data
	temp.nodes = NULL;
	temp.parents = NULL;
	temp.proxies = NULL;
	temp.swapNodes = NULL;
	temp.leafIndices = NULL;
	temp.leafNodes = NULL;
	temp.leafBoxes = NULL;
	temp.leafCenters = NULL;
	temp.binIndices = NULL;
	temp.rebuildCapacity = 0;

	temp.nodeCapacity = tree->nodeEnd;

	fwrite( &temp, sizeof( b3DynamicTree ), 1, file );

	if ( tree->nodeEnd > 0 )
	{
		fwrite( tree->nodes, sizeof( b3TreeNode ), tree->nodeEnd, file );
		fwrite( tree->parents, sizeof( int32_t ), tree->nodeEnd, file );
	}

	if ( tree->proxyCapacity > 0 )
	{
		fwrite( tree->proxies, sizeof( b3TreeProxy ), tree->proxyCapacity, file );
	}

	fclose( file );
}

b3DynamicTree b3DynamicTree_Load( const char* fileName, float scale )
{
	b3DynamicTree tree = { 0 };

	FILE* file = b3OpenTreeFile( fileName, "rb" );
	if ( file == NULL )
	{
		return tree;
	}

	int readCount = (int)fread( &tree, sizeof( b3DynamicTree ), 1, file );
	if ( readCount != 1 )
	{
		fclose( file );
		memset( &tree, 0, sizeof( b3DynamicTree ) );
		return tree;
	}

	if ( tree.version != B3_DYNAMIC_TREE_VERSION )
	{
		fclose( file );
		memset( &tree, 0, sizeof( b3DynamicTree ) );
		return tree;
	}

	tree.nodes = NULL;
	tree.parents = NULL;
	tree.proxies = NULL;
	tree.swapNodes = NULL;
	tree.leafIndices = NULL;
	tree.leafNodes = NULL;
	tree.leafBoxes = NULL;
	tree.leafCenters = NULL;
	tree.binIndices = NULL;
	tree.rebuildCapacity = 0;

	bool ok = tree.nodeCapacity >= 2 && tree.proxyCapacity >= 1 && ( tree.nodeEnd & 1 ) == 0 && tree.nodeEnd >= 2 &&
			  tree.nodeEnd <= tree.nodeCapacity;

	if ( ok )
	{
		tree.nodes = (b3TreeNode*)b3Alloc( tree.nodeCapacity * sizeof( b3TreeNode ) );
		tree.parents = (int32_t*)b3Alloc( tree.nodeCapacity * sizeof( int32_t ) );
		tree.proxies = (b3TreeProxy*)b3Alloc( tree.proxyCapacity * sizeof( b3TreeProxy ) );

		ok = (int)fread( tree.nodes, sizeof( b3TreeNode ), tree.nodeCapacity, file ) == tree.nodeCapacity;
		ok = ok && (int)fread( tree.parents, sizeof( int32_t ), tree.nodeCapacity, file ) == tree.nodeCapacity;
		ok = ok && (int)fread( tree.proxies, sizeof( b3TreeProxy ), tree.proxyCapacity, file ) == tree.proxyCapacity;
	}

	if ( ok == false )
	{
		b3Free( tree.nodes, tree.nodeCapacity * sizeof( b3TreeNode ) );
		b3Free( tree.parents, tree.nodeCapacity * sizeof( int32_t ) );
		b3Free( tree.proxies, tree.proxyCapacity * sizeof( b3TreeProxy ) );
		fclose( file );
		memset( &tree, 0, sizeof( b3DynamicTree ) );
		return tree;
	}

	// An empty node has infinite bounds, so the scale has to skip it.
	for ( int i = 0; i < tree.nodeEnd; ++i )
	{
		b3TreeNode* node = tree.nodes + i;
		if ( b3IsEmptyNode( node ) )
		{
			continue;
		}

		node->aabb.lowerBound = b3MulSV( scale, node->aabb.lowerBound );
		node->aabb.upperBound = b3MulSV( scale, node->aabb.upperBound );
	}

	fclose( file );

	return tree;
}
