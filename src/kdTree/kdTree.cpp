#include "kdTree.h"

/************************************************************************/
void KdTree::InsertInternal( Node** root,
							 int depth,
							 XYZI_POINT point,
							 int index )
	/************************************************************************/
{
	if( NULL == ( *root ) )
	{
		*root = new Node( point, index ) ;
	}
	else
	{
		int curDm = depth % 3 ;

		if( point[ curDm ] < ( *root )->point[ curDm ] )
		{
			InsertInternal( &( ( *root )->left ), depth + 1, point, index ) ;
		}
		else
		{
			InsertInternal( &( ( *root )->right ), depth + 1, point, index ) ;
		}
	}
}
/************************************************************************/
void KdTree::SearchInternal( XYZI_POINT target,
							 Node* curNode,
							 int depth,
							 float distanceTol,
							 POINT_INDEX_LIST& indexList )
/************************************************************************/
{
	if( NULL != curNode )
	{
		if( ( curNode->point[ X ] >= ( target[ X ] - distanceTol ) && curNode->point[ X ] <= ( target[ X ] + distanceTol ) ) &&
			( curNode->point[ Y ] >= ( target[ Y ] - distanceTol ) && curNode->point[ Y ] <= ( target[ Y ] + distanceTol ) ) && 
			( curNode->point[ Z ] >= ( target[ Z ] - distanceTol ) && curNode->point[ Z ] <= ( target[ Z ] + distanceTol ) ) )
		{
			float distance = sqrt(
				( ( curNode->point[ X ] - target[ X ] ) * ( curNode->point[ X ] - target[ X ] ) ) +
				( ( curNode->point[ Y ] - target[ Y ] ) * ( curNode->point[ Y ] - target[ Y ] ) ) +
				( ( curNode->point[ Z ] - target[ X ] ) * ( curNode->point[ Z ] - target[ Z ] ) )
			) ;

			if( distance <= distanceTol )
			{
				indexList.push_back( curNode->pointIndex ) ;
			}
		}

		//Base on target boundary
		if( target[ depth % 3 ] < ( curNode->point[ depth % 3 ] + distanceTol ) )
		{
			SearchInternal( target, curNode->left, depth + 1, distanceTol, indexList ) ;
		}

		if( target[ depth % 3 ] > ( curNode->point[ depth % 3 ] - distanceTol ) )
		{
			SearchInternal( target, curNode->right, depth + 1, distanceTol, indexList ) ;
		}
	}
}

/************************************************************************/
void KdTree::Insert( std::vector<float> point, int pointIndex )
/************************************************************************/
{
	InsertInternal( &root, 0, point, pointIndex ) ;
}

/************************************************************************/
POINT_INDEX_LIST KdTree::Search( XYZI_POINT target, float distanceTol )
/************************************************************************/
{
	std::vector<int> ret ;

	SearchInternal( target, root, 0, distanceTol, ret ) ;

	return ret ;
}

