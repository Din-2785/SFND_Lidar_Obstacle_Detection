#ifndef KDTREE_H_
#define KDTREE_H_

#include <vector>

/************************************************************************/
//Type Def
/************************************************************************/
using XYZI_POINT = std::vector<float> ;
using POINT_INDEX_LIST = std::vector<int> ;

struct Node
{
	Node* left ;
	Node* right ;

	int pointIndex ;
	
	XYZI_POINT point ;

	Node( XYZI_POINT coordinate, int index ) :
		point( coordinate ),
		pointIndex( index ),
		left( NULL ),
		right( NULL )
	{}
} ;

enum XYZ_element
{
	X = 0,
	Y = 1,
	Z = 2
};

/************************************************************************/
/************************************************************************/

class KdTree
{
public:
	KdTree() : root( NULL ) {}
	~KdTree() {}

	void Insert( std::vector<float> point, int pointIndex ) ;

	POINT_INDEX_LIST Search( XYZI_POINT target, float distanceTol ) ;


private:
	void InsertInternal( Node** root,
						 int depth,
						 XYZI_POINT point,
						 int index ) ;

	void SearchInternal( XYZI_POINT target,
						 Node* curNode,
						 int depth,
						 float distanceTol,
						 POINT_INDEX_LIST& indexList ) ;

	Node* root ;
} ;
#endif
