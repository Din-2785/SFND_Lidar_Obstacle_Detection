// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "render/render.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints( typename pcl::PointCloud<PointT>::Ptr cloud )
{
	std::cout << cloud->points.size() << std::endl ;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud( typename pcl::PointCloud<PointT>::Ptr cloud, 
																			   float filterRes, 
																			   Eigen::Vector4f minPoint,
																			   Eigen::Vector4f maxPoint )
{
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now() ;

	// TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered( new pcl::PointCloud<PointT> ) ;

	pcl::VoxelGrid<PointT> vg ;
	vg.setInputCloud( cloud ) ;
	vg.setLeafSize( filterRes, filterRes, filterRes ) ;
	vg.filter( *cloudFiltered ) ;

	typename pcl::PointCloud<PointT>::Ptr cloudRegion( new pcl::PointCloud<PointT> ) ;

	//region, user will handle with filter.
	pcl::CropBox<PointT> region( true ) ;
	region.setMin( minPoint ) ;
	region.setMax( maxPoint ) ;
	region.setInputCloud( cloudFiltered ) ;
	region.filter( *cloudRegion ) ;

	std::vector<int> indicies ;

	pcl::CropBox<PointT> roof( true ) ;
	roof.setMin( Eigen::Vector4f( -1.5, -1.7, -1, 1 ) ) ;
	roof.setMax( Eigen::Vector4f( 2.6, 1.7, -0.4, 1 ) ) ;
	roof.setInputCloud( cloudFiltered ) ;
	roof.filter( indicies ) ;

	pcl::PointIndices::Ptr inliers{ new pcl::PointIndices } ;

	for( int point : indicies )
	{
		inliers->indices.push_back( point ) ;
	}

	pcl::ExtractIndices<PointT> extract ;
	extract.setInputCloud( cloudRegion ) ;
	extract.setIndices( inliers ) ;
	extract.setNegative( true ) ;
	extract.filter( *cloudRegion ) ;

	auto endTime = std::chrono::steady_clock::now() ;
	auto elapsedTime = std::chrono::duration_cast< std::chrono::milliseconds >( endTime - startTime ) ;
	std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl ;

	return cloudRegion ;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds( pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud )
{
	// TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult( cloud, cloud ) ;
	return segResult ;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane( typename pcl::PointCloud<PointT>::Ptr cloud,
																																  int maxIterations,
																																  float distanceThreshold )
{
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now() ;
	//pcl::PointIndices::Ptr inliers ;

	// TODO:: Fill in this function to find inliers for the cloud.
	std::unordered_set<int> inliers = Ransac3d( cloud, maxIterations, distanceThreshold ) ;

	typename pcl::PointCloud<PointT>::Ptr planeCloud( new pcl::PointCloud<PointT>() ) ;
	typename pcl::PointCloud<PointT>::Ptr obstCloud( new pcl::PointCloud<PointT>() ) ;

	int cloudPointSize = cloud->points.size() ;

	for( int index = 0; index < cloudPointSize; index++ )
	{
		auto point = cloud->points[ index ] ;

		if( 0 < inliers.count( index ) )
		{
			planeCloud->points.push_back( point ) ;
		}
		else
		{
			obstCloud->points.push_back( point ) ;
		}
	}


	auto endTime = std::chrono::steady_clock::now() ;
	auto elapsedTime = std::chrono::duration_cast< std::chrono::milliseconds >( endTime - startTime ) ;
	std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl ;

	//std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds( inliers, cloud ) ;

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = std::make_pair( obstCloud, planeCloud ) ;

	return segResult ;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering( typename pcl::PointCloud<PointT>::Ptr cloud,
																						   float clusterTolerance,
																						   int minSize,
																						   int maxSize )
{

	// Time clustering process
	auto startTime = std::chrono::steady_clock::now() ;

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters ;

	// TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	
	//By using builtin function
	typename pcl::search::KdTree<PointT>::Ptr tree( new pcl::search::KdTree<PointT> ) ;
	tree->setInputCloud( cloud ) ;
	
	std::vector<pcl::PointIndices> cluster_indices ;
	pcl::EuclideanClusterExtraction<PointT> ec ;
	ec.setClusterTolerance( clusterTolerance ) ;
	ec.setMinClusterSize( minSize ) ;
	ec.setMaxClusterSize( maxSize ) ;
	ec.setSearchMethod( tree ) ;
	ec.setInputCloud( cloud ) ;
	ec.extract( cluster_indices ) ;

	for( pcl::PointIndices getIndices : cluster_indices )
	{
		typename pcl::PointCloud<PointT>::Ptr cloudCluster( new pcl::PointCloud<PointT> ) ;

		for( int index : getIndices.indices )
			cloudCluster->points.push_back( cloud->points[ index ] ) ;

		cloudCluster->width = cloudCluster->points.size() ;
		cloudCluster->height = 1 ;
		cloudCluster->is_dense = true ;

		clusters.push_back( cloudCluster ) ;
	}
	
	auto endTime = std::chrono::steady_clock::now() ;
	auto elapsedTime = std::chrono::duration_cast< std::chrono::milliseconds >( endTime - startTime ) ;
	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl ;

	return clusters ;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox( typename pcl::PointCloud<PointT>::Ptr cluster )
{

	// Find bounding box for one of the clusters
	PointT minPoint, maxPoint ;
	pcl::getMinMax3D( *cluster, minPoint, maxPoint ) ;

	Box box ;
	box.x_min = minPoint.x ;
	box.y_min = minPoint.y ;
	box.z_min = minPoint.z ;
	box.x_max = maxPoint.x ;
	box.y_max = maxPoint.y ;
	box.z_max = maxPoint.z ;

	return box ;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd( typename pcl::PointCloud<PointT>::Ptr cloud, std::string file )
{
	pcl::io::savePCDFileASCII( file, *cloud ) ;
	std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl ;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd( std::string file )
{

	typename pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT> ) ;

	if( pcl::io::loadPCDFile<PointT>( file, *cloud ) == -1 ) //* load the file
	{
		PCL_ERROR( "Couldn't read file \n" ) ;
	}
	std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl ;

	return cloud ;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd( std::string dataPath )
{

	std::vector<boost::filesystem::path> paths( boost::filesystem::directory_iterator{ dataPath }, boost::filesystem::directory_iterator{} ) ;

	// sort files in accending order so playback is chronological
	sort( paths.begin(), paths.end() ) ;

	return paths ;

}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3d( typename pcl::PointCloud<PointT>::Ptr cloud,
															  int maxIterations, 
															  float distanceTo )
{
	int sizeOfPointInCloud = cloud->points.size() ;

	std::unordered_set<int> inliersResult ;

	srand( time( NULL ) ) ;

	while( 0 < maxIterations )
	{
		std::unordered_set<int> inliers ;

		while( inliers.size() < 3 )
		{
			inliers.insert( rand() % ( cloud->points.size() ) ) ;
		}

		float x1, y1, z1 ;
		float x2, y2, z2 ;
		float x3, y3, z3 ;

		x1 = y1 = z1 = 0 ;
		x2 = y2 = z2 = 0 ;
		x3 = y3 = z3 = 0 ;

		auto itr = inliers.begin() ;

		x1 = cloud->points[ *itr ].x ;
		y1 = cloud->points[ *itr ].y ;
		z1 = cloud->points[ *itr ].z ;

		itr++ ;

		x2 = cloud->points[ *itr ].x ;
		y2 = cloud->points[ *itr ].y ;
		z2 = cloud->points[ *itr ].z ;

		itr++ ;

		x3 = cloud->points[ *itr ].x ;
		y3 = cloud->points[ *itr ].y ;
		z3 = cloud->points[ *itr ].z ;

		Vect3 v1( ( x2 - x1 ), ( y2 - y1 ), ( z2 - z1 ) ) ;
		Vect3 v2( ( x3 - x1 ), ( y3 - y1 ), ( z3 - z1 ) ) ;

		//The formula is defined by "Cross product"
		Vect3 crossProductV1V2
		(
			( ( y2 - y1 ) * ( z3 - z1 ) - ( z2 - z1 ) * ( y3 - y1 ) ),
			( ( z2 - z1 ) * ( x3 - x1 ) - ( x2 - x1 ) * ( z3 - z1 ) ),
			( ( x2 - x1 ) * ( y3 - y1 ) - ( y2 - y1 ) * ( x3 - x1 ) )
		) ;

		float a = crossProductV1V2.x ;
		float b = crossProductV1V2.y ;
		float c = crossProductV1V2.z ;
		float d = -1 * ( a * x1 + b * y1 + c * z1 ) ;

		for( int index = 0 ; index < sizeOfPointInCloud ; index++ )
		{
			if( inliers.count( index ) > 0 )
			{
				continue ;
			}

			auto point = cloud->points[ index ] ;

			float x = point.x ;
			float y = point.y ;
			float z = point.z ;

			float calculatedDistance =
				abs( ( a * x ) + ( b * y ) + ( c * z ) + d ) / sqrt( ( a * a ) + ( b * b ) + ( c * c ) ) ;

			if( distanceTo >= calculatedDistance )
			{
				inliers.insert( index ) ;
			}
		}

		if( inliers.size() > inliersResult.size() )
		{
			inliersResult = inliers ;
		}

		maxIterations-- ;
	}

	return inliersResult ;
}
