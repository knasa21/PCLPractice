#pragma once

#define BOOST_TYPEOF_EMULATION

#include <vtkAutoInit.h>
VTK_MODULE_INIT( vtkRenderingOpenGL );

#include <chrono>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>

#include <pcl/console/parse.h>

#include "realsense_manager.h"
#include "PCLWrapper/PWCVisualizer.h"
#include "PCLWrapper/PWCRSStreamerLib1121.h"
#include "PCLWrapper/PWCRSStreamerLib2.h"
#include "PCLWrapper/PWUColor.h"

#include <boost/range/algorithm_ext/erase.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

#include <pcl/features/integral_image_normal.h>

#include <pcl/filters/voxel_grid.h>
// bilateral filter
#include <pcl/filters/fast_bilateral_omp.h>
// normal estimation
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
// OMPS
#include <pcl/segmentation/extract_polygonal_prism_data.h>
// OCCS
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

#include <pcl/common/time.h>


using namespace PCLWrapper;

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal	  NormalType;

const std::string fileRoot = "..\\data";

void CheckLength( pcl::PointCloud<pcl::PointXYZRGB>& cloud, double length )
{
	for ( auto it = cloud.begin(); it != cloud.end(); ++it ) {
		/*if( it->y < -0.3 ) {
			it->r = 0;
			it->g = 255;
			it->b = 0;
		}*/
	}
	boost::remove_erase_if( cloud, [=]( pcl::PointXYZRGB p ) { return p.z > length || p.y < -0.3; } );
}

// バイラテラルフィルタ
void BilateralFilter(
	const pcl::PointCloud<PointType>::ConstPtr &srcCloud,
	const float &sigmaR,
	const float &sigmaS,
	pcl::PointCloud<PointType>::Ptr &dstCloud )
{
	//pcl::ScopeTime scopetime( "BilateralFilter" );
	pcl::FastBilateralFilterOMP<PointType> bf;
	bf.setInputCloud( srcCloud );
	bf.setSigmaR( sigmaR );
	bf.setSigmaS( sigmaS );
	bf.applyFilter( *dstCloud );
}

// 法線算出
void ComputeNormals(
	const pcl::PointCloud<PointType>::ConstPtr	&cloud,
	pcl::PointCloud<NormalType>::Ptr			&normals,
	const double &maxDepthChangeFactor,
	const double &smoothingSize
)
{
	/*NormalTypeEstimation<PointType, NormalType> ne;
	ne.setSearchMethod( kdTree );
	ne.setRadiusSearch( radius );
	ne.setInputCloud( cloud );
	ne.compute( *normals );*/

	pcl::IntegralImageNormalEstimation<PointType, NormalType> ne;
	ne.setNormalEstimationMethod( ne.AVERAGE_3D_GRADIENT );
	ne.setMaxDepthChangeFactor( maxDepthChangeFactor );
	ne.setNormalSmoothingSize( smoothingSize );
	ne.setInputCloud( cloud );
	ne.compute( *normals );
}

void DrawSegment(
	pcl::PointCloud<PointType>::Ptr			&cloud,
	const std::vector<pcl::PointIndices>	&inlierIndices,
	const int								&minSize
)
{
	static const int angle = 25;

	for ( int inlierI = 0; inlierI < inlierIndices.size(); ++inlierI )
	{
		int indicesSize = inlierIndices[inlierI].indices.size();
		if ( indicesSize < minSize ) {
			continue;
		}

		for ( int i = 0; i < indicesSize; ++i )
		{
			cloud->points[inlierIndices[inlierI].indices[i]].rgb = *PWUColor::HsvToRgb(
				( angle * inlierI ) % 255 ,
				200,
				200 );
		}
	}
}

// OMPS + OCCS
void segment_objects(
	pcl::PointCloud<PointType>::Ptr		&cloud,
	const pcl::PointCloud<NormalType>::ConstPtr	&normals,
	const int									&minInliers,
	const double								&angularThreshold,
	const double								&distanceThreshold,
	pcl::PointCloud<PointType>::Ptr				&dstCloud
)
{
	// OMPS
	pcl::OrganizedMultiPlaneSegmentation<PointType, NormalType, pcl::Label> mps;
	mps.setMinInliers( minInliers );
	mps.setAngularThreshold( angularThreshold );
	mps.setDistanceThreshold( distanceThreshold );
	mps.setInputNormals( normals );
	mps.setInputCloud( cloud );

	std::vector<pcl::PlanarRegion<PointType>,
		Eigen::aligned_allocator<pcl::PlanarRegion<PointType>>> regions;

	std::vector<pcl::ModelCoefficients> modelCoefficients;
	std::vector<pcl::PointIndices>		inlierIndices;

	pcl::PointCloud<pcl::Label>::Ptr	labels( new pcl::PointCloud<pcl::Label> );
	std::vector<pcl::PointIndices>		labelIndices;
	std::vector<pcl::PointIndices>		boundaryIndices;

	mps.segmentAndRefine(
		regions,				// 生成された平面領域の配列
		modelCoefficients,		// 各平面モデルの係数の配列
		inlierIndices,			// 各平面に対応する点群のインデックス配列
		labels,					// セグメンテーションに対応するラベル
		labelIndices,			// 各ラベルの点群インデックス
		boundaryIndices			// 各ラベルの輪郭に対応する点群インデックス
	);


	//DrawSegment( cloud, inlierIndices, 100 );

	// OCCS
	pcl::PointCloud<pcl::Label> euclideanLabels;
	std::vector<pcl::PointIndices> euclideanLabelIndices;

	//クラスタリングされた平面をboolの配列で持つ
	std::vector<bool> planeLabels;
	planeLabels.resize( labelIndices.size(), false );
	for ( size_t i = 0; i < labelIndices.size(); i++ ) {
		if ( labelIndices[i].indices.size() > minInliers )
		{
			planeLabels[i] = true;
		}
	}

	// ユークリッド距離に基づくクラスタリング
	pcl::EuclideanClusterComparator
		<PointType, NormalType, pcl::Label>::Ptr ecc(
			new pcl::EuclideanClusterComparator<PointType, NormalType, pcl::Label>()
		);

	ecc->setInputCloud( cloud );
	ecc->setLabels( labels );
	ecc->setExcludeLabels( planeLabels );
	ecc->setDistanceThreshold( distanceThreshold, false );

	pcl::PointCloud<PointType>::CloudVectorType clusters;
	pcl::OrganizedConnectedComponentSegmentation
		< PointType, pcl::Label> occs( ecc );
	occs.setInputCloud( cloud );
	occs.segment( euclideanLabels, euclideanLabelIndices );

	DrawSegment( cloud, euclideanLabelIndices, 100 );

	return;

	for ( size_t i = 0; i < euclideanLabelIndices.size(); ++i ) {
		pcl::PointCloud<PointType> cluster;
		pcl::copyPointCloud( *cloud, euclideanLabelIndices[i].indices, cluster );
		clusters.push_back( cluster );
	}

	*dstCloud = clusters[0];
}

int main( int argc, int** argv )
{

	// PointCloud
	pcl::PointCloud<PointType>::Ptr cloud( new pcl::PointCloud<PointType>() );
	pcl::PointCloud<PointType>::Ptr rsCloud( new pcl::PointCloud<PointType>() );

	// Realsense
	//std::unique_ptr<PWIRSStreamer> rsStreamer( new PWCRSStreamerLib1121( 0 ) );
	std::unique_ptr<PWIRSStreamer> rsStreamer( new PWCRSStreamerLib2( 0 ) );
	rsStreamer->SetStreamingMode( PWIRSStreamer::Depth );
	rsStreamer->Start();

	// pcd 読み込み
	//pcl::io::loadPCDFile<PointType>( "..\\data\\table\\table_scene_lms400_downsampled.pcd", *cloud );
	//pcl::io::loadPCDFile<PointType>( "test_pcd.pcd", *rsCloud );
	//pcl::io::loadPCDFile<PointType>( fileRoot + "\\floor\\floor_bin.pcd", *rsCloud );


	// Visualizer
	PWCVisualizer<PointType> visualizer( "window", "rsCloud" );
	visualizer.Create( rsCloud );
	visualizer.GetViewer()->addPointCloud( cloud, "cloud" );


	//visualizer.UpdatePointCloud( cloud );

	double angle = 3.0, distance = 0.03;
	double mDepth = 0.04, sSize = 6.0;
	bool filterOn = false;
	visualizer.GetViewer()->registerKeyboardCallback( [&]( const pcl::visualization::KeyboardEvent &event )
	{
		if ( event.keyDown() ) {
			switch ( event.getKeySym()[0] ) {

			case 'b':
				filterOn = !filterOn;
				break;
			
			case 'y':
				//angle += 0.1;
				mDepth += 0.001;
				break;

			case 'h':
				//angle -= 0.1;
				mDepth -= 0.001;
				break;

			case 'i':
				//distance += 0.01;
				sSize += 1;
				break;

			case 'k':
				//distance -= 0.01;
				sSize -= 1;
				break;
			
			}
			std::cout << " angle = " << angle << ", distance = " << distance << std::endl;
			std::cout << " maxDepth = " << mDepth << ", soomsingSize = " << sSize << std::endl;
		}

		if( event.getKeySym() == "s" && event.keyDown() ) {
			pcl::io::savePCDFileBinary( fileRoot + "\\floor\\floor_bin.pcd", *rsCloud );
			pcl::io::savePCDFileBinaryCompressed( fileRoot + "\\floor\\floor_bincon.pcd", *rsCloud );
		}
	} );


	while ( !visualizer.WasStopped() )
	{
		rsStreamer->Update( rsCloud, false );

		static pcl::PointCloud<PointType>::Ptr		filteredCloud( new pcl::PointCloud<PointType>() );
		static pcl::PointCloud<PointType>::Ptr		segmentedCloud( new pcl::PointCloud<PointType>() );
		static pcl::PointCloud<NormalType>::Ptr	cloudNormals( new pcl::PointCloud<NormalType> );
		static pcl::search::KdTree<PointType>::Ptr	tree( new pcl::search::KdTree<PointType>() );

		if ( filterOn ) {


			BilateralFilter( rsCloud, 0.05, 5.0, filteredCloud );

			//tree->setInputCloud( filteredCloud );
			//ComputeNormals( filteredCloud, cloudNormals, 0.04, 3.0 );
			ComputeNormals( filteredCloud, cloudNormals, mDepth, sSize );

			//visualizer.GetViewer()->removePointCloud( "cloud" );
			//visualizer.GetViewer()->removePointCloud( "normals" );
			//visualizer.GetViewer()->addPointCloudNormals<PointType, NormalType>( filteredCloud, cloudNormals );

			segment_objects( filteredCloud, cloudNormals, 1000, pcl::deg2rad( angle ), distance, segmentedCloud );
			//visualizer.UpdatePointCloud( segmentedCloud );

			visualizer.UpdatePointCloud( filteredCloud );
		}
		else {
			//pcl::VoxelGrid<PointType> sor;
			//sor.setInputCloud( rsCloud );
			//sor.setLeafSize( 0.01f, 0.01f, 0.01f );
			//sor.filter( *rsCloud );
			visualizer.UpdatePointCloud( rsCloud );
		}

		visualizer.GetViewer()->spinOnce();
	}

}