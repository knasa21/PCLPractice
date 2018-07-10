#pragma once
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <pcl/console/parse.h>

#include "pcl_functions.h"
#include "realsense_manager.h"
#include "PCLWrapper/PWCVisualizer.h"
#include "PCLWrapper/PWCRSStreamerLib1121.h"

#include <boost/range/algorithm_ext/erase.hpp>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/features/integral_image_normal.h>

using namespace PCLWrapper;

typedef pcl::PointXYZ PointType;

void CheckLength( pcl::PointCloud<pcl::PointXYZRGB>& cloud, double length )
{
	for( auto it = cloud.begin(); it != cloud.end(); ++it ) {
		/*if( it->y < -0.3 ) {
			it->r = 0;
			it->g = 255;
			it->b = 0;
		}*/
	}
	boost::remove_erase_if( cloud, [=]( pcl::PointXYZRGB p ) { return p.z > length || p.y < -0.3; } );
}

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );
pcl::PointIndices::Ptr inliers( new pcl::PointIndices );

//平面検出関数
void planeDetect( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold )
{
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients( true );
	//必須
	seg.setInputCloud( cloud );
	seg.setModelType( pcl::SACMODEL_PLANE );//モデル
	seg.setMethodType( pcl::SAC_RANSAC );//検出手法
	seg.setDistanceThreshold( threshold ); //閾値 0.5とか
	seg.segment( *inliers, *coefficients );
}

//平面除去関数
void planeRemoval( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative )
{
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	extract.setInputCloud( cloud );
	extract.setIndices( inliers );
	extract.setNegative( negative );// true にすると平面を除去、false にすると平面以外を除去
	extract.filter( *cloud );
}


int main( int argc, int** argv )
{

	// PointCloud
	pcl::PointCloud<PointType>::Ptr cloud( new pcl::PointCloud<PointType>() );
	pcl::PointCloud<PointType>::Ptr rsCloud( new pcl::PointCloud<PointType>() );
	
	// Realsense
	std::unique_ptr<PWIRSStreamer> rsStreamer( new PWCRSStreamerLib1121( 0 ) );
	rsStreamer->SetStreamingMode( PWIRSStreamer::Depth );
	rsStreamer->Start();

	// pcd 読み込み
	pcl::io::loadPCDFile<PointType>( "..\\data\\table\\table_scene_lms400_downsampled.pcd", *cloud );

	// Visualizer
	PWCVisualizer<PointType> visualizer( "window", "rsCloud" );
	visualizer.Create( rsCloud );
	visualizer.GetViewer()->addPointCloud( cloud, "cloud" );

	//planeDetect( cloud, 0.03 );
	//planeRemoval( cloud, inliers, true );

	//visualizer.UpdatePointCloud( cloud );

	while( !visualizer.WasStopped() )
	{
		rsStreamer->Update( rsCloud );

		visualizer.UpdatePointCloud( rsCloud );
		visualizer.GetViewer()->spinOnce();
	}

}