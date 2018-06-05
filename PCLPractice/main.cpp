#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "pcl_functions.h"
#include "realsense_manager.h"

int main( int argc, int** argv )
{

	// Realsence context
	rs::context ctx;
	PCLFunctions::PrintRSContextInfo( &ctx );

	//デバイスの取得
	rs::device * dev = ctx.get_device( 0 );

	PCLFunctions::ConfigureRSStreams( dev );

	//サンプル点群 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr = PCLFunctions::GetSamplePointCloudPtr( );

	//Realsense点群
	pcl::PointCloud<pcl::PointXYZ>::Ptr rsCloudPtr( new pcl::PointCloud<pcl::PointXYZ> );

	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	viewer = PCLFunctions::CreatePointCloudViewer( rsCloudPtr, "cloud_viewer" );

	// ====================
	// メインループ
	// ====================
	while( !viewer->wasStopped( ) )
	{
		// ===== データの取得 =====
		int err = PCLFunctions::GenerateRSPointCloud( dev, rsCloudPtr );
		assert( err == 0 );

		// ===== データの更新 =====
		viewer->updatePointCloud( rsCloudPtr, "cloud_viewer" );

		viewer->spinOnce( 1 );
	}

	return 0;
}