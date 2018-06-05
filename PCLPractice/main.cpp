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

	//�f�o�C�X�̎擾
	rs::device * dev = ctx.get_device( 0 );

	PCLFunctions::ConfigureRSStreams( dev );

	//�T���v���_�Q 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr = PCLFunctions::GetSamplePointCloudPtr( );

	//Realsense�_�Q
	pcl::PointCloud<pcl::PointXYZ>::Ptr rsCloudPtr( new pcl::PointCloud<pcl::PointXYZ> );

	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	viewer = PCLFunctions::CreatePointCloudViewer( rsCloudPtr, "cloud_viewer" );

	// ====================
	// ���C�����[�v
	// ====================
	while( !viewer->wasStopped( ) )
	{
		// ===== �f�[�^�̎擾 =====
		int err = PCLFunctions::GenerateRSPointCloud( dev, rsCloudPtr );
		assert( err == 0 );

		// ===== �f�[�^�̍X�V =====
		viewer->updatePointCloud( rsCloudPtr, "cloud_viewer" );

		viewer->spinOnce( 1 );
	}

	return 0;
}