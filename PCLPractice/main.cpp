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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataCloudPtr( new pcl::PointCloud<pcl::PointXYZRGB> );

	// �_�Q�f�[�^�ǂݍ��� 
	//pcl::io::loadPCDFile<pcl::PointXYZRGB>( "..\\data\\robot\\raw_0.pcd", *dataCloudPtr );
	pcl::io::loadPCDFile<pcl::PointXYZRGB>( "realsense_color.pcd", *dataCloudPtr );

	//Realsense�_�Q
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rsCloudPtr( new pcl::PointCloud<pcl::PointXYZRGB> );

	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	viewer = PCLFunctions::CreatePointCloudViewer<pcl::PointXYZRGB>( dataCloudPtr, "cloud_viewer" );
	//viewer = PCLFunctions::CreatePointCloudViewer<pcl::PointXYZRGB>( rsCloudPtr, "cloud_viewer" );
	// s�L�[���������Ƃ��Ƀf�[�^��ۑ�����
	viewer->registerKeyboardCallback( [ & ]( const pcl::visualization::KeyboardEvent &event ) { 
		if( event.getKeySym( ) == "s" && event.keyDown( ) ) {
			pcl::io::savePCDFileBinary( "realsense_color.pcd", *rsCloudPtr );
		}
	} );

	// ====================
	// ���C�����[�v
	// ====================
	while( !viewer->wasStopped( ) )
	{
		// ===== �f�[�^�̎擾 =====
		int err = PCLFunctions::GenerateRSPointCloud( dev, rsCloudPtr );
		assert( err == 0 );

		//PCLFunctions::PlaneDetect( rsCloudPtr, 0.1 );

		// ===== �f�[�^�̍X�V =====
		//viewer->updatePointCloud( rsCloudPtr, "cloud_viewer" );

		viewer->spinOnce( 1 );
	}

	return 0;
}