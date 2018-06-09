#pragma once

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace PCLFunctions
{
	/// <summary>
	/// �|�C���g�N���E�h��\�����邽�߂̃E�B���h�E���쐬����
	/// </summary>
	/// <param name="cloud"> �\������|�C���g�N���E�h </param>
	/// <returns> �r�W���A���C�U��Ԃ� </returns>
	template <typename PointT>
	std::shared_ptr<pcl::visualization::PCLVisualizer> CreatePointCloudViewer(
		const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
		const std::string &id,
		int viewport = 0
	)
	{
		// Open 3D viewer and add point cloud
		// 3D�r���[���[���J���|�C���g�N���E�h��ǉ�
		std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer( "PCL Viewer" )
		);

		viewer->setBackgroundColor( 0.251, 0.251, 0.251 );
		viewer->addPointCloud<PointT>( cloud, id );
		viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id
		);
		viewer->addCoordinateSystem( 1.0 );
		viewer->initCameraParameters( );
		viewer->setShowFPS( false );

		return( viewer );
	}


	// �T���v���_�Q�̐���
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr GetSamplePointCloudPtr( );

	// ���ʌ��o
	pcl::PointIndices::Ptr PlaneDetect( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold );
	pcl::PointIndices::Ptr PlaneDetect( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double threshold );

}