#pragma once

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace PCLFunctions
{
	// �|�C���g�N���E�h��\�����邽�߂̃E�B���h�E���쐬����
	std::shared_ptr< pcl::visualization::PCLVisualizer > CreatePointCloudViewer(
		const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
		const std::string &id,
		int viewport = 0
	);

	std::shared_ptr< pcl::visualization::PCLVisualizer > CreatePointCloudViewer(
		const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
		const std::string &id,
		int viewport = 0
	);


	pcl::PointCloud< pcl::PointXYZRGB >::Ptr GetSamplePointCloudPtr( );

}