#pragma once

#include <librealsense/rs.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>

namespace PCLFunctions
{
	// �f�o�C�X�̐ڑ��m�F������ΏI��
	int PrintRSContextInfo( rs::context *c );

	//RealSense�L���v�`���J�n
	int ConfigureRSStreams( rs::device *rsCamera );

	// �|�C���g�N���E�h�f�[�^�̍쐬
	int GenerateRSPointCloud( rs::device *dev, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rsCloudPtr );
	int GenerateRSPointCloud( rs::device *dev, pcl::PointCloud<pcl::PointXYZ>::Ptr rsCloudPtr );

}