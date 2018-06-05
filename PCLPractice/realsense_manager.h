#pragma once

#include <librealsense/rs.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>

namespace PCLFunctions
{
	// デバイスの接続確認無ければ終了
	int PrintRSContextInfo( rs::context *c );

	//RealSenseキャプチャ開始
	int ConfigureRSStreams( rs::device *rsCamera );

	// ポイントクラウドデータの作成
	int GenerateRSPointCloud( rs::device *dev, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rsCloudPtr );
	int GenerateRSPointCloud( rs::device *dev, pcl::PointCloud<pcl::PointXYZ>::Ptr rsCloudPtr );

}