#pragma once

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace PCLFunctions
{
	/// <summary>
	/// ポイントクラウドを表示するためのウィンドウを作成する
	/// </summary>
	/// <param name="cloud"> 表示するポイントクラウド </param>
	/// <returns> ビジュアライザを返す </returns>
	template <typename PointT>
	std::shared_ptr<pcl::visualization::PCLVisualizer> CreatePointCloudViewer(
		const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
		const std::string &id,
		int viewport = 0
	)
	{
		// Open 3D viewer and add point cloud
		// 3Dビューワーを開きポイントクラウドを追加
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


	// サンプル点群の生成
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr GetSamplePointCloudPtr( );

	// 平面検出
	pcl::PointIndices::Ptr PlaneDetect( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold );
	pcl::PointIndices::Ptr PlaneDetect( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double threshold );

}