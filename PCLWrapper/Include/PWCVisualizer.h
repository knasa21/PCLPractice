#pragma once

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

// 名前空間
//---------------------------------
namespace PCLWrapper
{

/// <summary>
/// PCLVisualizerラッパークラス
/// </summary>
template <typename PointT>
class PWCVisualizer
{
public:
	PWCVisualizer( const std::string, const std::string );
	~PWCVisualizer();

	//template <typename PointT>
	void Create( const typename pcl::PointCloud<PointT>::ConstPtr & );

	//template <typename PointT>
	void UpdatePointCloud( const typename pcl::PointCloud<PointT>::ConstPtr & );

	bool WasStopped();

	//	====================
	//	ゲッター
	//	====================

	std::shared_ptr<pcl::visualization::PCLVisualizer> GetViewer();
	std::string GetWindowName();
	std::string GetPointCloudId();

private:
	std::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
	std::string m_windowName;
	std::string m_pointCloudId;

};


//	====================
//	実装
//	====================

template <typename PointT>
PWCVisualizer<typename PointT>::PWCVisualizer( const std::string windowName, const std::string pointCloudId )
	: m_windowName( windowName )
	, m_pointCloudId( pointCloudId )
{
}

template <typename PointT>
PWCVisualizer<typename PointT>::~PWCVisualizer()
{
}


/// <summary>
/// ポイントクラウドを表示するためのウィンドウを作成する
/// </summary>
/// <param name="cloud"> 表示するポイントクラウド </param>
/// <returns> ビジュアライザ </returns>
template <typename PointT>
void PWCVisualizer<typename PointT>::Create( const typename pcl::PointCloud<PointT>::ConstPtr &cloud )
{
	// viewerの作成
	m_viewer = std::make_shared<pcl::visualization::PCLVisualizer>( m_windowName );

	// viewerの設定とポイントクラウドの設定

	m_viewer->addPointCloud<PointT>( cloud, m_pointCloudId );

	m_viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, m_pointCloudId
	);

	m_viewer->setBackgroundColor( 0.251, 0.251, 0.251 );
	m_viewer->addCoordinateSystem( 1.0 );
	m_viewer->initCameraParameters();
	m_viewer->setShowFPS( false );

}

/// <summary>
/// ポイントクラウドの更新
/// </summary>
template <typename PointT>
void PWCVisualizer<typename PointT>::UpdatePointCloud( const typename pcl::PointCloud<PointT>::ConstPtr &cloud )
{
	m_viewer->updatePointCloud( cloud, m_pointCloudId );
}


/// <summary>
///	ビューワーの停止状態を返す
/// </summary>
template <typename PointT>
bool PWCVisualizer<typename PointT>::WasStopped()
{
	return m_viewer->wasStopped();
}


//	====================
//	ゲッター
//	====================

template <typename PointT>
std::shared_ptr<pcl::visualization::PCLVisualizer> PWCVisualizer<typename PointT>::GetViewer()
{
	return m_viewer;
}

template <typename PointT>
std::string PWCVisualizer<typename PointT>::GetWindowName()
{
	return m_windowName;
}

template <typename PointT>
std::string PWCVisualizer<typename PointT>::GetPointCloudId()
{
	return m_pointCloudId;
}



}	// PCLWrapper



