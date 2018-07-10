/// <summary>
/// Realsenseから点群データを取得するためのインタフェースクラス
/// </summary>
#pragma once
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>

// 名前空間
//---------------------------------
namespace PCLWrapper
{

/// <summary>
/// Realsenseから点群データを取得するためのインタフェースクラス
/// </summary>
class PWIRSStreamer
{

// 列挙子
//---------------------------------
public:
	static enum StreamMode { Depth = 0, Color = 1, DepthAndColor = 2 };

public:
	virtual void Start() = 0;
	virtual void SetStreamingMode( StreamMode ) = 0;
	virtual void Update( pcl::PointCloud<pcl::PointXYZ>::Ptr & ) = 0;
	virtual void Update( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, bool ) = 0;

};


} // PCLWrapper