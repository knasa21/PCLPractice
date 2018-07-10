/// <summary>
/// librealsense1.12.1を使用してRealsenseから点群データを取得する
/// </summary>
#pragma once
#include "PWIRSStreamer.h"
#include <librealsense/rs.hpp>

// 名前空間
//---------------------------------
namespace PCLWrapper
{

class PWCRSStreamerLib1121 : public PWIRSStreamer
{
public:

	PWCRSStreamerLib1121( int );
	~PWCRSStreamerLib1121();

	void Start();
	void SetStreamingMode( StreamMode );
	void Update( pcl::PointCloud<pcl::PointXYZ>::Ptr & );
	void Update( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, bool );
	

private:
	void PrintContextInfo();
	void GetDevice( int );
	void EnableStreame( rs::stream, rs::preset );
	void GeneratePointCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr & );

private:
	std::unique_ptr<rs::context> ctx;
	rs::device *dev;
	

};


} // PCLWrapper