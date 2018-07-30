/// <summary>
/// librealsense2を使用してRealsenseから点群データを取得する
/// </summary>
#pragma once
#include "PWIRSStreamer.h"
#include <librealsense2/rs.hpp>

// 名前空間
//---------------------------------
namespace PCLWrapper
{

class PWCRSStreamerLib2 : public PWIRSStreamer
{
public:

	PWCRSStreamerLib2( int );
	~PWCRSStreamerLib2();

	void Start();
	void SetStreamingMode( StreamMode );
	void Update( pcl::PointCloud<pcl::PointXYZ>::Ptr & );
	void Update( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, bool );


private:

	bool isEnable();

	void PrintContextInfo();
	void GetDevice( int );
	void EnableStreame();
	void GeneratePointCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr & );
	void GeneratePointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr & );

private:
	rs2::context ctx;
	rs2::device  dev;

	rs2::pipeline pipe;
	rs2::config cfg;
	rs2::pointcloud pc;
	rs2::points points;

	const int CAPACITY;
	rs2::frame_queue queue;

};


} // PCLWrapper