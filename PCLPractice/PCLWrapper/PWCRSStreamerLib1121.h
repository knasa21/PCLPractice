/// <summary>
/// librealsense1.12.1���g�p����Realsense����_�Q�f�[�^���擾����
/// </summary>
#pragma once
#include "PWIRSStreamer.h"
#include <librealsense/rs.hpp>

// ���O���
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