/// <summary>
/// librealsense2���g�p����Realsense����_�Q�f�[�^���擾����
/// </summary>
#pragma once


// �C���N���[�h
//--------------------------------------------------------------------
#include "PWCRSStreamerLib2.h"

#include <pcl/filters/passthrough.h>

#include <iostream>
#include <thread>


// ���O���
//--------------------------------------------------------------------
namespace PCLWrapper
{


// ���J�֐�
//--------------------------------------------------------------------

/// <summary>
/// �R���X�g���N�^
/// </summary>
/// <param name="devId"> �f�o�C�XID( default = 0 ) </param>
PWCRSStreamerLib2::PWCRSStreamerLib2( int devId = 0 )
	: pipe( rs2::pipeline() )
	, cfg( rs2::config() )
	, CAPACITY( 5 )
	, queue( CAPACITY )
{
	// Get a snapshot of currently conneccted devices
	auto list = ctx.query_devices();
	if( list.size() == 0 )
		throw std::runtime_error( "No device detected." );
	dev = list.front();
}

PWCRSStreamerLib2::~PWCRSStreamerLib2()
{
}


/// <summary>
/// �X�g���[�~���O�J�n
/// </summary>
void PWCRSStreamerLib2::Start()
{
	pipe.start(cfg);

}


void PWCRSStreamerLib2::SetStreamingMode( StreamMode mode )
{
	cfg.disable_all_streams();

	switch( mode )
	{

		case PCLWrapper::PWIRSStreamer::Depth:
			cfg.enable_stream( RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90 );
			break;

		case PCLWrapper::PWIRSStreamer::Color:
			cfg.enable_stream( RS2_STREAM_COLOR );
			break;

		case PCLWrapper::PWIRSStreamer::DepthAndColor:
			cfg.enable_stream( RS2_STREAM_COLOR );
			cfg.enable_stream( RS2_STREAM_DEPTH );
			break;

		default:
			break;
	}
}

/// <summary>
/// PointCloud�̍X�V
/// </summary>
/// <param name="cloud"></param>
void PWCRSStreamerLib2::Update( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud )
{
	if( !isEnable() ) return;

	rs2::frameset frames = pipe.wait_for_frames();
	rs2::depth_frame depth = frames.get_depth_frame();

	points = pc.calculate( depth );

	GeneratePointCloud( cloud );
}

/// <summary>
/// PointCloud�̍X�V
/// </summary>
/// <param name="cloud"></param>
void PWCRSStreamerLib2::Update( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, bool isColor )
{
	if( !isEnable() ) return;

	rs2::frameset frames = pipe.wait_for_frames();
	rs2::depth_frame depth = frames.get_depth_frame();

	points = pc.calculate( depth );

	GeneratePointCloud( cloud );

}

// ����J�֐�
//--------------------------------------------------------------------

/// <summary>
/// ����\����Ԃ�
/// </summary>
bool PWCRSStreamerLib2::isEnable()
{
	static bool enable = true;
	return enable;
}

/// <summary>
/// Realsense�f�o�C�X�̎擾
/// </summary>
/// <param name="id"> �f�o�C�XID( default = 0 ) </param>
void PWCRSStreamerLib2::GetDevice( int devId = 0 )
{
}

/// <summary>
/// rs::Context�̏����o��
/// </summary>
void PWCRSStreamerLib2::PrintContextInfo()
{
	std::cout
		<< "There are "
		<< " connected Realsense devices."
		<< std::endl;

}

/// <summary>
/// �ݒ���s��
/// </summary>
void PWCRSStreamerLib2::EnableStreame()
{
	if( !isEnable() ) return;

	std::cout << "Configuring RS streaming " << "...";

}

/// <summary>
/// Realsense�f�[�^����|�C���g�N���E�h�̐���
/// </summary>
/// <param name="cloud"></param>
void PWCRSStreamerLib2::GeneratePointCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud )
{
	const static auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->clear();

	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize( points.size() );
	const rs2::vertex *ptr = points.get_vertices();
	for( auto &p : cloud->points )
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud( cloud );
	//pass.setFilterFieldName( "x" );
	//pass.setFilterLimits( -0.3, 0.3 );
	//pass.filter( *cloud );
	//pass.setFilterFieldName( "y" );
	//pass.setFilterLimits( -0.3, 0.5 );
	//pass.filter( *cloud );
	//pass.setFilterFieldName( "z" );
	//pass.setFilterLimits( 0.5, 1.2 );
	//pass.filter( *cloud );

}


/// <summary>
/// Realsense�f�[�^����|�C���g�N���E�h�̐���
/// </summary>
void PWCRSStreamerLib2::GeneratePointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud )
{
	const static auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->clear();

	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize( points.size() );
	const rs2::vertex *ptr = points.get_vertices();
	for( auto &p : cloud->points )
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}
}

} // PCLWrapper