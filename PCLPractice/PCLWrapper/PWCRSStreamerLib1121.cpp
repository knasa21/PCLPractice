/// <summary>
/// librealsense1.12.1���g�p����Realsense����_�Q�f�[�^���擾����
/// </summary>
#pragma once
#include "PWCRSStreamerLib1121.h"

#include <iostream>


// ���O���
//----------------------------------
namespace PCLWrapper
{


// ���J�֐�
//----------------------------------

/// <summary>
/// �R���X�g���N�^
/// </summary>
/// <param name="devId"> �f�o�C�XID( default = 0 ) </param>
PWCRSStreamerLib1121::PWCRSStreamerLib1121( int devId = 0 )
{
	// context�̍쐬
	ctx.reset( new rs::context() );
	PrintContextInfo();

	if( !isEnable() ) return;

	GetDevice( devId );
}

PWCRSStreamerLib1121::~PWCRSStreamerLib1121()
{
	dev->stop();
}


/// <summary>
/// �X�g���[�~���O�J�n
/// </summary>
void PWCRSStreamerLib1121::Start()
{
	if( !isEnable() ) return;

	dev->start();
}


void PWCRSStreamerLib1121::SetStreamingMode( StreamMode mode )
{
	switch( mode )
	{

		case PCLWrapper::PWIRSStreamer::Depth:
			EnableStreame( rs::stream::depth, rs::preset::best_quality );
			break;

		case PCLWrapper::PWIRSStreamer::Color:
			EnableStreame( rs::stream::color, rs::preset::best_quality );
			break;

		case PCLWrapper::PWIRSStreamer::DepthAndColor:
			EnableStreame( rs::stream::depth, rs::preset::best_quality );
			EnableStreame( rs::stream::color, rs::preset::best_quality );
			break;

		default:
			break;
	}
}

/// <summary>
/// PointCloud�̍X�V
/// </summary>
/// <param name="cloud"></param>
void PWCRSStreamerLib1121::Update( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud )
{
	if( !isEnable() ) return;

	GeneratePointCloud( cloud );
}

/// <summary>
/// PointCloud�̍X�V
/// </summary>
/// <param name="cloud"></param>
void PWCRSStreamerLib1121::Update( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, bool isColor )
{
	if( !isEnable() ) return;
}

// ����J�֐�
//----------------------------------

/// <summary>
/// ����\����Ԃ�
/// </summary>
bool PWCRSStreamerLib1121::isEnable()
{
	static bool enable = ( ctx->get_device_count() > 0 );
	return enable;
}

/// <summary>
/// Realsense�f�o�C�X�̎擾
/// </summary>
/// <param name="id"> �f�o�C�XID( default = 0 ) </param>
void PWCRSStreamerLib1121::GetDevice( int devId = 0 )
{
	dev = ctx->get_device( devId );
}

/// <summary>
/// rs::Context�̏����o��
/// </summary>
void PWCRSStreamerLib1121::PrintContextInfo()
{
	std::cout
		<< "There are "
		<< ctx->get_device_count()
		<< " connected Realsense devices."
		<< std::endl;

	if( ctx->get_device_count() == 0 ) {
		std::cerr << "No device detected." << std::endl;
	}

}

/// <summary>
/// �ݒ���s��
/// </summary>
void PWCRSStreamerLib1121::EnableStreame( rs::stream stream, rs::preset preset )
{
	if( !isEnable() ) return;

	std::cout << "Configuring RS streaming " << dev->get_name() << "...";

	dev->enable_stream( stream, preset );
}

/// <summary>
/// Realsense�f�[�^����|�C���g�N���E�h�̐���
/// </summary>
/// <param name="cloud"></param>
void PWCRSStreamerLib1121::GeneratePointCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud )
{
	//�V�����f�[�^��҂�
	if( dev->is_streaming() )
		dev->wait_for_frames();

	//�摜�p�ϐ�
	const uint16_t *depthImage = (const uint16_t *)dev->get_frame_data( rs::stream::depth );

	static const rs::intrinsics depthIntrin = dev->get_stream_intrinsics( rs::stream::depth );

	static const float scale = dev->get_depth_scale();

	//�f�v�X�̊e�T�C�Y
	static const int dw	= depthIntrin.width;
	static const int dh	= depthIntrin.height;
	static const int dwh	= dw * dh;

	//�������f�[�^�̍폜
	const float noisy	= 3.0;

	//Cloud�f�[�^�̏�����
	cloud->clear();
	//cloud->is_dense = false;
	cloud->resize( dwh );

	// �f�[�^�̓���邽�߂̃��[�v
	// ��
	for( int dy = 0; dy < dh; ++dy ) {

		//�s
		for( int dx = 0; dx < dw; ++dx ) {
			// �C���f�b�N�X�擾
			unsigned int i = dy * dw + dx;

			// 16�r�b�g�̐[�x�f�[�^
			uint16_t depthValue = depthImage[i];

			if( depthValue == 0 )
				continue;

			// �����f�[�^�Ƀ}�b�v���邽�߂̃X�P�[��
			float depthInMeters	  = depthValue * scale;

			// �f�v�X�f�[�^�������̋����f�[�^�ɕϊ�
			rs::float2 depthPixel ={ (float)dx, (float)dy };
			rs::float3 depthPoint = depthIntrin.deproject( depthPixel, depthInMeters );

			static const float nan = std::numeric_limits<float>::quiet_NaN();

			//�s�v�ȓ_�̍폜
			bool depthFail = true;

			depthFail = ( depthPoint.z > noisy );

			// ==== �N���E�h�|�C���^�̐ݒ� ====

			// depth�f�[�^
			float *dpX;
			float *dpY;
			float *dpZ;

			dpX = &( cloud->points[i].x );
			dpY = &( cloud->points[i].y );
			dpZ = &( cloud->points[i].z );

			// ==== �N���E�h�f�[�^ ====
			// �f�v�X�f�[�^�̃Z�b�g�A�b�v
			float realX			= 0;
			float realY			= 0;
			float realZ			= 0;
			float adjustedX		= 0;
			float adjustedY		= 0;
			float adjustedZ		= 0;

			realX = depthPoint.x;
			realY = depthPoint.y;
			realZ = depthPoint.z;

			// ���W�̒���
			adjustedX = -1 * realX;
			adjustedY = -1 * realY;
			adjustedZ = realZ;

			// ==== Cloud�|�C���g�̕]�� ====
			// bad�|�C���g�͍폜or�X�L�b�v
			if( depthFail )
			{
				*dpX = *dpY = *dpZ = (float)nan;
				continue;
			}
			// �N���E�h�ɒǉ�
			else {
				// Fill in cloud depth
				*dpX = adjustedX;
				*dpY = adjustedY;
				*dpZ = adjustedZ;

			}
		}
	}
}

} // PCLWrapper