/// <summary>
/// librealsense1.12.1を使用してRealsenseから点群データを取得する
/// </summary>
#pragma once
#include "PWCRSStreamerLib1121.h"

#include <iostream>


// 名前空間
//----------------------------------
namespace PCLWrapper
{


// 公開関数
//----------------------------------

/// <summary>
/// コンストラクタ
/// </summary>
/// <param name="devId"> デバイスID( default = 0 ) </param>
PWCRSStreamerLib1121::PWCRSStreamerLib1121( int devId = 0 )
{
	// contextの作成
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
/// ストリーミング開始
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
/// PointCloudの更新
/// </summary>
/// <param name="cloud"></param>
void PWCRSStreamerLib1121::Update( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud )
{
	if( !isEnable() ) return;

	GeneratePointCloud( cloud );
}

/// <summary>
/// PointCloudの更新
/// </summary>
/// <param name="cloud"></param>
void PWCRSStreamerLib1121::Update( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, bool isColor )
{
	if( !isEnable() ) return;
}

// 非公開関数
//----------------------------------

/// <summary>
/// 動作可能かを返す
/// </summary>
bool PWCRSStreamerLib1121::isEnable()
{
	static bool enable = ( ctx->get_device_count() > 0 );
	return enable;
}

/// <summary>
/// Realsenseデバイスの取得
/// </summary>
/// <param name="id"> デバイスID( default = 0 ) </param>
void PWCRSStreamerLib1121::GetDevice( int devId = 0 )
{
	dev = ctx->get_device( devId );
}

/// <summary>
/// rs::Contextの情報を出力
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
/// 設定を行う
/// </summary>
void PWCRSStreamerLib1121::EnableStreame( rs::stream stream, rs::preset preset )
{
	if( !isEnable() ) return;

	std::cout << "Configuring RS streaming " << dev->get_name() << "...";

	dev->enable_stream( stream, preset );
}

/// <summary>
/// Realsenseデータからポイントクラウドの生成
/// </summary>
/// <param name="cloud"></param>
void PWCRSStreamerLib1121::GeneratePointCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud )
{
	//新しいデータを待つ
	if( dev->is_streaming() )
		dev->wait_for_frames();

	//画像用変数
	const uint16_t *depthImage = (const uint16_t *)dev->get_frame_data( rs::stream::depth );

	static const rs::intrinsics depthIntrin = dev->get_stream_intrinsics( rs::stream::depth );

	static const float scale = dev->get_depth_scale();

	//デプスの各サイズ
	static const int dw	= depthIntrin.width;
	static const int dh	= depthIntrin.height;
	static const int dwh	= dw * dh;

	//遠距離データの削除
	const float noisy	= 3.0;

	//Cloudデータの初期化
	cloud->clear();
	//cloud->is_dense = false;
	cloud->resize( dwh );

	// データの入れるためのループ
	// 列
	for( int dy = 0; dy < dh; ++dy ) {

		//行
		for( int dx = 0; dx < dw; ++dx ) {
			// インデックス取得
			unsigned int i = dy * dw + dx;

			// 16ビットの深度データ
			uint16_t depthValue = depthImage[i];

			if( depthValue == 0 )
				continue;

			// 現実データにマップするためのスケール
			float depthInMeters	  = depthValue * scale;

			// デプスデータを現実の距離データに変換
			rs::float2 depthPixel ={ (float)dx, (float)dy };
			rs::float3 depthPoint = depthIntrin.deproject( depthPixel, depthInMeters );

			static const float nan = std::numeric_limits<float>::quiet_NaN();

			//不要な点の削除
			bool depthFail = true;

			depthFail = ( depthPoint.z > noisy );

			// ==== クラウドポインタの設定 ====

			// depthデータ
			float *dpX;
			float *dpY;
			float *dpZ;

			dpX = &( cloud->points[i].x );
			dpY = &( cloud->points[i].y );
			dpZ = &( cloud->points[i].z );

			// ==== クラウドデータ ====
			// デプスデータのセットアップ
			float realX			= 0;
			float realY			= 0;
			float realZ			= 0;
			float adjustedX		= 0;
			float adjustedY		= 0;
			float adjustedZ		= 0;

			realX = depthPoint.x;
			realY = depthPoint.y;
			realZ = depthPoint.z;

			// 座標の調整
			adjustedX = -1 * realX;
			adjustedY = -1 * realY;
			adjustedZ = realZ;

			// ==== Cloudポイントの評価 ====
			// badポイントは削除orスキップ
			if( depthFail )
			{
				*dpX = *dpY = *dpZ = (float)nan;
				continue;
			}
			// クラウドに追加
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