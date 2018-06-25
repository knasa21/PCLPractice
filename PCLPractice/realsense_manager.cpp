#include "realsense_manager.h"
#include <iostream>

namespace PCLFunctions {

	//===================================================================
	// provides context info and will exit if no devices
	//===================================================================
	int PrintRSContextInfo( rs::context *c )
	{
		printf( "There are %d connected RealSense devices.\n", c->get_device_count( ) );
		if( c->get_device_count( ) == 0 )
			throw std::runtime_error( "No device detected. Is it plugged in?" );

		return EXIT_SUCCESS;
	}



	//===================================================================
	// stream config & enabling for device
	//===================================================================
	int ConfigureRSStreams( rs::device *rsCamera )
	{
		std::cout << "Configuring RS streaming " << rsCamera->get_name( ) << "... ";

		//rsCamera->enable_stream( rs::stream::depth, rs::preset::best_quality );
		//rsCamera->enable_stream( rs::stream::color, rs::preset::best_quality );
		rsCamera->enable_stream( rs::stream::depth, rs::preset::highest_framerate );
		rsCamera->enable_stream( rs::stream::color, rs::preset::highest_framerate );
		rsCamera->start( );

		std::cout << "RS streaming enabled and running.\n";

		return EXIT_SUCCESS;
	}


	/// <summary>
	/// ポイントクラウドデータの作成
	/// </summary>
	/// <param name="dev">入力のRealSence device</param>
	/// <param name="rsCloudPtr">ポイントクラウドデータ</param>
	/// <returns>終了状態</returns>
	int GenerateRSPointCloud( rs::device *dev, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rsCloudPtr )
	{

		//新しいデータを待つ
		if( dev->is_streaming( ) )
			dev->wait_for_frames( );

		//画像用変数
		const uint16_t * depthImage		= ( const uint16_t * )dev->get_frame_data( rs::stream::depth );
		const uint8_t  * colorImage	= ( const uint8_t  * )dev->get_frame_data( rs::stream::color );

		//デプスとカラーの合成用
		rs::intrinsics depthIntrin		= dev->get_stream_intrinsics( rs::stream::depth );
		rs::extrinsics depthToColor		= dev->get_extrinsics( rs::stream::depth, rs::stream::color );
		rs::intrinsics colorIntrin		= dev->get_stream_intrinsics( rs::stream::color );
		float scale						= dev->get_depth_scale( );

		//デプスの各サイズ
		int dw	= 0;
		int dh	= 0;
		int dwh	= 0;

		dw = depthIntrin.width;
		dh = depthIntrin.height;

		dwh = dw * dh;

		//遠距離データの削除
		const int noisy	= 3.0;

		//Cloudデータの初期化
		rsCloudPtr->clear( );
		rsCloudPtr->is_dense = false;
		rsCloudPtr->resize( dwh );

		// データの入れるためのループ
		// 列
		for( int dy = 0; dy < dh; dy++ ) {

			//行
			for( int dx = 0; dx < dw; dx++ ) {
				unsigned int i = dy * dw + dx;
				uint16_t depthValue = depthImage[ i ];

				if( depthValue == 0 )
					continue;

				rs::float2 depthPixel ={ ( float )dx, ( float )dy };
				float depthInMeters	  = depthValue * scale;

				rs::float3 depthPoint = depthIntrin.deproject( depthPixel, depthInMeters );
				rs::float3 colorPoint = depthToColor.transform( depthPoint );
				rs::float2 colorPixel = colorIntrin.project( colorPoint );

				const int cx = ( int )std::round( colorPixel.x );
				const int cy = ( int )std::round( colorPixel.y );

				static const float nan = std::numeric_limits<float>::quiet_NaN( );

				//不要な点の削除
				bool depthFail = true;
				bool colorFail = true;

				depthFail = ( depthPoint.z > noisy );
				colorFail = ( cx < 0 || cy < 0 || cx > colorIntrin.width || cy > colorIntrin.height );

				// ==== クラウドポインタの設定 ====

				// depthデータ
				float *dpX;
				float *dpY;
				float *dpZ;

				dpX = &( rsCloudPtr->points[ i ].x );
				dpY = &( rsCloudPtr->points[ i ].y );
				dpZ = &( rsCloudPtr->points[ i ].z );

				// colorデータ
				uint8_t *cpR;
				uint8_t *cpG;
				uint8_t *cpB;

				cpR = &( rsCloudPtr->points[ i ].r );
				cpG = &( rsCloudPtr->points[ i ].g );
				cpB = &( rsCloudPtr->points[ i ].b );

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

				// カラーポイントデータのセット
				const uint8_t *offset = ( colorImage + ( cy * colorIntrin.width + cx ) * 3 );

				uint8_t rawR		= 0;
				uint8_t rawG		= 0;
				uint8_t rawB		= 0;
				uint8_t adjustedR	= 0;
				uint8_t adjustedG	= 0;
				uint8_t adjustedB	= 0;

				rawR = *( offset );
				rawG = *( offset + 1 );
				rawB = *( offset + 2 );

				// 色の位置調整
				adjustedR = rawR;
				adjustedG = rawG;
				adjustedB = rawB;

				// ==== Cloudポイントの評価 ====
				// badポイントは削除orスキップ
				if( depthFail || colorFail )
				{
					*dpX = *dpY = *dpZ = ( float )nan;
					*cpR = *cpG = *cpB = 0;
					continue;
				}
				// クラウドに追加
				else {
					// Fill in cloud depth
					*dpX = adjustedX;
					*dpY = adjustedY;
					*dpZ = adjustedZ;

					// Fill in cloud color
					*cpR = adjustedR;
					*cpG = adjustedG;
					*cpB = adjustedB;

				}
			}
		}

		return EXIT_SUCCESS;
	}

	int GenerateRSPointCloud( rs::device *dev, pcl::PointCloud<pcl::PointXYZ>::Ptr rsCloudPtr )
	{

		//新しいデータを待つ
		if( dev->is_streaming( ) )
			dev->wait_for_frames( );

		//画像用変数
		const uint16_t * depthImage		= ( const uint16_t * )dev->get_frame_data( rs::stream::depth );

		//デプスとカラーの合成用
		rs::intrinsics depthIntrin		= dev->get_stream_intrinsics( rs::stream::depth );
		float scale						= dev->get_depth_scale( );

		//デプスの各サイズ
		int dw	= 0;
		int dh	= 0;
		int dwh	= 0;

		dw = depthIntrin.width;
		dh = depthIntrin.height;

		dwh = dw * dh;

		//遠距離データの削除
		const float noisy	= 1.5;

		//Cloudデータの初期化
		rsCloudPtr->clear( );
		//rsCloudPtr.reset( new pcl::PointCloud<pcl::PointXYZ>( dw, dh ));
		rsCloudPtr->is_dense = false;
		rsCloudPtr->resize( dwh );

		// データの入れるためのループ
		// 列
		for( int dy = 0; dy < dh; ++dy ) {

			//行
			for( int dx = 0; dx < dw; ++dx ) {
				// インデックス取得
				unsigned int i = dy * dw + dx;

				// 16ビットの深度データ
				uint16_t depthValue = depthImage[ i ];

				if( depthValue == 0 )
					continue;

				// 現実データにマップするためのスケール
				float depthInMeters	  = depthValue * scale;

				// デプスデータを現実の距離データに変換
				rs::float2 depthPixel ={ ( float )dx, ( float )dy };
				rs::float3 depthPoint = depthIntrin.deproject( depthPixel, depthInMeters );

				static const float nan = std::numeric_limits<float>::quiet_NaN( );

				//不要な点の削除
				bool depthFail = true;

				depthFail = ( depthPoint.z > noisy );

				// ==== クラウドポインタの設定 ====

				// depthデータ
				float *dpX;
				float *dpY;
				float *dpZ;

				dpX = &( rsCloudPtr->points[ i ].x );
				dpY = &( rsCloudPtr->points[ i ].y );
				dpZ = &( rsCloudPtr->points[ i ].z );

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
					*dpX = *dpY = *dpZ = ( float )nan;
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

		return EXIT_SUCCESS;
	}
}
