/// <summary>
/// Realsense����_�Q�f�[�^���擾���邽�߂̃C���^�t�F�[�X�N���X
/// </summary>
#pragma once
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>

// ���O���
//---------------------------------
namespace PCLWrapper
{

/// <summary>
/// Realsense����_�Q�f�[�^���擾���邽�߂̃C���^�t�F�[�X�N���X
/// </summary>
class PWIRSStreamer
{

// �񋓎q
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