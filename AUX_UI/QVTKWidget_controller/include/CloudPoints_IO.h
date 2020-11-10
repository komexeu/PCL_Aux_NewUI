#pragma once

//qt
#include <qstring.h>
#include <qfiledialog.h>
#include <qlocale.h>
#include <qtextstream.h>
#include <qobject.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;

struct PointXYZRGBObjectID
{
	union
	{
		float coordinate[5];
		struct
		{
			float x;
			float y;
			float z;
		};
	};
	union
	{
		float rgb;
	};
	union
	{
		int obj_id;
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN16;
//自訂義型別
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBObjectID,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, rgb, rgb)
	(int, obj_id, obj_id)
)

class CloudPoints_IO
{
public:
	CloudPoints_IO() : import_cloud_()
	{
		//import_cloud_.reset(new PointCloud<PointXYZRGB>);
	};

	//點雲匯入
	bool CloudImport();
	//點雲匯出
	bool CloudExport(std::vector<PointCloud<PointXYZRGB>::Ptr> childern_cloudData);

	//匯入檔案轉換後點雲
	std::vector< PointCloud<PointXYZRGB>::Ptr> import_cloud_;
	QStringList file_name_;
protected:
	//檔案路徑
	QStringList q_file_path_;
private:
	bool RootSelector();
	bool csv2pointCloud(QString filepath);
};