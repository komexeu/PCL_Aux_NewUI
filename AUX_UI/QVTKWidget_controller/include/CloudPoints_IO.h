#pragma once

//qt
#include <qstring.h>
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

#include <qdebug>
template <typename PointType>
class CloudPoints_IO
{
public:
	CloudPoints_IO() {};

	//點雲匯入
	bool CloudImport(QString filename) {
		QFileInfo qfi(filename);
		if (qfi.suffix() == "csv") {
			file_name_.push_back(qfi.fileName());
			if (!CloudPoints_IO::csv2pointCloud(qfi.filePath()))
				return(false);
		}
		else if (qfi.suffix() == "pcd") {
			PointCloud<PointType>::Ptr cloud(new PointCloud<PointType>);
			int importReault = io::loadPCDFile<PointType>
				(qfi.filePath().toLocal8Bit().data(), *cloud);
			if (importReault == 0) {
				file_name_.push_back(qfi.fileName());
				VoxelGrid<PointType> vox;
				vox.setInputCloud(cloud);
				vox.setLeafSize(0.001f, 0.001f, 0.001f);
				vox.filter(*cloud);
				import_cloud_.push_back(*cloud);
			}
		}
		else
			return(false);

		return(true);
	}
	//點雲匯出
	bool CloudExport(std::vector<PointCloud<PointType>> childern_cloudData) {
		QString output_root = QFileDialog::getSaveFileName(nullptr, QObject::tr("Select a root."),
			"C:/", QObject::tr("All file(*.pcd)"));
		if (output_root.isEmpty())
			return(false);

		PointCloud<PointXYZRGBObjectID>::Ptr myObj(new PointCloud < PointXYZRGBObjectID>);
		for (int i = 0; i < childern_cloudData.size(); i++)
		{
			PointCloud<PointType> tmpcld = childern_cloudData[i];

			for (int j = 0; j < tmpcld.size(); j++)
			{
				PointXYZRGBObjectID tmp;
				tmp.x = tmpcld.points[j].x;
				tmp.y = tmpcld.points[j].y;
				tmp.z = tmpcld.points[j].z;
				tmp.rgb = tmpcld.points[j].rgb;
				tmp.obj_id = i;
				myObj->push_back(tmp);
			}
		}

		QString pcdFileRoot = output_root;
		io::savePCDFile(pcdFileRoot.toStdString(), *myObj);
		return(true);
	}

	//匯入檔案轉換後點雲
	std::vector<PointCloud<PointType>> import_cloud_;
	QStringList file_name_;
protected:

private:
	bool csv2pointCloud(QString filepath) {
		QFile csvfile(filepath);
		PointCloud<PointType>::Ptr cloud(new PointCloud < PointType>);
		if (csvfile.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			QTextStream infile(&csvfile);
			int Line_i = 0;

			while (!infile.atEnd()) {
				QString lineStr = infile.readLine();
				Line_i++;

				QStringList res = lineStr.split(",");//分割","
				if (res.size() == 1)
				{
					res = lineStr.split(" ");
				}
				if (res.size() == 1)
				{
					res = lineStr.split("/t");
				}

				if (Line_i == 1) {
					if ((res[0] == "x" || res[0] == "X") &&
						(res[1] == "y" || res[1] == "Y") &&
						(res[2] == "z" || res[2] == "Z"))
					{
						continue;
					}
					else if (res.size() >= 3 && res.size() <= 6)
					{
						continue;
					}
					else
					{
						return(false);
					}
				}

				PointType tmp;
				if (res.size() == 3)
				{
					tmp.x = atof(res[0].toStdString().c_str());
					tmp.y = atof(res[1].toStdString().c_str());
					tmp.z = atof(res[2].toStdString().c_str());
					tmp.r = 255;
					tmp.g = 255;
					tmp.b = 255;
				}
				else if (res.size() == 6)
				{
					tmp.x = atof(res[0].toStdString().c_str());
					tmp.y = atof(res[1].toStdString().c_str());
					tmp.z = atof(res[2].toStdString().c_str());
					tmp.r = atof(res[3].toStdString().c_str());
					tmp.g = atof(res[4].toStdString().c_str());
					tmp.b = atof(res[5].toStdString().c_str());
				}
				else
					break;
				cloud->push_back(tmp);
			}
			csvfile.close();

			VoxelGrid<PointType> vox;
			vox.setInputCloud(cloud);
			vox.setLeafSize(0.001f, 0.001f, 0.001f);
			vox.filter(*cloud);

			import_cloud_.push_back(*cloud);

			if (Line_i <= 1)
				return (false);
			else
				return(true);
		}
	}
};