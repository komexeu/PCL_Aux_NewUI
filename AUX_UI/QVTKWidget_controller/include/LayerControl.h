#pragma once

#include "../../common_data.h"
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//stl
#include <vector>
//qt
#include <qfont.h>
#include <qstring.h>
#include <qstandarditemmodel.h>

using namespace pcl;

Q_DECLARE_METATYPE(pcl::PointCloud<PointXYZRGB>::Ptr)


//---------�ϼh������----------
class LayerController
{
public:
	LayerController() {};
private:

};

class TreeLayerController {
public:
	TreeLayerController(QStandardItemModel* model) : data_model_() {
		data_model_ = model;
	}
	bool AddLayer(QString layerName, PointCloud<PointXYZRGB>::Ptr cloud);
	virtual bool AddLayer(QString layerName, PointCloud<PointXYZRGB>::Ptr cloud, QModelIndex selectId);
private:
	QStandardItemModel* data_model_;
};

