#include "LayerControl.h"
//this is for new pointcloud
bool TreeLayerController::AddLayer(QString layerName, PointCloud<PointXYZRGB>::Ptr cloud) {
	QIcon icon;
	icon.addFile(QString::fromUtf8("./my_source/cursor1-2.png"), QSize(), QIcon::Normal, QIcon::Off);

	QStandardItem* qitem = new QStandardItem(layerName);
	QVariant itemCloud;
	itemCloud.setValue(cloud);
	qitem->setData(itemCloud);
	qitem->setIcon(icon);
	data_model_->appendRow(qitem);
	return(true);
}

bool TreeLayerController::AddLayer(QString layerName, PointCloud<PointXYZRGB>::Ptr cloud, QModelIndex selectId) {
	QIcon icon;
	icon.addFile(QString::fromUtf8("./my_source/cursor1-2.png"), QSize(), QIcon::Normal, QIcon::Off);

	QStandardItem* qitem = new QStandardItem(layerName);
	QVariant itemCloud;
	itemCloud.setValue(cloud);
	qitem->setData(itemCloud);
	qitem->setIcon(icon);

	data_model_->itemFromIndex(selectId)->appendRow(qitem);
	return(true);
}
//--------tree for struct data-----------
bool TreeLayerController::AddLayer(QString layerName, complax_cloudInformation complaxInform, QModelIndex selectId) {
	QIcon icon;
	icon.addFile(QString::fromUtf8("./my_source/cursor1-2.png"), QSize(), QIcon::Normal, QIcon::Off);

	QStandardItem* qitem = new QStandardItem(layerName);
	QVariant itemCloud;
	itemCloud.setValue(complaxInform);
	qitem->setData(itemCloud);
	qitem->setIcon(icon);

	data_model_->itemFromIndex(selectId)->appendRow(qitem);
	return(true);
}