#pragma once
#include "common_data.h"
class object_action
{
	General_DataClass::brush_radius;
	//void Tree_Smooth() {
	//	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
	//		return;
	//	RedSelectClear();
	//	CloudPoints_Tools cpTools;
	//	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	//	PointCloud<PointXYZRGB>::Ptr cld = qt_data.standardModel->itemFromIndex(index)->data().value<PointCloud<PointXYZRGB>::Ptr>();

	//	my_ui.message->setText("working...");
	//	//30¬°·j´M½d³ò¡A*0.5·j´M¥b®|
	//	PointCloud<PointXYZRGB>::Ptr smooth_cld = cpTools.CloudSmooth(cld, general_data.nowCloud_avg_distance * my_ui.smooth_spinbox->value() * 0.5);

	//	if (smooth_cld->size() > 0)
	//	{
	//		//data update
	//		QVariant itemCloud;
	//		itemCloud.setValue(smooth_cld);
	//		qt_data.standardModel->itemFromIndex(index)->setData(itemCloud);

	//		general_data.nowLayerCloud = smooth_cld;
	//		general_data.Selected_cloud->clear();
	//		general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();
	//		//view update
	//		ViewCloudUpdate(smooth_cld, false);
	//	}
	//	else
	//	{
	//		my_ui.message->setText("NO DATA AFTER SMOOTH,Please set a bigger value.");
	//	}

	//	my_ui.message->setText("Finish!");
	//	ui.treeView->selectionModel()->clear();
	//}
	//void Slider_PreSegCloud();
};