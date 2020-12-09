#include "object_work.h"

//--------IO pointcloud-------
#include <qinputdialog.h>
#include "QVTKWidget_controller/include/CloudPoints_IO.h"
#include "QVTKWidget_controller/include/CloudPoints_Tools.h"
#include <qfiledialog.h>
#include <fstream>
#include <typeinfo>
//-----tool----
#include "QVTKWidget_controller/include/LayerControl.h"

#include <QtConcurrent/qtconcurrentrun.h>

void object_work::SetBrushMode() {
	qt_data.brush_sliderAction->setVisible(true);
	qt_data.brush_spinBoxAction->setVisible(true);

	my_ui.brush_spinbox->setValue(general_data.brush_radius);

	QIcon the_icon;
	the_icon.addFile("./my_source/cursor1-2.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::BRUSH_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(0);
}
void object_work::SetAreaMode() {
	qt_data.brush_sliderAction->setVisible(false);
	qt_data.brush_spinBoxAction->setVisible(false);

	QIcon the_icon;
	the_icon.addFile("./my_source/AreaSelect.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::AREA_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(1);
}
void object_work::SetNoneMode() {
	qt_data.brush_sliderAction->setVisible(false);
	qt_data.brush_spinBoxAction->setVisible(false);

	QIcon the_icon;
	the_icon.addFile("./my_source/NonMode.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(0);
}

QModelIndex object_work::searchParent(QModelIndex index) {
	if (index.parent().row() == -1) {
		return index;
	}
	else {
		QModelIndex parentItem = index.parent();
		while (parentItem.parent().row() != -1)
			parentItem = parentItem.parent();

		return parentItem;
	}
}

void object_work::ViewCloudUpdate(PointCloud<PointXYZRGB>::Ptr updateCloud, bool resetCamera) {
	pcl_data.viewer->updatePointCloud(updateCloud, "cld");
	if (resetCamera)
		pcl_data.viewer->resetCamera();
	ui.qvtkWidget->update();
}
void object_work::RedSelectClear() {
	map_redSelected->clear();
	general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();
}
void object_work::reset_point_color() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	PointCloud<PointXYZRGB>::Ptr cld(new PointCloud<PointXYZRGB>);
	cld = qt_data.standardModel->itemFromIndex(index)->data().
		value<PointCloud<PointXYZRGB>::Ptr>()->makeShared();

	ViewCloudUpdate(cld, false);
	RedSelectClear();
}
#include <pcl/kdtree/kdtree_flann.h>
void object_work::Tree_selectionChangedSlot(const QItemSelection&, const QItemSelection&) {
	RedSelectClear();
	general_data.SegClouds.clear();

	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	if (index.row() == -1) {
		PointCloud<PointXYZRGB>::Ptr nullCloud(new PointCloud<PointXYZRGB>);
		ViewCloudUpdate(nullCloud, true);
		return;
	}

	int size = qt_data.standardModel->itemFromIndex(index)->data().value<PointCloud<PointXYZRGB>::Ptr>()->size();
	general_data.nowLayerCloud = qt_data.standardModel->itemFromIndex(index)->data().value<PointCloud<PointXYZRGB>::Ptr>();

	general_data.Selected_cloud->clear();
	general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();

	QModelIndex TopParent = searchParent(index);
	PointCloud<PointXYZRGB>::Ptr TopCloud(new PointCloud<PointXYZRGB>);
	TopCloud = qt_data.standardModel->itemFromIndex(TopParent)->data().value<PointCloud<PointXYZRGB>::Ptr>()->makeShared();
	ViewCloudUpdate(TopCloud, true);
	ViewCloudUpdate(general_data.nowLayerCloud, false);

	QString selectedText = QString::fromStdString(std::to_string(size)) + " points.";
	my_ui.message->setText(selectedText);

	//取1000點做平均取距離
	std::vector<int> k_indices;
	std::vector<float> k_sqr_distances;
	int n = 0;
	double norm = 0;
	int searched_points = 0;
	pcl::KdTreeFLANN<PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<PointXYZRGB>);
	tree->setInputCloud(general_data.nowLayerCloud);

	for (int i = 0; i < (general_data.nowLayerCloud->size() >= 1000 ? 1000 : general_data.nowLayerCloud->size()); ++i)
	{
		n = tree->nearestKSearch(i, 2, k_indices, k_sqr_distances);
		if (n == 2)
		{
			double n = sqrt(k_sqr_distances[1]);
			if (n < VTK_DOUBLE_MIN || n>VTK_DOUBLE_MAX)
				continue;
			norm += n;
			++searched_points;
		}
	}

	if (searched_points != 0) {
		general_data.nowCloud_avg_distance = norm / searched_points;
	}
	else {
		general_data.nowCloud_avg_distance = 0;
	}
}

void object_work::ImportCloud() {
	if (tree_layerController == NULL)
		tree_layerController = new TreeLayerController(qt_data.standardModel);
	RedSelectClear();

	QFileDialog add_dialog;
	add_dialog.setFileMode(QFileDialog::ExistingFiles);
	QStringList filelist = add_dialog.getOpenFileNames(nullptr, QObject::tr("Select a root."),
		"C:/",
		QObject::tr("All file(*.*);;pcd file (*.pcd);;csv file(*.csv)"));
	if (filelist.isEmpty())
	{
		ui.treeView->selectionModel()->clear();
		return;
	}

	for (int i = 0; i < filelist.size(); i++)
	{
		CloudPoints_IO<PointXYZRGB> IO_Tool;
		if (!IO_Tool.CloudImport(filelist[i])) {
			QString selectedText = "Import fail.";
			my_ui.message->setText(selectedText);
			return;
		}

		for (int i = 0; i < IO_Tool.file_name_.size(); i++)
		{
			bool ok;
			QString text = QInputDialog::getText(NULL, tr("QInputDialog::getText()"),
				tr("Layer name:"), QLineEdit::Normal,
				IO_Tool.file_name_[i], &ok);
			if (ok && !text.isEmpty()) {
				std::string BaseLayerName = text.toStdString();
				std::string objName = "NONE" + text.toStdString();
				if (!tree_layerController->AddLayer(text, IO_Tool.import_cloud_[i].makeShared()))
					return;
				QString selectedText = "Import success.";
				my_ui.message->setText(selectedText);
			}
		}
	}
	ui.treeView->selectionModel()->clear();
}
void object_work::ExportCloud() {
	if (tree_layerController == NULL)
		tree_layerController = new TreeLayerController(qt_data.standardModel);
	auto indexes = ui.treeView->selectionModel()->selectedIndexes();
	if (indexes.size() <= 0)
	{
		my_ui.message->setText("No layer selected.");
		return;
	}

	CloudPoints_IO<PointXYZRGB> IO_Tool;
	vector<PointCloud<PointXYZRGB>> children_cloud_data;
	vector<QModelIndex> parentIndexes;
	for (int i = 0; i < indexes.size(); ++i)
	{
		QModelIndex parent_index = searchParent(indexes[i]);
		vector<QModelIndex>::iterator it_result =
			std::find(parentIndexes.begin(), parentIndexes.end(), parent_index);
		if (it_result != parentIndexes.end())
			continue;
		parentIndexes.push_back(parent_index);
		for (int j = 0; j < qt_data.standardModel->itemFromIndex(parent_index)->rowCount(); ++j)
		{
			children_cloud_data.push_back(*qt_data.standardModel->itemFromIndex(parent_index)->child(j)
				->data().value<PointCloud<PointXYZRGB>::Ptr>());
		}
		if (IO_Tool.CloudExport(children_cloud_data))
			my_ui.message->setText("Export success.");
		else
			my_ui.message->setText("Export fail.");
	}
}

void object_work::deleteLayer() {
	QModelIndexList indexes = ui.treeView->selectionModel()->selectedIndexes();
	RedSelectClear();
	if (indexes.size() <= 0)
		return;
	qSort(indexes.begin(), indexes.end(), qGreater<QModelIndex>());
	vector<QModelIndex> parent_index;
	for (int i = 0; i < indexes.size(); ++i) {
		if (qt_data.standardModel->itemFromIndex(indexes[i])->parent() == NULL) {
			parent_index.push_back(indexes[i]);
		}
		else {
			qt_data.standardModel->itemFromIndex(indexes[i])->parent()->removeRow(indexes[i].row());
		}
	}
	for (int i = 0; i < parent_index.size(); ++i)
		qt_data.standardModel->removeRow(parent_index[i].row());

	ui.treeView->selectionModel()->clearCurrentIndex();
	my_ui.message->setText("");
	PointCloud<PointXYZRGB>::Ptr null(new PointCloud<PointXYZRGB>);
	ViewCloudUpdate(null, false);
}

void object_work::onCustomContextMenu(const QPoint& point)
{
	QModelIndex index = ui.treeView->indexAt(point);
	QMenu menu;
	menu.addAction(QStringLiteral("merge"), this, SLOT(mergeLayer()));
	menu.addAction(QStringLiteral("delete"), this, SLOT(deleteLayer()));
	menu.addSeparator();
	menu.exec(ui.treeView->viewport()->mapToGlobal(point));
}
void object_work::mergeLayer() {
	QModelIndexList indexes = ui.treeView->selectionModel()->selectedIndexes();
	if (indexes.size() <= 0)
		return;
	for (int i = 0; i < indexes.size(); ++i)
		if (qt_data.standardModel->itemFromIndex(indexes[i])->parent() == NULL)
			return;

	qSort(indexes.begin(), indexes.end(), qGreater<QModelIndex>());
	PointCloud<PointXYZRGB>::Ptr mergedCloud(new PointCloud<PointXYZRGB>);
	for (int i = 0; i < indexes.size(); ++i)
		*mergedCloud += *qt_data.standardModel->itemFromIndex(indexes[i])->data().value<PointCloud<PointXYZRGB>::Ptr>();

	if (!tree_layerController->AddLayer("merge_layer", mergedCloud, searchParent(ui.treeView->selectionModel()->currentIndex().parent())))
		return;
	for (int i = 0; i < indexes.size(); ++i)
		qt_data.standardModel->itemFromIndex(indexes[i])->parent()->removeRow(indexes[i].row());

	ui.treeView->selectionModel()->clearCurrentIndex();
	my_ui.message->setText("");
	PointCloud<PointXYZRGB>::Ptr nullcloud(new PointCloud<PointXYZRGB>);
	ViewCloudUpdate(nullcloud, false);
}

void object_work::voxelFilter() {
	general_data.Voxel_cloud->clear();
	if (GLOBAL_SELECTMODE != SelectMode::NO_SELECT_MODE)
		SetNoneMode();
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	PointCloud<PointXYZRGB>::Ptr cld(new PointCloud<PointXYZRGB>);
	cld = qt_data.standardModel->itemFromIndex(index)->data().
		value<PointCloud<PointXYZRGB>::Ptr>();

	PointCloud<PointXYZRGB>::Ptr voxel_cld(new PointCloud<PointXYZRGB>);
	CloudPoints_Tools tools;
	voxel_cld = tools.CloudDensity(cld, my_ui.leaf_spinbox->value(), general_data.nowCloud_avg_distance)->makeShared();
	general_data.Voxel_cloud = voxel_cld->makeShared();

	ViewCloudUpdate(voxel_cld, false);
	RedSelectClear();
	my_ui.message->setText(QString::fromStdString(std::to_string(voxel_cld->size())));
}
void object_work::VoxelWork() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	if (general_data.Voxel_cloud->size() <= 0)
		return;

	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	QVariant itemCloud;
	itemCloud.setValue(general_data.Voxel_cloud->makeShared());
	qt_data.standardModel->itemFromIndex(index)->setData(itemCloud);
	general_data.Voxel_cloud->clear();

	my_ui.message->setText("Filter Finish.");
}

void object_work::Tree_Smooth() {
	if (smoothing)
	{
		my_ui.message->setText("You have to wait until smooth done.");
		return;
	}
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	RedSelectClear();
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	PointCloud<PointXYZRGB>::Ptr cld = qt_data.standardModel->itemFromIndex(index)->data().value<PointCloud<PointXYZRGB>::Ptr>();

	CycleProgress* c_progress = new CycleProgress(QString("Smoothing"));
	c_progress->show();
	c_progress->Start(10, 100);
	QtConcurrent::run([=]() {
		smoothing = true;
		CloudPoints_Tools cpTools;
		//30為搜尋範圍，*0.5搜尋半徑
		PointCloud<PointXYZRGB>::Ptr smooth_cld = cpTools.CloudSmooth(cld, general_data.nowCloud_avg_distance * my_ui.smooth_spinbox->value() * 0.5);

		if (smooth_cld->size() > 0)
		{
			//data update
			QVariant itemCloud;
			itemCloud.setValue(smooth_cld);
			qt_data.standardModel->itemFromIndex(index)->setData(itemCloud);

			general_data.nowLayerCloud = smooth_cld;
			general_data.Selected_cloud->clear();
			general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();
			//view update
			ViewCloudUpdate(smooth_cld, false);
			my_ui.message->setText("Smooth finish.");
		}
		else
		{
			my_ui.message->setText("NO DATA AFTER SMOOTH,Please set a bigger value.");
		}
		ui.treeView->selectionModel()->clear();
		c_progress->Stop();
		smoothing = false;
	});
}
//segment
void object_work::Slider_PreSegCloud() {
	if (preseg_working)
		return;

	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	general_data.SegClouds.clear();
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();

	QtConcurrent::run([=]() {
		preseg_working = true;
		PointCloud<PointXYZRGB>::Ptr database_cloud(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cld(new PointCloud<PointXYZRGB>);
		copyPointCloud(*general_data.nowLayerCloud, *database_cloud);
		copyPointCloud(*general_data.nowLayerCloud, *cld);

		CloudPoints_Tools cpTools;
		std::vector<PointIndices> seg_cloud_2;
		if (GLOBAL_SEGMENTMODE == SegmentMode::EUCLIDEAN_CLUSTER_EXTRACTION)
			seg_cloud_2 = cpTools.CloudSegmentation(cld, my_ui.preSeg_spinbox->value(), general_data.nowCloud_avg_distance);
		else if (GLOBAL_SEGMENTMODE == SegmentMode::REGION_GROWING)
			seg_cloud_2 = cpTools.CloudSegmentation_regionGrowing(cld, my_ui.preSeg_spinbox->value(), general_data.nowCloud_avg_distance);

		for (int i = 0; i < cld->size(); i++)
		{
			cld->points[i].r = 255;
			cld->points[i].g = 255;
			cld->points[i].b = 255;
		}
		for (vector<PointIndices>::const_iterator i = seg_cloud_2.begin(); i < seg_cloud_2.end(); i++)
		{
			int color_R = rand() % 250;
			int color_G = rand() % 250;
			int color_B = rand() % 250;
			PointCloud<PointXYZRGB>::Ptr tmp(new PointCloud<PointXYZRGB>);
			for (std::vector<int>::const_iterator j = i->indices.begin(); j < i->indices.end(); j++)
			{
				tmp->push_back(database_cloud->points[*j]);
				cld->points[*j].r = color_R;
				cld->points[*j].g = color_G;
				cld->points[*j].b = color_B;
			}
			general_data.SegClouds.push_back(tmp);
		}
		ViewCloudUpdate(cld, false);
		RedSelectClear();
		preseg_working = false;
	});
}
void object_work::confirm_colors_segment() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	if (general_data.SegClouds.size() == 0)
		return;

	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	for (int i = 0; i < general_data.SegClouds.size(); ++i)
	{
		QString segLayer = QString::fromStdString(std::to_string(i));
		if (!tree_layerController->AddLayer(segLayer, general_data.SegClouds[i], searchParent(index)))
			return;
	}
	/*if (index.parent().row() != -1)
		Tree_deleteLayer();*/

	QString children_message = general_data.SegClouds.size() <= 1 ?
		QString::fromStdString("Segment " + std::to_string(general_data.SegClouds.size()) + " child") :
		QString::fromStdString("Segment " + std::to_string(general_data.SegClouds.size()) + " children");
	my_ui.message->setText(children_message);
	general_data.SegClouds.clear();

	ui.treeView->selectionModel()->clear();
}

void object_work::Set_lightRange(const QColor& c) {
	general_data.rgb_data = QColor{ c.red(), c.green(), c.blue() };
	Color_PreSegment();
}
void object_work::Color_PreSegment() {
	int dark_color_h = (general_data.rgb_data.hue() - my_ui.H_range_spinbox->value()) < 0 ?
		(general_data.rgb_data.hue() - my_ui.H_range_spinbox->value()) + 360 :
		general_data.rgb_data.hue() - my_ui.H_range_spinbox->value();
	int light_color_h = (general_data.rgb_data.hue() + my_ui.H_range_spinbox->value()) >= 360 ?
		(general_data.rgb_data.hue() + my_ui.H_range_spinbox->value()) % 360 :
		general_data.rgb_data.hue() + my_ui.H_range_spinbox->value();

	int dark_color_v = (general_data.rgb_data.value() - my_ui.V_range_spinbox->value()) <= 0 ?
		0 : general_data.rgb_data.value() - my_ui.V_range_spinbox->value();
	int light_color_v = (general_data.rgb_data.value() + my_ui.V_range_spinbox->value()) >= 255 ?
		255 : general_data.rgb_data.value() + my_ui.V_range_spinbox->value();

	QColor rgb_dark_data;
	rgb_dark_data.setHsv(dark_color_h, general_data.rgb_data.saturation(), dark_color_v);
	QColor rgb_light_data;
	rgb_light_data.setHsv(light_color_h, general_data.rgb_data.saturation(), light_color_v);

	my_ui.color_widget->setStyleSheet(QString("background-color:"
		"qlineargradient("
		"spread:"
		"pad, x1:0, y1:0.5,x2:1, y2:0.5,"
		"stop:0 rgb(%1, %2, %3),"
		"stop:0.5 rgb(%4, %5, %6),"
		"stop:1 rgb(%7, %8, %9));")
		.arg(
			QString::number(rgb_dark_data.red()),
			QString::number(rgb_dark_data.green()),
			QString::number(rgb_dark_data.blue()),
			QString::number(general_data.rgb_data.red()),
			QString::number(general_data.rgb_data.green()),
			QString::number(general_data.rgb_data.blue()),
			QString::number(rgb_light_data.red()),
			QString::number(rgb_light_data.green()),
			QString::number(rgb_light_data.blue())));
	//-----------
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	general_data.SegClouds.clear();
	CloudPoints_Tools cpTools;
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();

	PointCloud<PointXYZRGB>::Ptr database_cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cld(new PointCloud<PointXYZRGB>);
	copyPointCloud(*general_data.nowLayerCloud, *database_cloud);
	copyPointCloud(*general_data.nowLayerCloud, *cld);

	std::vector<PointIndices> seg_cloud_2;
	seg_cloud_2 = cpTools.CloudSegmentation_RGB(cld,
		general_data.rgb_data, my_ui.H_range_spinbox->value(), my_ui.V_range_spinbox->value());
	for (int i = 0; i < cld->size(); i++)
	{
		cld->points[i].r = 255;
		cld->points[i].g = 255;
		cld->points[i].b = 255;
	}
	for (vector<PointIndices>::const_iterator i = seg_cloud_2.begin(); i < seg_cloud_2.end(); i++)
	{
		PointCloud<PointXYZRGB>::Ptr tmp(new PointCloud<PointXYZRGB>);
		for (std::vector<int>::const_iterator j = i->indices.begin(); j < i->indices.end(); j++)
		{
			tmp->push_back(database_cloud->points[*j]);
			cld->points[*j].r = general_data.rgb_data.red();
			cld->points[*j].g = general_data.rgb_data.green();
			cld->points[*j].b = general_data.rgb_data.blue();
		}
		general_data.SegClouds.push_back(tmp);
	}
	ViewCloudUpdate(cld, false);
	RedSelectClear();
}

//USER segment
void object_work::Tree_UserSegmentation() {
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	if (index.row() == -1)
		return;
	qDebug() << map_redSelected->size();
	if (map_redSelected->size() > 0)
	{
		bool ok;
		QString text = QInputDialog::getText(NULL, tr("QInputDialog::getText()"),
			tr("Layer name:"), QLineEdit::Normal,
			QDir::home().dirName(), &ok);
		if (ok && !text.isEmpty())
		{
			PointCloud<PointXYZRGB>::Ptr newCloud(new PointCloud<PointXYZRGB>);
			PointCloud<PointXYZRGB>::Ptr newCloud2(new PointCloud<PointXYZRGB>);

			for (int i = 0; i < general_data.nowLayerCloud->size(); ++i)
			{
				if (map_redSelected->find(i) != map_redSelected->end())
					newCloud->push_back(general_data.nowLayerCloud->points.at(i));
				else
					newCloud2->push_back(general_data.nowLayerCloud->points.at(i));
			}

			//改為全部只有一層子類
			if (!tree_layerController->AddLayer(text, newCloud->makeShared(), searchParent(index)))
				return;
			if (newCloud2->size() > 0)
			{
				if (!tree_layerController->AddLayer("base", newCloud2->makeShared(), searchParent(index)))
					return;
			}

			if (index.parent().row() != -1)
				deleteLayer();

			ui.treeView->selectionModel()->clear();
			RedSelectClear();
		}
	}
}