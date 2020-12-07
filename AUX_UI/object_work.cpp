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

QModelIndex searchParent(QModelIndex index) {
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
	//select_map.clear();
	general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();
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

void object_work::voxelFilter() {
	general_data.Voxel_cloud->clear();
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

void object_work::SetBrushMode() {
	qt_data.brush_sliderAction->setVisible(true);
	qt_data.brush_spinBoxAction->setVisible(true);

	my_ui.brush_spinbox->setValue(general_data.brush_radius);

	QIcon the_icon;
	the_icon.addFile("./my_source/cursor1-2.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::BRUSH_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(0);
	//WhiteCursorUpdate(false);
}
void object_work::SetAreaMode() {
	qt_data.brush_sliderAction->setVisible(false);
	qt_data.brush_spinBoxAction->setVisible(false);

	QIcon the_icon;
	the_icon.addFile("./my_source/AreaSelect.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::AREA_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(1);
	//WhiteCursorUpdate(true);
}
void object_work::SetNoneMode() {
	qt_data.brush_sliderAction->setVisible(false);
	qt_data.brush_spinBoxAction->setVisible(false);

	QIcon the_icon;
	the_icon.addFile("./my_source/NonMode.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(0);
	//WhiteCursorUpdate(true);
}