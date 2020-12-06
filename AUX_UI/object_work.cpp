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
void object_work::ImportCloud() {
	tree_layerController = new TreeLayerController(qt_data.standardModel);
	//RedSelectClear();

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
void object_work::ExportCloud() {
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