#include "AUX_UI.h"
#include <qcolordialog.h>

#include <qdebug.h>
#include <qtoolbar.h>

//--------import pointcloud-------
#include <qinputdialog.h>
#include "QVTKWidget_controller/include/CloudPoints_IO.h"
#include "QVTKWidget_controller/include/CloudPoints_Tools.h"

//1000avg
#include <pcl/kdtree/kdtree_flann.h>

extern Ui::AUX_UIClass ui;
extern my_UI::Obj_UIClass my_ui;

extern Data_Class::General_Data general_data;
extern Data_Class::Key_Data key_data;
extern Data_Class::PCL_Data pcl_data;
extern Data_Class::QT_Data qt_data;

extern TreeLayerController* tree_layerController;

AUX_UI::AUX_UI(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	ui_work uiw;
	uiw.Init_UI(this);
	//----------pcl visualizer----------
	pcl_data.my_interactorStyle = InteractorStyle_override::New();
	pcl_data.viewer.reset(new pcl::visualization::PCLVisualizer(__argc, __argv, "pcl_data.viewer", pcl_data.my_interactorStyle, false));
	ui.qvtkWidget->SetRenderWindow(pcl_data.viewer->getRenderWindow());
	ui.qvtkWidget->setFocusPolicy(Qt::StrongFocus);
	pcl_data.viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	pcl_data.viewer->setBackgroundColor(0, 0, 0);
	//------colordialog-----------
	QColorDialog* Qcolordia = new QColorDialog();
	connect(Qcolordia, SIGNAL(colorSelected(const QColor&)), this, SLOT(changeWindowsColor(const QColor&)));
	connect(my_ui.UI_Color_Style, SIGNAL(clicked()), Qcolordia, SLOT(open()));

	ui_connect u_connect;
	u_connect.Init_Ui_connect();
	
	//--------------
	changeWindowsColor(ColorScale::Color_struct.colorC);
	my_ui.message->clear();
	ui.statusBar->addPermanentWidget(my_ui.message);
	//------^ UI Setting ^--------	
	//------v ToolConnect v-------
	Init_Basedata();
	Set_ToolConnect();
}

void AUX_UI::changeWindowsColor(const QColor& c) {
	ColorScale::SetBaseColor(c);

	my_ui.Top_toolBar->setStyleSheet(QString("background-color: rgb(%1, %2, %3);")
		.arg(ColorScale::Color_struct.colorE.red())
		.arg(ColorScale::Color_struct.colorE.green())
		.arg(ColorScale::Color_struct.colorE.blue()));
	my_ui.Tool_Mode->set_styleSheet_color(ColorScale::Color_struct.colorE, ColorScale::Color_struct.colorE);
	my_ui.brush_spinbox->SetSliderStylesheet_default(ColorScale::Color_struct.colorE);
	my_ui.brush_slider->SetSliderStylesheet_default(ColorScale::Color_struct.colorB,
		QColor(255, 255, 255), ColorScale::Color_struct.colorA);
	my_ui.confirm_userSeg->set_styleSheet_color(ColorScale::Color_struct.colorC, ColorScale::Color_struct.colorB);
	my_ui.UI_Color_Style->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	my_ui.Viewer_Color_Style->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);

	QPalette pal_widget;
	pal_widget.setColor(QPalette::Window, ColorScale::Color_struct.colorC);
	ui.dockWidget->setPalette(pal_widget);
	ui.dockWidget_2->setPalette(pal_widget);
	my_ui.smooth_confirm->set_styleSheet_color(ColorScale::Color_struct.colorD, ColorScale::Color_struct.colorB);
	my_ui.preSeg_confirm->set_styleSheet_color(ColorScale::Color_struct.colorD, ColorScale::Color_struct.colorB);

	ui.mainToolBar->setStyleSheet(QString("background-color: rgb(%1, %2, %3);")
		.arg(ColorScale::Color_struct.colorE.red())
		.arg(ColorScale::Color_struct.colorE.green())
		.arg(ColorScale::Color_struct.colorE.blue()));
	my_ui.New_Pointcloud->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	my_ui.Exprot_Pointcloud->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	my_ui.Area->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	my_ui.Brush->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	my_ui.Default->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	my_ui.TrashCan->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);

	my_ui.smooth_spinbox->SetSliderStylesheet_default(ColorScale::Color_struct.colorE);
	my_ui.smooth_slider->SetSliderStylesheet_default(ColorScale::Color_struct.colorB,
		ColorScale::Color_struct.colorE, ColorScale::Color_struct.colorA);

	my_ui.SegMode_button->set_styleSheet_color(ColorScale::Color_struct.colorD, ColorScale::Color_struct.colorC);
	my_ui.preSeg_spinbox->SetSliderStylesheet_default(ColorScale::Color_struct.colorE);
	my_ui.preSeg_slider->SetSliderStylesheet_default(ColorScale::Color_struct.colorB,
		ColorScale::Color_struct.colorE, ColorScale::Color_struct.colorA);

	my_ui.H_range_spinbox->SetSliderStylesheet_default(ColorScale::Color_struct.colorE);
	my_ui.H_range_slider->SetSliderStylesheet_default(ColorScale::Color_struct.colorB,
		ColorScale::Color_struct.colorE, ColorScale::Color_struct.colorA);

	my_ui.V_range_spinbox->SetSliderStylesheet_default(ColorScale::Color_struct.colorE);
	my_ui.V_range_slider->SetSliderStylesheet_default(ColorScale::Color_struct.colorB,
		ColorScale::Color_struct.colorE, ColorScale::Color_struct.colorA);
	my_ui.color_filter_start_button->set_styleSheet_color(ColorScale::Color_struct.colorD, ColorScale::Color_struct.colorB);

	my_ui.leaf_spinbox->SetSliderStylesheet_default(ColorScale::Color_struct.colorE);
	my_ui.leaf_slider->SetSliderStylesheet_default(ColorScale::Color_struct.colorB,
		ColorScale::Color_struct.colorE, ColorScale::Color_struct.colorA);
	my_ui.color_filter_start_button->set_styleSheet_color(ColorScale::Color_struct.colorD, ColorScale::Color_struct.colorB);
	my_ui.pointDensity_start_button->set_styleSheet_color(ColorScale::Color_struct.colorD, ColorScale::Color_struct.colorB);

	my_ui.message->setText("Color changed!");
	ui.statusBar->addPermanentWidget(my_ui.message);
}

void AUX_UI::Init_Basedata() {
	general_data.rgb_data = QColor{ 128,128,128 };

	my_ui.H_range_spinbox->setValue(10);
	my_ui.V_range_spinbox->setValue(128);

	general_data.nowLayerCloud.reset(new PointCloud<PointXYZRGB>);
	general_data.brush_radius = 20;
	general_data.Selected_cloud.reset(new PointCloud<PointXYZRGB>);
	general_data.Voxel_cloud.reset(new PointCloud<PointXYZRGB>);

	pcl_data.viewer->registerKeyboardCallback(&KeyBoard_eventController);
	pcl_data.viewer->registerMouseCallback(&cursor_BrushSelector);
	pcl_data.viewer->registerAreaPickingCallback(&Area_PointCloud_Selector);

	PointCloud<PointXYZRGB>::Ptr nullCloud(new PointCloud<PointXYZRGB>);
	pcl_data.viewer->addPointCloud(nullCloud, "cld");
	pcl_data.viewer->addPointCloud(nullCloud, "White_BrushCursorPoints");
}

void AUX_UI::Set_ToolConnect() {	
	QObject::connect(my_ui.smooth_confirm, SIGNAL(clicked()), this, SLOT(Tree_Smooth()));
	//------slider pre segmentation----
	QObject::connect(my_ui.preSeg_spinbox, SIGNAL(valueChanged(int)), this, SLOT(Slider_PreSegCloud()));
	//--------color segment--------
	QColorDialog* Qcolordia_SegColor = new QColorDialog();
	connect(Qcolordia_SegColor, SIGNAL(colorSelected(const QColor&)), this, SLOT(Set_lightRange(const QColor&)));
	connect(my_ui.color_widget, SIGNAL(clicked()), Qcolordia_SegColor, SLOT(open()));
	connect(my_ui.color_widget, SIGNAL(clicked()), this, SLOT(reset_point_color()));
	connect(my_ui.V_range_spinbox, SIGNAL(valueChanged(int)), this, SLOT(Color_PreSegment()));
	connect(my_ui.H_range_spinbox, SIGNAL(valueChanged(int)), this, SLOT(Color_PreSegment()));
	connect(my_ui.color_filter_start_button, SIGNAL(clicked()), this, SLOT(confirm_colors_segment()));
	//confirm
	QObject::connect(my_ui.preSeg_confirm, SIGNAL(clicked()), this, SLOT(confirm_colors_segment()));
	//USER confirm
	QObject::connect(my_ui.confirm_userSeg, SIGNAL(clicked()), this, SLOT(Tree_UserSegmentation()));
	//-------delete layer------
	QObject::connect(my_ui.TrashCan, SIGNAL(clicked()), this, SLOT(Tree_deleteLayer()));
	//---------slider/spinbox set brush size----
	QObject::connect(my_ui.brush_spinbox, SIGNAL(valueChanged(int)), this, SLOT(Brush_SizeChange()));
	//------segmode change-------
	QObject::connect(my_ui.SegMode_button, SIGNAL(clicked()), this, SLOT(SegMode_Change()));
	//-------pcl_data.viewer/color change----
	QColorDialog* Viewer_Qcolordia = new QColorDialog();
	connect(my_ui.Viewer_Color_Style, SIGNAL(clicked()), Viewer_Qcolordia, SLOT(open()));
	connect(Viewer_Qcolordia, SIGNAL(colorSelected(const QColor&)), this, SLOT(changeViewerColor(const QColor&)));
	//-------layer merge------
	connect(ui.treeView, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(onCustomContextMenu(const QPoint&)));
}

void AUX_UI::reset_point_color() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	PointCloud<PointXYZRGB>::Ptr cld(new PointCloud<PointXYZRGB>);
	cld = qt_data.standardModel->itemFromIndex(index)->data().
		value<PointCloud<PointXYZRGB>::Ptr>()->makeShared();

	ViewCloudUpdate(cld, false);
	RedSelectClear();
}

void AUX_UI::onCustomContextMenu(const QPoint& point)
{
	QModelIndex index = ui.treeView->indexAt(point);
	QMenu menu;
	menu.addAction(QStringLiteral("merge"), this, SLOT(mergeLayer()));
	menu.addSeparator();
	menu.exec(ui.treeView->viewport()->mapToGlobal(point));
}

void AUX_UI::mergeLayer() {
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

void AUX_UI::changeViewerColor(const QColor& c) {
	pcl_data.viewer->setBackgroundColor((float)c.red() / 255, (float)c.green() / 255, (float)c.blue() / 255);
}

#include <qfiledialog.h>
#include <fstream>
#include <typeinfo>

void AUX_UI::ViewCloudUpdate(PointCloud<PointXYZRGB>::Ptr updateCloud, bool resetCamera) {
	/*pcl_data.viewer->updatePointCloud(updateCloud, "cld");
	if (resetCamera)
		pcl_data.viewer->resetCamera();
	ui.qvtkWidget->update();*/
}
void AUX_UI::RedSelectClear() {
	/*select_map.clear();
	general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();*/
}
void AUX_UI::initModes() {
	//SetNoneMode();
	PointCloud<PointXYZRGB>::Ptr nullCloud(new PointCloud<PointXYZRGB>);
	pcl_data.viewer->updatePointCloud(nullCloud, "White_BrushCursorPoints");
}
void AUX_UI::SegMode_Change() {
	if (GLOBAL_SEGMENTMODE == SegmentMode::REGION_GROWING) {
		GLOBAL_SEGMENTMODE = SegmentMode::EUCLIDEAN_CLUSTER_EXTRACTION;
		my_ui.SegMode_button->setText("Euclidean");
		my_ui.preSeg_slider->setRange(0, 500);
		my_ui.preSeg_spinbox->setRange(0, 500);
		my_ui.preSeg_spinbox->setValue(0);
	}
	else if (GLOBAL_SEGMENTMODE == SegmentMode::EUCLIDEAN_CLUSTER_EXTRACTION) {
		GLOBAL_SEGMENTMODE = SegmentMode::REGION_GROWING;
		my_ui.SegMode_button->setText("Region Growing");
		my_ui.preSeg_slider->setRange(0, 200);
		my_ui.preSeg_spinbox->setRange(0, 200);
		my_ui.preSeg_spinbox->setValue(0);
	}
}

QModelIndex AUX_UI::searchParent(QModelIndex index) {
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

void AUX_UI::Tree_Smooth() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	RedSelectClear();
	CloudPoints_Tools cpTools;
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	PointCloud<PointXYZRGB>::Ptr cld = qt_data.standardModel->itemFromIndex(index)->data().value<PointCloud<PointXYZRGB>::Ptr>();

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
	}
	else
	{
		my_ui.message->setText("NO DATA AFTER SMOOTH,Please set a bigger value.");
	}

	ui.treeView->selectionModel()->clear();
}

//segment
void AUX_UI::Slider_PreSegCloud() {
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
}
void AUX_UI::confirm_colors_segment() {
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

void AUX_UI::Set_lightRange(const QColor& c) {
	general_data.rgb_data = QColor{ c.red(), c.green(), c.blue() };
	Color_PreSegment();
}
void AUX_UI::Color_PreSegment() {
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
void AUX_UI::Color_Segment() {

}

//USER segment
void AUX_UI::Tree_UserSegmentation() {
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	if (index.row() == -1)
		return;

	if (select_map.size() > 0)
	{
		bool ok;
		QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),
			tr("Layer name:"), QLineEdit::Normal,
			QDir::home().dirName(), &ok);
		if (ok && !text.isEmpty())
		{
			PointCloud<PointXYZRGB>::Ptr newCloud(new PointCloud<PointXYZRGB>);
			PointCloud<PointXYZRGB>::Ptr newCloud2(new PointCloud<PointXYZRGB>);

			for (int i = 0; i < general_data.nowLayerCloud->size(); ++i)
			{
				if (select_map.find(i) != select_map.end())
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
				Tree_deleteLayer();

			ui.treeView->selectionModel()->clear();
			RedSelectClear();
		}
	}
}

void AUX_UI::Tree_deleteLayer() {
	QModelIndexList indexes = ui.treeView->selectionModel()->selectedIndexes();
	RedSelectClear();
	if (indexes.size() <= 0)
		return;
	qSort(indexes.begin(), indexes.end(), qGreater<QModelIndex>());
	vector<QModelIndex> parent_index;
	for (int i = 0; i < indexes.size(); ++i) {
		qDebug() << i;
		if (qt_data.standardModel->itemFromIndex(indexes[i])->parent() == NULL) {
			qDebug() << "PARENT";
			parent_index.push_back(indexes[i]);
		}
		else {
			qDebug() << "CHILD";
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

//key
#include <vtkInteractorStyleRubberBandPick.h>
void AUX_UI::KeyBoard_eventController(const pcl::visualization::KeyboardEvent& event)
{
	if (event.isCtrlPressed()) {
		key_data.keyBoard_ctrl = true;
	}
	else if (event.isAltPressed())
		key_data.keyBoard_alt = true;
	else if (event.isShiftPressed()) {

	}

	if (event.keyUp()) {
		key_data.keyBoard_ctrl = false;
		key_data.keyBoard_alt = false;
	}

	if ((event.getKeySym() == "x" || event.getKeySym() == "X") && event.keyDown()) {
		if (GLOBAL_SELECTMODE != SelectMode::AREA_SELECT_MODE)
		{
			//SetAreaMode();
		}
		else
		{
			//SetNoneMode();
		}
	}

	if ((event.getKeySym() == "b" || event.getKeySym() == "B") && event.keyDown()) {
		if (GLOBAL_SELECTMODE != SelectMode::BRUSH_SELECT_MODE)
		{
			//SetBrushMode();
			my_ui.brush_spinbox->setValue(general_data.brush_radius);

			QModelIndex index = ui.treeView->selectionModel()->currentIndex();
			if (index.row() == -1)
				return;
		}
		else
		{
			//SetNoneMode();
		}
	}

	if ((event.getKeySym() == "n" || event.getKeySym() == "N") && event.keyDown() &&
		GLOBAL_SELECTMODE == SelectMode::BRUSH_SELECT_MODE) {
		general_data.brush_radius - 1 < 1 ? general_data.brush_radius = 1 : --general_data.brush_radius;
		my_ui.brush_spinbox->setValue(general_data.brush_radius);
		WhiteCursorUpdate(false);
	}
	if ((event.getKeySym() == "m" || event.getKeySym() == "M") && event.keyDown() &&
		GLOBAL_SELECTMODE == SelectMode::BRUSH_SELECT_MODE) {
		++general_data.brush_radius;
		my_ui.brush_spinbox->setValue(general_data.brush_radius);
		WhiteCursorUpdate(false);
	}
}

//selector 
void AUX_UI::cursor_BrushSelector(const pcl::visualization::MouseEvent& event) {
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	if (index.row() == -1)
		return;

	if (general_data.nowLayerCloud->size() > 0 && GLOBAL_SELECTMODE == SelectMode::BRUSH_SELECT_MODE)
	{
		if (event.getType() == event.MouseMove)
		{
			vtkRenderWindowInteractor* viewer_interactor = pcl_data.viewer->getRenderWindow()->GetInteractor();
			vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast(viewer_interactor->GetPicker());
			float mouseX = (viewer_interactor->GetEventPosition()[0]);
			float mouseY = (viewer_interactor->GetEventPosition()[1]);
			viewer_interactor->StartPickCallback();
			vtkRenderer* ren = viewer_interactor->FindPokedRenderer(mouseX, mouseY);
			point_picker->Pick(mouseX, mouseY, 0.0, ren);
			double picked[3]; point_picker->GetPickPosition(picked);

			PointCloud<PointXYZRGB>::Ptr cursor_premark(new PointCloud<PointXYZRGB>);
			KdTreeFLANN<PointXYZRGB>::Ptr tree(new KdTreeFLANN<PointXYZRGB>);
			std::vector<int> foundPointID;
			std::vector<float> foundPointSquaredDistance;
			tree->setInputCloud(general_data.nowLayerCloud);
			PointXYZRGB pickPoint;
			pickPoint.x = (float)picked[0]; (float)pickPoint.y = picked[1]; (float)pickPoint.z = picked[2];
			pickPoint.r = 255, pickPoint.g = 255, pickPoint.b = 255;

			if (tree->radiusSearch(pickPoint, general_data.nowCloud_avg_distance * general_data.brush_radius, foundPointID, foundPointSquaredDistance) > 0)
			{
				for (int i = 0; i < foundPointID.size() - 1; ++i) {
					cursor_premark->push_back(general_data.nowLayerCloud->points[foundPointID[i]]);
					int nowLayer_selectedID = foundPointID[i];
					if (key_data.keyBoard_ctrl && select_map.find(nowLayer_selectedID) == select_map.end())
					{
						general_data.Selected_cloud->points.at(nowLayer_selectedID) = general_data.nowLayerCloud->points.at(nowLayer_selectedID);
						general_data.Selected_cloud->points.at(nowLayer_selectedID).r = 255;
						general_data.Selected_cloud->points.at(nowLayer_selectedID).g = 0;
						general_data.Selected_cloud->points.at(nowLayer_selectedID).b = 0;
						select_map.insert(pair<int, PointXYZRGB>(nowLayer_selectedID, general_data.nowLayerCloud->points.at(nowLayer_selectedID)));
					}

					else if (key_data.keyBoard_alt && select_map.find(nowLayer_selectedID) != select_map.end())
					{
						general_data.Selected_cloud->points.at(nowLayer_selectedID).r = general_data.nowLayerCloud->points.at(nowLayer_selectedID).r;
						general_data.Selected_cloud->points.at(nowLayer_selectedID).g = general_data.nowLayerCloud->points.at(nowLayer_selectedID).g;
						general_data.Selected_cloud->points.at(nowLayer_selectedID).b = general_data.nowLayerCloud->points.at(nowLayer_selectedID).b;
						select_map.erase(nowLayer_selectedID);
					}
				}
			}

			if (key_data.keyBoard_ctrl || key_data.keyBoard_alt)
			{
				general_data.SegClouds.clear();
				ViewCloudUpdate(general_data.Selected_cloud->makeShared(), false);
			}
			visualization::PointCloudColorHandlerCustom<PointXYZRGB> white(cursor_premark, 255, 255, 255);
			pcl_data.viewer->removePointCloud("White_BrushCursorPoints");
			pcl_data.viewer->addPointCloud(cursor_premark, white, "White_BrushCursorPoints");
			ui.qvtkWidget->update();
		}
	}
}

void AUX_UI::Area_PointCloud_Selector(const pcl::visualization::AreaPickingEvent& event) {
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	if (index.row() == -1)
		return;
	//AREA PICK CLOUD
	std::vector<int> foundPointID;
	if (event.getPointsIndices(foundPointID) <= 0) {
		if (!select_map.empty()) {
			select_map.clear();
			general_data.Selected_cloud->clear();
			general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();
			ViewCloudUpdate(general_data.nowLayerCloud->makeShared(), false);
		}
		return;
	}

	if (key_data.keyBoard_ctrl) {
		for (int i = 0; i < foundPointID.size(); ++i) {
			int nowLayer_selectedID = foundPointID[i];
			if (select_map.find(nowLayer_selectedID) == select_map.end())
			{
				select_map.insert(pair<int, PointXYZRGB>(nowLayer_selectedID, general_data.nowLayerCloud->points.at(nowLayer_selectedID)));
			}
		}
	}
	else if (key_data.keyBoard_alt)
	{
		for (int i = 0; i < foundPointID.size(); ++i) {
			int nowLayer_selectedID = foundPointID[i];
			if (select_map.find(nowLayer_selectedID) != select_map.end())
			{
				select_map.erase(nowLayer_selectedID);
			}
		}
	}
	else
	{
		select_map.clear();
		general_data.Selected_cloud->clear();
		general_data.Selected_cloud->resize(general_data.nowLayerCloud->size());
		for (int i = 0; i < foundPointID.size(); ++i) {
			int nowLayer_selectedID = foundPointID[i];
			if (select_map.find(nowLayer_selectedID) == select_map.end()) {
				select_map.insert(pair<int, PointXYZRGB>(nowLayer_selectedID, general_data.nowLayerCloud->points.at(nowLayer_selectedID)));
			}
		}
	}
	general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();
	for (map<int, PointXYZRGB>::iterator iter = select_map.begin(); iter != select_map.end(); ++iter)
	{
		general_data.Selected_cloud->points.at(iter->first).r = 255;
		general_data.Selected_cloud->points.at(iter->first).g = 0;
		general_data.Selected_cloud->points.at(iter->first).b = 0;
	}
	general_data.SegClouds.clear();
	ViewCloudUpdate(general_data.Selected_cloud, false);
}
//-------brush-----
void  AUX_UI::Brush_SizeChange() {
	if (GLOBAL_SELECTMODE == SelectMode::BRUSH_SELECT_MODE)
	{
		general_data.brush_radius = my_ui.brush_spinbox->value();
	}
}

void AUX_UI::WhiteCursorUpdate(bool whiteCursor_clear) {
	if (general_data.nowLayerCloud->size() > 0 && !whiteCursor_clear)
	{
		vtkRenderWindowInteractor* viewer_interactor = pcl_data.viewer->getRenderWindow()->GetInteractor();
		vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast(viewer_interactor->GetPicker());
		float mouseX = (viewer_interactor->GetEventPosition()[0]);
		float mouseY = (viewer_interactor->GetEventPosition()[1]);
		viewer_interactor->StartPickCallback();
		vtkRenderer* ren = viewer_interactor->FindPokedRenderer(mouseX, mouseY);
		point_picker->Pick(mouseX, mouseY, 0.0, ren);
		double picked[3]; point_picker->GetPickPosition(picked);

		PointCloud<PointXYZRGB>::Ptr cursor_premark(new PointCloud<PointXYZRGB>);
		KdTreeFLANN<PointXYZRGB>::Ptr tree(new KdTreeFLANN<PointXYZRGB>);
		std::vector<int> foundPointID;
		std::vector<float> foundPointSquaredDistance;
		tree->setInputCloud(general_data.nowLayerCloud);
		PointXYZRGB pickPoint;
		pickPoint.x = (float)picked[0]; (float)pickPoint.y = picked[1]; (float)pickPoint.z = picked[2];
		pickPoint.r = 255, pickPoint.g = 255, pickPoint.b = 255;

		if (tree->radiusSearch(pickPoint, general_data.nowCloud_avg_distance * general_data.brush_radius, foundPointID, foundPointSquaredDistance) > 0)
		{
			for (int i = 0; i < foundPointID.size() - 1; ++i) {
				cursor_premark->push_back(general_data.nowLayerCloud->points[foundPointID[i]]);
			}
		}

		visualization::PointCloudColorHandlerCustom<PointXYZRGB> white(cursor_premark, 255, 255, 255);
		pcl_data.viewer->removePointCloud("White_BrushCursorPoints");
		pcl_data.viewer->addPointCloud(cursor_premark, white, "White_BrushCursorPoints");
		ui.qvtkWidget->update();
	}
	else if (whiteCursor_clear)
	{
		PointCloud<PointXYZRGB>::Ptr nullCloud(new PointCloud<PointXYZRGB>);
		pcl_data.viewer->updatePointCloud(nullCloud, "White_BrushCursorPoints");
		ui.qvtkWidget->update();
	}
}