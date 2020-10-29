#include "AUX_UI.h"
#include <qcolordialog.h>

#include <qdebug.h>
#include <qtoolbar.h>

//--------import pointcloud-------
#include <qinputdialog.h>
#include "LayerControl.h"
#include "CloudPoints_IO.h"
#include "CloudPoints_Tools.h"

//1000avg
#include <pcl/kdtree/kdtree_flann.h>

 AUX_UI::AUX_UI(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	ui.treeView->setFocusPolicy(Qt::NoFocus);
	//------init tree view------
	standardModel = new QStandardItemModel(ui.treeView);
	ui.treeView->setHeaderHidden(true);
	item = standardModel->invisibleRootItem();
	ui.treeView->setModel(standardModel);
	ui.treeView->expandAll();
	//----------pcl visualizer----------
	sty_ovr = InteractorStyle_override::New();
	viewer.reset(new pcl::visualization::PCLVisualizer(__argc, __argv, "viewer", sty_ovr, false));
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	viewer->setBackgroundColor(0, 0, 0);
	//---------------------------------
	ColorScale::SetBaseColor(QColor(100, 100, 100));
	message = new QLabel(ui.statusBar);
	ui.treeView->setStyleSheet(QString::fromUtf8(" background-color:  rgb(255,255,255);"));
	//--------top tool bar--------------
	Top_toolBar = new QToolBar(this);
	Top_toolBar->setMovable(false);
	Top_toolBar->setObjectName(QString::fromUtf8("ToolBar"));
	Top_toolBar->setAutoFillBackground(true);
	this->addToolBar(Qt::TopToolBarArea, Top_toolBar);

	Tool_Mode = new my_toolButton(Top_toolBar, "Tool_Mode", "./my_source/NonMode.png");
	Top_toolBar->addWidget(Tool_Mode);

	brush_spinbox = new my_spinBox(Top_toolBar, "brush_spinbox");
	brush_spinbox->setRange(1, 100);
	Top_toolBar->addWidget(brush_spinbox);

	QLabel* sapceLable = new QLabel(NULL);
	Top_toolBar->addWidget(sapceLable);

	brush_slider = new my_slider(Top_toolBar);
	brush_slider->setRange(1, 100);
	brush_slider->setMaximumWidth(80);
	Top_toolBar->addWidget(brush_slider);

	QLabel* sapceLable_1 = new QLabel(NULL);
	Top_toolBar->addWidget(sapceLable_1);

	confirm_userSeg = new my_button(Top_toolBar, QString::fromUtf8("confirm"));
	confirm_userSeg->set_font_color(QColor(255, 255, 255));
	confirm_userSeg->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
	Top_toolBar->addWidget(confirm_userSeg);

	QWidget* SpaceExpand = new QWidget(Top_toolBar);
	SpaceExpand->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	Top_toolBar->addWidget(SpaceExpand);

	UI_Color_Style = new my_toolButton(Top_toolBar, "Color Style", "./my_source/UI_ColorChange.png");
	Top_toolBar->addWidget(UI_Color_Style);

	Viewer_Color_Style = new my_toolButton(Top_toolBar, "Viewer Color Style", "./my_source/color.jpg");
	Top_toolBar->addWidget(Viewer_Color_Style);
	//---------left tool bar---------
	New_Pointcloud = new my_toolButton(ui.mainToolBar, "New Pointcloud", "./my_source/NewFile.png");
	ui.mainToolBar->addWidget(New_Pointcloud);

	Exprot_Pointcloud = new my_toolButton(ui.mainToolBar, "Pointcloud Export", "./my_source/export-icon.png");
	ui.mainToolBar->addWidget(Exprot_Pointcloud);

	ui.mainToolBar->addSeparator();

	Area = new my_toolButton(ui.mainToolBar, "Area", "./my_source/AreaSelect.png");
	ui.mainToolBar->addWidget(Area);

	Brush = new my_toolButton(ui.mainToolBar, "Brush", "./my_source/cursor1-2.png");
	ui.mainToolBar->addWidget(Brush);

	Default = new my_toolButton(ui.mainToolBar, "Default", "./my_source/NonMode.png");
	ui.mainToolBar->addWidget(Default);

	QWidget* SpaceExpand_2 = new QWidget(Top_toolBar);
	SpaceExpand_2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	ui.mainToolBar->addWidget(SpaceExpand_2);

	TrashCan = new my_toolButton(ui.mainToolBar, "TrashCan", "./my_source/images.png");
	ui.mainToolBar->addWidget(TrashCan);
	//----groupbox(smooth)----
	smooth_groupbox = new my_foldGroupBox("Smooth", ui.dockWidgetContents, my_foldGroupBox::STATE_EXPAND);

	smooth_spinbox = new my_spinBox(smooth_groupbox, "smooth_spinbox");
	smooth_spinbox->setRange(1, 70);
	smooth_groupbox->addWidget(0, QFormLayout::LabelRole, smooth_spinbox);

	smooth_slider = new my_slider(smooth_groupbox);
	smooth_slider->setRange(1, 70);
	smooth_groupbox->addWidget(0, QFormLayout::FieldRole, smooth_slider);

	smooth_confirm = new my_button(smooth_groupbox, QString::fromUtf8("confirm"));
	smooth_confirm->set_font_color(QColor(255, 255, 255));
	smooth_groupbox->addWidget(1, QFormLayout::SpanningRole, smooth_confirm);
	//-----groupbox(preSegmentation)----
	preSeg_groupbox = new my_foldGroupBox("PreSegment", ui.dockWidgetContents, my_foldGroupBox::STATE_EXPAND);

	preSeg_spinbox = new my_spinBox(smooth_groupbox, "preSeg_spinBox");
	preSeg_groupbox->addWidget(0, QFormLayout::LabelRole, preSeg_spinbox);

	preSeg_slider = new my_slider(preSeg_groupbox);
	preSeg_groupbox->addWidget(0, QFormLayout::FieldRole, preSeg_slider);

	preSeg_confirm = new my_button(preSeg_groupbox, QString::fromUtf8("confirm"));
	preSeg_confirm->set_font_color(QColor(255, 255, 255));
	preSeg_groupbox->addWidget(1, QFormLayout::SpanningRole, preSeg_confirm);
	//---------
	ui.formLayout->setWidget(ui.formLayout->count() + 1, QFormLayout::FieldRole, smooth_groupbox);
	ui.formLayout->setWidget(ui.formLayout->count() + 1, QFormLayout::FieldRole, preSeg_groupbox);
	//------colordialog-----------
	QColorDialog* Qcolordia = new QColorDialog();
	connect(Qcolordia, SIGNAL(colorSelected(const QColor&)), this, SLOT(changeWindowsColor(const QColor&)));
	connect(UI_Color_Style, SIGNAL(clicked()), Qcolordia, SLOT(open()));
	//-----------spinbox & slider connect------------
	connect(brush_slider, SIGNAL(valueChanged(int)), brush_spinbox, SLOT(setValue(int)));
	connect(brush_spinbox, SIGNAL(valueChanged(int)), brush_slider, SLOT(setValue(int)));
	connect(smooth_slider, SIGNAL(valueChanged(int)), smooth_spinbox, SLOT(setValue(int)));
	connect(smooth_spinbox, SIGNAL(valueChanged(int)), smooth_slider, SLOT(setValue(int)));
	connect(preSeg_slider, SIGNAL(valueChanged(int)), preSeg_spinbox, SLOT(setValue(int)));
	connect(preSeg_spinbox, SIGNAL(valueChanged(int)), preSeg_slider, SLOT(setValue(int)));
	//----------Mode Change------
	connect(Brush, SIGNAL(clicked()), this, SLOT(SetBrushMode()));
	connect(Area, SIGNAL(clicked()), this, SLOT(SetAreaMode()));
	connect(Default, SIGNAL(clicked()), this, SLOT(SetNoneMode()));
	//--------------
	changeWindowsColor(ColorScale::Color_struct.colorC);
	message->clear();
	ui.statusBar->addPermanentWidget(message);
	//------^ UI Setting ^--------
	//------v ToolConnect v-------
	Init_Basedata();
	Set_ToolConnect();
}

void AUX_UI::changeWindowsColor(const QColor& c) {
	ColorScale::SetBaseColor(c);

	Top_toolBar->setStyleSheet(QString("background-color: rgb(%1, %2, %3);")
		.arg(ColorScale::Color_struct.colorE.red())
		.arg(ColorScale::Color_struct.colorE.green())
		.arg(ColorScale::Color_struct.colorE.blue()));
	Tool_Mode->set_styleSheet_color(ColorScale::Color_struct.colorE, ColorScale::Color_struct.colorE);
	brush_spinbox->SetSliderStylesheet_default(ColorScale::Color_struct.colorE);
	brush_slider->SetSliderStylesheet_default(ColorScale::Color_struct.colorB,
		QColor(255, 255, 255), ColorScale::Color_struct.colorA);
	confirm_userSeg->set_styleSheet_color(ColorScale::Color_struct.colorC, ColorScale::Color_struct.colorB);
	UI_Color_Style->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	Viewer_Color_Style->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);

	QPalette pal_widget;
	pal_widget.setColor(QPalette::Window, ColorScale::Color_struct.colorC);
	ui.dockWidget->setPalette(pal_widget);
	ui.dockWidget_2->setPalette(pal_widget);
	smooth_confirm->set_styleSheet_color(ColorScale::Color_struct.colorD, ColorScale::Color_struct.colorB);
	preSeg_confirm->set_styleSheet_color(ColorScale::Color_struct.colorD, ColorScale::Color_struct.colorB);

	ui.mainToolBar->setStyleSheet(QString("background-color: rgb(%1, %2, %3);")
		.arg(ColorScale::Color_struct.colorE.red())
		.arg(ColorScale::Color_struct.colorE.green())
		.arg(ColorScale::Color_struct.colorE.blue()));
	New_Pointcloud->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	Exprot_Pointcloud->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	Area->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	Brush->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	Default->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);
	TrashCan->set_styleSheet_color(ColorScale::Color_struct.colorB, ColorScale::Color_struct.colorE);

	smooth_slider->SetSliderStylesheet_default(ColorScale::Color_struct.colorB,
		ColorScale::Color_struct.colorE, ColorScale::Color_struct.colorA);
	preSeg_slider->SetSliderStylesheet_default(ColorScale::Color_struct.colorB,
		ColorScale::Color_struct.colorE, ColorScale::Color_struct.colorA);
	smooth_spinbox->SetSliderStylesheet_default(ColorScale::Color_struct.colorE);
	preSeg_spinbox->SetSliderStylesheet_default(ColorScale::Color_struct.colorE);

	message->setText("Color changed!");
	ui.statusBar->addPermanentWidget(message);
}

void AUX_UI::Init_Basedata() {
	viewer->registerKeyboardCallback(&KeyBoard_eventController);
	viewer->registerMouseCallback(&cursor_BrushSelector);
	viewer->registerAreaPickingCallback(&Area_PointCloud_Selector);

	Selected_cloud.reset(new PointCloud<PointXYZRGB>);

	PointCloud<PointXYZRGB>::Ptr nullCloud(new PointCloud<PointXYZRGB>);
	viewer->addPointCloud(nullCloud, "cld");
	viewer->addPointCloud(nullCloud, "White_BrushCursorPoints");

	selectionModel = ui.treeView->selectionModel();
}

void AUX_UI::Set_ToolConnect() {
	//-------button tool control-------
	connect(New_Pointcloud, SIGNAL(clicked()), this, SLOT(Tree_importCloud()));
	//-------click layer------
	connect(selectionModel, SIGNAL(selectionChanged(const QItemSelection&, const QItemSelection&)), this,
		SLOT(Tree_selectionChangedSlot(const QItemSelection&, const QItemSelection&)));
	//------smooth------
	QObject::connect(smooth_confirm, SIGNAL(clicked()), this, SLOT(Tree_Smooth()));
	//------slider pre segmentation----
	QObject::connect(preSeg_spinbox, SIGNAL(valueChanged(int)), this, SLOT(Slider_PreSegCloud()));
	//confirm
	QObject::connect(preSeg_confirm, SIGNAL(clicked()), this, SLOT(Slider_confirmSegCloud()));
	//USER confirm
	QObject::connect(confirm_userSeg, SIGNAL(clicked()), this, SLOT(Tree_UserSegmentation()));
	//-------delete layer------
	QObject::connect(TrashCan, SIGNAL(clicked()), this, SLOT(Tree_deleteLayer()));
	//---------set brush----
	QObject::connect(brush_spinbox, SIGNAL(valueChanged(int)), this, SLOT(Brush_change()));
	//-------viewer color change----
	QColorDialog* Viewer_Qcolordia = new QColorDialog();
	connect(Viewer_Color_Style, SIGNAL(clicked()), Viewer_Qcolordia, SLOT(open()));
	connect(Viewer_Qcolordia, SIGNAL(colorSelected(const QColor&)), this, SLOT(changeViewerColor(const QColor&)));
}

void AUX_UI::changeViewerColor(const QColor& c) {
	viewer->setBackgroundColor((float)c.red() / 255, (float)c.green() / 255, (float)c.blue() / 255);
	qDebug() << c.red() << "," << c.green() << "," << c.blue();
}

void AUX_UI::Tree_importCloud() {
	RedSelectClear();
	initModes();

	CloudPoints_IO IO_Tool;
	if (!IO_Tool.CloudImport()) {
		QString selectedText = "Import fail.";
		message->setText(selectedText);
		return;
	}

	bool ok;
	QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),
		tr("Layer name:"), QLineEdit::Normal,
		QString::fromStdString(IO_Tool.file_name_), &ok);
	if (ok && !text.isEmpty()) {
		std::string BaseLayerName = text.toStdString();
		std::string objName = "NONE" + text.toStdString();
		TreeLayerController* tree_layerController = new TreeLayerController(standardModel);
		if (!tree_layerController->AddLayer(text, IO_Tool.import_cloud_->makeShared()))
			return;
		QString selectedText = "Import success.";
		message->setText(selectedText);
	}
}

void AUX_UI::ViewCloudUpdate(PointCloud<PointXYZRGB>::Ptr updateCloud, bool resetCamera) {
	viewer->updatePointCloud(updateCloud, "cld");
	if (resetCamera)
		viewer->resetCamera();
	ui.qvtkWidget->update();
}
void AUX_UI::RedSelectClear() {
	select_map.clear();
	Selected_cloud->clear();
}
void AUX_UI::initModes() {
	SetNoneMode();
	PointCloud<PointXYZRGB>::Ptr nullCloud(new PointCloud<PointXYZRGB>);
	viewer->removePointCloud("White_BrushCursorPoints");
	viewer->addPointCloud(nullCloud, "White_BrushCursorPoints");
}

QModelIndex AUX_UI::searchParent(QModelIndex index) {
	if (index.parent().row() == -1) {
		return index;
	}
	else {
		QModelIndex parentItem = index.parent();
		qDebug() << parentItem;
		while (parentItem.parent().row() != -1)
			parentItem = parentItem.parent();

		return parentItem;
	}
}

void AUX_UI::Tree_selectionChangedSlot(const QItemSelection&, const QItemSelection&) {
	RedSelectClear();
	SegClouds.clear();
	initModes();

	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	if (index.row() == -1) {
		PointCloud<PointXYZRGB>::Ptr nullCloud(new PointCloud<PointXYZRGB>);
		ViewCloudUpdate(nullCloud, true);
		return;
	}

	int size = 0;
	PointCloud<PointXYZRGB>::Ptr nowCloud(new PointCloud<PointXYZRGB>);
	if (std::string(standardModel->itemFromIndex(index)->data().typeName()) == "complax_cloudInformation")
	{
		size = standardModel->itemFromIndex(index)->data().value<complax_cloudInformation>().cloud_data->size();
		nowCloud = standardModel->itemFromIndex(index)->data().value<complax_cloudInformation>().cloud_data;
	}
	else if (std::string(standardModel->itemFromIndex(index)->data().typeName()) == "pcl::PointCloud<PointXYZRGB>::Ptr")
	{
		size = standardModel->itemFromIndex(index)->data().value<PointCloud<PointXYZRGB>::Ptr>()->size();
		nowCloud = standardModel->itemFromIndex(index)->data().value<PointCloud<PointXYZRGB>::Ptr>();
	}
	Selected_cloud->clear();
	Selected_cloud = nowCloud->makeShared();

	QModelIndex TopParent = searchParent(index);
	qDebug() << TopParent;
	PointCloud<PointXYZRGB>::Ptr TopCloud(new PointCloud<PointXYZRGB>);
	TopCloud=standardModel->itemFromIndex(TopParent)->data().value<PointCloud<PointXYZRGB>::Ptr>()->makeShared();
	ViewCloudUpdate(TopCloud, true);
	ViewCloudUpdate(nowCloud, false);

	QString selectedText = QString::fromStdString(std::to_string(size)) + " points.";
	message->setText(selectedText);

	//取1000點做平均取距離
	std::vector<int> k_indices;
	std::vector<float> k_sqr_distances;
	int n = 0;
	double norm = 0;
	int searched_points = 0;
	pcl::KdTreeFLANN<PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<PointXYZRGB>);
	tree->setInputCloud(nowCloud);

	for (int i = 0; i < (nowCloud->size() >= 1000 ? 1000 : nowCloud->size()); ++i)
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
		nowCloud_avg_distance = norm / searched_points;
		brush_radius = nowCloud_avg_distance * 50;
	}
	else {
		nowCloud_avg_distance = 0;
		brush_radius = 0;
	}
}

void AUX_UI::Tree_Smooth() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	RedSelectClear();
	CloudPoints_Tools cpTools;
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	PointCloud<PointXYZRGB>::Ptr cld = standardModel->itemFromIndex(index)->data().value<PointCloud<PointXYZRGB>::Ptr>();

	//30為搜尋範圍，*0.5搜尋半徑
	PointCloud<PointXYZRGB>::Ptr smooth_cld = cpTools.CloudSmooth(cld, nowCloud_avg_distance * smooth_spinbox->value() * 0.5);

	if (smooth_cld->size() > 0)
	{
		//data update
		QVariant itemCloud;
		itemCloud.setValue(smooth_cld);
		standardModel->itemFromIndex(index)->setData(itemCloud);
		//view update
		ViewCloudUpdate(smooth_cld, false);
	}
	else
	{
		message->setText("NO DATA AFTER SMOOTH,Please set a bigger value.");
	}
}

//segment
void AUX_UI::Slider_PreSegCloud() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	SegClouds.clear();
	CloudPoints_Tools cpTools;
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();

	PointCloud<PointXYZRGB>::Ptr database_cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cld(new PointCloud<PointXYZRGB>);
	if (std::string(standardModel->itemFromIndex(index)->data().typeName()) == "complax_cloudInformation") {
		copyPointCloud(*standardModel->itemFromIndex(index)->data().value<complax_cloudInformation>().cloud_data, *database_cloud);
		copyPointCloud(*standardModel->itemFromIndex(index)->data().value<complax_cloudInformation>().cloud_data, *cld);
	}
	else if (std::string(standardModel->itemFromIndex(index)->data().typeName()) == "pcl::PointCloud<PointXYZRGB>::Ptr") {
		copyPointCloud(*standardModel->itemFromIndex(index)->data().value<PointCloud<PointXYZRGB>::Ptr>(), *database_cloud);
		copyPointCloud(*standardModel->itemFromIndex(index)->data().value<PointCloud<PointXYZRGB>::Ptr>(), *cld);
	}

	std::vector<PointIndices> seg_cloud_2 = cpTools.CloudSegmentation(cld, preSeg_spinbox->value(), nowCloud_avg_distance);
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
		SegClouds.push_back(tmp);
	}
	ViewCloudUpdate(cld, false);

	RedSelectClear();
}
void AUX_UI::Slider_confirmSegCloud() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	for (int i = 0; i < SegClouds.size(); ++i)
	{
		QString segLayer ="child_" + QString::fromStdString(std::to_string(i));
		TreeLayerController ly(standardModel);

		QModelIndex index = ui.treeView->selectionModel()->currentIndex();
		if (!ly.AddLayer(segLayer, SegClouds[i], index))
			return;
	}
	QString children_message = SegClouds.size() <= 1 ?
		QString::fromStdString("Segment " + std::to_string(SegClouds.size()) + " child") :
		QString::fromStdString("Segment " + std::to_string(SegClouds.size()) + " children");
	message->setText(children_message);
	SegClouds.clear();
}
//USER segment
void AUX_UI::Tree_UserSegmentation() {
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	if (index.row() == -1)
		return;
	PointCloud<PointXYZRGB>::Ptr LayerCloud;
	if (std::string(standardModel->itemFromIndex(index)->data().typeName()) == "complax_cloudInformation")
		LayerCloud = standardModel->itemFromIndex(index)->data().value <complax_cloudInformation>().cloud_data;
	else if (std::string(standardModel->itemFromIndex(index)->data().typeName()) == "pcl::PointCloud<PointXYZRGB>::Ptr")
		LayerCloud = standardModel->itemFromIndex(index)->data().value <PointCloud<PointXYZRGB>::Ptr>();

	if (select_map.size() > 0)
	{
		bool ok;
		QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),
			tr("Layer name:"), QLineEdit::Normal,
			QDir::home().dirName(), &ok);
		if (ok && !text.isEmpty())
		{
			TreeLayerController ly(standardModel);
			PointCloud<PointXYZRGB>::Ptr newCloud(new PointCloud<PointXYZRGB>);

			for (map<int, PointXYZRGB>::iterator iter = select_map.begin(); iter != select_map.end(); ++iter)
				newCloud->push_back(LayerCloud->points.at(iter->first));

			if (!ly.AddLayer(text, newCloud->makeShared(), index))
				return;

			RedSelectClear();
		}
	}
}

void AUX_UI::Tree_deleteLayer() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	RedSelectClear();
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	//多層刪除
	if (standardModel->itemFromIndex(index)->hasChildren())
		standardModel->itemFromIndex(index)->removeRows(0, standardModel->itemFromIndex(index)->rowCount());
	//沒有(上層)父類
	if (index.parent().row() == -1)
		standardModel->removeRow(index.row());
	else
		standardModel->itemFromIndex(index)->parent()->removeRow(index.row());
}

//key
#include <vtkInteractorStyleRubberBandPick.h>
void AUX_UI::KeyBoard_eventController(const pcl::visualization::KeyboardEvent& event)
{
	if (event.isCtrlPressed())
		keyBoard_ctrl = true;
	else if (event.isAltPressed())
		keyBoard_alt = true;

	if (event.keyUp()) {
		keyBoard_ctrl = false;
		keyBoard_alt = false;
	}

	if ((event.getKeySym() == "x" || event.getKeySym() == "X") && event.keyDown()) {
		if (GLOBAL_SELECTMODE != SelectMode::AREA_SELECT_MODE)
		{
			GLOBAL_SELECTMODE = SelectMode::AREA_SELECT_MODE;
			brush_spinbox->setValue(1);

			brush_slider->setVisible(false);
			brush_spinbox->setVisible(false);
			QIcon the_icon;
			the_icon.addFile("./my_source/AreaSelect.png", QSize(), QIcon::Normal, QIcon::Off);
			Tool_Mode->setIcon(the_icon);
		}
		else
		{
			GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;
			brush_spinbox->setValue(1);

			brush_slider->setVisible(false);
			brush_spinbox->setVisible(false);
			QIcon the_icon;
			the_icon.addFile("./my_source/NonMode.png", QSize(), QIcon::Normal, QIcon::Off);
			Tool_Mode->setIcon(the_icon);
		}
	}

	if ((event.getKeySym() == "b" || event.getKeySym() == "B") && event.keyDown()) {
		if (GLOBAL_SELECTMODE != SelectMode::BRUSH_SELECT_MODE)
		{
			GLOBAL_SELECTMODE = SelectMode::BRUSH_SELECT_MODE;
			brush_spinbox->setValue(std::ceil(brush_radius / nowCloud_avg_distance) < 1 ? 1 :
				std::ceil(brush_radius / nowCloud_avg_distance));

			brush_slider->setVisible(true);
			brush_spinbox->setVisible(true);
			QIcon the_icon;
			the_icon.addFile("./my_source/cursor1-2.png", QSize(), QIcon::Normal, QIcon::Off);
			Tool_Mode->setIcon(the_icon);
		}
		else
		{
			GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;
			brush_spinbox->setValue(1);

			brush_slider->setVisible(false);
			brush_spinbox->setVisible(false);
			QIcon the_icon;
			the_icon.addFile("./my_source/NonMode.png", QSize(), QIcon::Normal, QIcon::Off);
			Tool_Mode->setIcon(the_icon);
		}
	}

	if ((event.getKeySym() == "n" || event.getKeySym() == "N") && event.keyDown() &&
		GLOBAL_SELECTMODE == SelectMode::BRUSH_SELECT_MODE) {
		brush_radius <= nowCloud_avg_distance ?
			brush_radius = nowCloud_avg_distance :
			brush_radius -= nowCloud_avg_distance;

		brush_spinbox->setValue(std::ceil(brush_radius / nowCloud_avg_distance) < 1 ? 1 :
			std::ceil(brush_radius / nowCloud_avg_distance));
		ui.qvtkWidget->update();
	}
	if ((event.getKeySym() == "m" || event.getKeySym() == "M") && event.keyDown() &&
		GLOBAL_SELECTMODE == SelectMode::BRUSH_SELECT_MODE) {
		brush_radius += nowCloud_avg_distance;
		brush_spinbox->setValue(std::ceil(brush_radius / nowCloud_avg_distance) < 1 ? 1 :
			std::ceil(brush_radius / nowCloud_avg_distance));
		ui.qvtkWidget->update();
	}
}

void AUX_UI::SetBrushMode() {
	brush_slider->setVisible(true);
	brush_spinbox->setVisible(true);
	brush_spinbox->setValue(std::ceil(brush_radius / nowCloud_avg_distance) < 1 ? 1 :
		std::ceil(brush_radius / nowCloud_avg_distance));

	QIcon the_icon;
	the_icon.addFile("./my_source/cursor1-2.png", QSize(), QIcon::Normal, QIcon::Off);
	Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::BRUSH_SELECT_MODE;
	sty_ovr->SetCurrentMode_AreaPick(0);
}
void AUX_UI::SetAreaMode() {
	brush_spinbox->setValue(1);
	brush_slider->setVisible(false);
	brush_spinbox->setVisible(false);

	QIcon the_icon;
	the_icon.addFile("./my_source/AreaSelect.png", QSize(), QIcon::Normal, QIcon::Off);
	Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::AREA_SELECT_MODE;
	sty_ovr->SetCurrentMode_AreaPick(1);
}
void AUX_UI::SetNoneMode() {
	brush_spinbox->setValue(1);
	brush_slider->setVisible(false);
	brush_spinbox->setVisible(false);

	QIcon the_icon;
	the_icon.addFile("./my_source/NonMode.png", QSize(), QIcon::Normal, QIcon::Off);
	Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;
	sty_ovr->SetCurrentMode_AreaPick(0);
}

//selector 
void AUX_UI::cursor_BrushSelector(const pcl::visualization::MouseEvent& event) {
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	if (index.row() == -1)
		return;
	PointCloud<PointXYZRGB>::Ptr LayerCloud;
	if (std::string(standardModel->itemFromIndex(index)->data().typeName()) == "complax_cloudInformation")
		LayerCloud = standardModel->itemFromIndex(index)->data().value <complax_cloudInformation>().cloud_data;
	else if (std::string(standardModel->itemFromIndex(index)->data().typeName()) == "pcl::PointCloud<PointXYZRGB>::Ptr")
		LayerCloud = standardModel->itemFromIndex(index)->data().value <PointCloud<PointXYZRGB>::Ptr>();

	if (LayerCloud->size() > 0 && GLOBAL_SELECTMODE == SelectMode::BRUSH_SELECT_MODE)
	{
		if (event.getType() == event.MouseMove)
		{
			vtkRenderWindowInteractor* viewer_interactor = viewer->getRenderWindow()->GetInteractor();
			vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast(viewer_interactor->GetPicker());
			float mouseX = (viewer_interactor->GetEventPosition()[0]);
			float mouseY = (viewer_interactor->GetEventPosition()[1]);
			viewer_interactor->StartPickCallback();
			//-------check-^^^---------
			vtkRenderer* ren = viewer_interactor->FindPokedRenderer(mouseX, mouseY);
			point_picker->Pick(mouseX, mouseY, 0.0, ren);
			double picked[3]; point_picker->GetPickPosition(picked);

			PointCloud<PointXYZRGB>::Ptr cursor_premark(new PointCloud<PointXYZRGB>);
			KdTreeFLANN<PointXYZRGB>::Ptr tree(new KdTreeFLANN<PointXYZRGB>);
			std::vector<int> foundPointID;
			std::vector<float> foundPointSquaredDistance;
			tree->setInputCloud(LayerCloud);
			PointXYZRGB pickPoint;
			pickPoint.x = (float)picked[0]; (float)pickPoint.y = picked[1]; (float)pickPoint.z = picked[2];
			pickPoint.r = 255, pickPoint.g = 255, pickPoint.b = 255;

			if (tree->radiusSearch(pickPoint, brush_radius, foundPointID, foundPointSquaredDistance) > 0)
			{
				for (int i = 0; i < foundPointID.size() - 1; ++i) {
					cursor_premark->push_back(LayerCloud->points[foundPointID[i]]);
					int nowLayer_selectedID = foundPointID[i];
					if (keyBoard_ctrl && select_map.find(nowLayer_selectedID) == select_map.end())
					{
						Selected_cloud->points.at(nowLayer_selectedID) = LayerCloud->points.at(nowLayer_selectedID);
						Selected_cloud->points.at(nowLayer_selectedID).r = 255;
						Selected_cloud->points.at(nowLayer_selectedID).g = 0;
						Selected_cloud->points.at(nowLayer_selectedID).b = 0;
						select_map.insert(pair<int, PointXYZRGB>(nowLayer_selectedID, LayerCloud->points.at(nowLayer_selectedID)));
					}

					else if (keyBoard_alt && select_map.find(nowLayer_selectedID) != select_map.end())
					{
						Selected_cloud->points.at(nowLayer_selectedID).r = LayerCloud->points.at(nowLayer_selectedID).r;
						Selected_cloud->points.at(nowLayer_selectedID).g = LayerCloud->points.at(nowLayer_selectedID).g;
						Selected_cloud->points.at(nowLayer_selectedID).b = LayerCloud->points.at(nowLayer_selectedID).b;
						select_map.erase(nowLayer_selectedID);
					}
				}
			}

			if (keyBoard_ctrl || keyBoard_alt)
			{
				ViewCloudUpdate(Selected_cloud->makeShared(), false);
			}
			visualization::PointCloudColorHandlerCustom<PointXYZRGB> white(cursor_premark, 255, 255, 255);
			viewer->removePointCloud("White_BrushCursorPoints");
			viewer->addPointCloud(cursor_premark, white, "White_BrushCursorPoints");
			ui.qvtkWidget->update();
		}
	}
}

void AUX_UI::Area_PointCloud_Selector(const pcl::visualization::AreaPickingEvent& event) {
	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	if (index.row() == -1)
		return;

	PointCloud<PointXYZRGB>::Ptr LayerCloud;
	if (std::string(standardModel->itemFromIndex(index)->data().typeName()) == "complax_cloudInformation")
		LayerCloud = standardModel->itemFromIndex(index)->data().value <complax_cloudInformation>().cloud_data;
	else if (std::string(standardModel->itemFromIndex(index)->data().typeName()) == "pcl::PointCloud<PointXYZRGB>::Ptr")
		LayerCloud = standardModel->itemFromIndex(index)->data().value <PointCloud<PointXYZRGB>::Ptr>();

	//AREA PICK CLOUD
	std::vector<int> foundPointID;
	if (event.getPointsIndices(foundPointID) <= 0) {
		//快速多次選取閃退(原因為更新點雲ViewCloudUpdate)
		//改成viewer->updatePointcloud()
		if (!select_map.empty()) {
			select_map.clear();
			Selected_cloud->clear();
			ViewCloudUpdate(LayerCloud->makeShared(), false);
		}
		return;
	}

	if (keyBoard_ctrl) {
		for (int i = 0; i < foundPointID.size(); ++i) {
			int nowLayer_selectedID = foundPointID[i];
			if (select_map.find(nowLayer_selectedID) == select_map.end())
			{
				select_map.insert(pair<int, PointXYZRGB>(nowLayer_selectedID, LayerCloud->points.at(nowLayer_selectedID)));
			}
		}
	}
	else if (keyBoard_alt)
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
		Selected_cloud->clear();
		Selected_cloud->resize(LayerCloud->size());
		for (int i = 0; i < foundPointID.size(); ++i) {
			int nowLayer_selectedID = foundPointID[i];
			if (select_map.find(nowLayer_selectedID) == select_map.end()) {
				select_map.insert(pair<int, PointXYZRGB>(nowLayer_selectedID, LayerCloud->points.at(nowLayer_selectedID)));
			}
		}
	}
	Selected_cloud = LayerCloud->makeShared();
	for (map<int, PointXYZRGB>::iterator iter = select_map.begin(); iter != select_map.end(); ++iter)
	{
		Selected_cloud->points.at(iter->first).r = 255;
		Selected_cloud->points.at(iter->first).g = 0;
		Selected_cloud->points.at(iter->first).b = 0;
	}
	ViewCloudUpdate(Selected_cloud, false);
}
//-------brush-----
void  AUX_UI::Brush_change() {
	if (GLOBAL_SELECTMODE == SelectMode::BRUSH_SELECT_MODE)
	{
		brush_radius = nowCloud_avg_distance * brush_spinbox->value();
		ui.qvtkWidget->update();
	}
}