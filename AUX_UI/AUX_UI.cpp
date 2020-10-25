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
	toolBar = new QToolBar(this);
	toolBar->setMovable(false);
	toolBar->setObjectName(QString::fromUtf8("ToolBar"));
	toolBar->setAutoFillBackground(true);
	this->addToolBar(Qt::TopToolBarArea, toolBar);

	Tool_Mode = new my_toolButton(toolBar, "Tool_Mode", "./my_source/NonMode.png");
	toolBar->addWidget(Tool_Mode);

	brush_spinbox = new my_spinBox(toolBar, "brush_spinbox");
	brush_spinbox->setRange(1, 99);
	toolBar->addWidget(brush_spinbox);

	QLabel* sapceLable = new QLabel(NULL);
	toolBar->addWidget(sapceLable);

	brush_slider = new my_slider(toolBar);
	brush_slider->setMaximumWidth(80);
	toolBar->addWidget(brush_slider);

	QLabel* sapceLable_1 = new QLabel(NULL);
	toolBar->addWidget(sapceLable_1);

	confirm_userSeg = new my_button(toolBar, QString::fromUtf8("confirm"));
	confirm_userSeg->set_font_color(QColor(255, 255, 255));
	confirm_userSeg->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
	toolBar->addWidget(confirm_userSeg);

	QWidget* SpaceExpand = new QWidget(toolBar);
	SpaceExpand->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	toolBar->addWidget(SpaceExpand);

	UI_Color_Style = new my_toolButton(toolBar, "Color Style", "./my_source/UI_ColorChange.png");
	toolBar->addWidget(UI_Color_Style);

	Viewer_Color_Style = new my_toolButton(toolBar, "Viewer Color Style", "./my_source/color.jpg");
	toolBar->addWidget(Viewer_Color_Style);
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

	QWidget* SpaceExpand_2 = new QWidget(toolBar);
	SpaceExpand_2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	ui.mainToolBar->addWidget(SpaceExpand_2);

	TrashCan = new my_toolButton(ui.mainToolBar, "TrashCan", "./my_source/images.png");
	ui.mainToolBar->addWidget(TrashCan);
	//----groupbox(smooth)----
	smooth_groupbox = new my_foldGroupBox("Smooth", ui.dockWidgetContents, my_foldGroupBox::STATE_EXPAND);

	smooth_spinbox = new my_spinBox(smooth_groupbox, "smooth_spinbox");
	smooth_groupbox->addWidget(0, QFormLayout::LabelRole, smooth_spinbox);

	smooth_slider = new my_slider(smooth_groupbox);
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

	toolBar->setStyleSheet(QString("background-color: rgb(%1, %2, %3);")
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

	PointCloud<PointXYZRGB>::Ptr nullCloud(new PointCloud<PointXYZRGB>);
	viewer->addPointCloud(nullCloud, "cld");

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
	QObject::connect(preSeg_spinbox, SIGNAL(valueChanged(int)), this, SLOT(Tree_Slider_PreSegCloud()));
	//conform
	QObject::connect(preSeg_confirm, SIGNAL(clicked()), this, SLOT(Tree_confirmSegCloud()));
	//-------delete layer------
	QObject::connect(TrashCan, SIGNAL(clicked()), this, SLOT(Tree_deleteLayer()));
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
	//RedSelectClear();
	//initModes();

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
	viewer->removePointCloud("cld");
	viewer->addPointCloud(updateCloud, "cld");
	if (resetCamera)
		viewer->resetCamera();
	ui.qvtkWidget->update();
}

void AUX_UI::Tree_selectionChangedSlot(const QItemSelection&, const QItemSelection&) {
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
	ViewCloudUpdate(nowCloud, true);
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
		//data.brush_radius = nowCloud_avg_distance * 50;
	}
	else {
		nowCloud_avg_distance = 0;
		//data.brush_radius = 0;
	}
}

void AUX_UI::Tree_Smooth() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	//RedSelectClear();
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

		//data.Selected_cloud = smooth_cld->makeShared();
		//view update
		ViewCloudUpdate(smooth_cld, false);
	}
	else
	{
		message->setText("NO DATA AFTER SMOOTH,Please set a bigger value.");
	}
}

void AUX_UI::Tree_Slider_PreSegCloud() {
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

	//---clear red_chosenPoints
	//RedSelectClear();
}
void AUX_UI::Tree_confirmSegCloud() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	for (int i = 0; i < SegClouds.size(); ++i)
	{
		QString segLayer = ui.treeView->selectionModel()->currentIndex().data(Qt::DisplayRole).toString() + "_" + QString::fromStdString(std::to_string(i));
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

void AUX_UI::Tree_deleteLayer() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	//RedSelectClear();
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
	if ((event.getKeySym() == "x" || event.getKeySym() == "X") && event.keyDown()) {
		if (GLOBAL_SELECTMODE == SelectMode::NO_SELECT_MODE || GLOBAL_SELECTMODE == SelectMode::BRUSH_SELECT_MODE)
		{
			GLOBAL_SELECTMODE = SelectMode::AREA_SELECT_MODE;

			QIcon the_icon;
			the_icon.addFile("./my_source/AreaSelect.png", QSize(), QIcon::Normal, QIcon::Off);
			Tool_Mode->setIcon(the_icon);
		}
		else
		{
			GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;

			QIcon the_icon;
			the_icon.addFile("./my_source/NonMode.png", QSize(), QIcon::Normal, QIcon::Off);
			Tool_Mode->setIcon(the_icon);
		}
	}

	if ((event.getKeySym() == "b" || event.getKeySym() == "B") && event.keyDown()) {
		if (GLOBAL_SELECTMODE == SelectMode::NO_SELECT_MODE || GLOBAL_SELECTMODE == SelectMode::AREA_SELECT_MODE)
		{
			GLOBAL_SELECTMODE = SelectMode::BRUSH_SELECT_MODE;
			QIcon the_icon;
			the_icon.addFile("./my_source/cursor1-2.png", QSize(), QIcon::Normal, QIcon::Off);
			Tool_Mode->setIcon(the_icon);
		}
		else
		{
			GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;

			QIcon the_icon;
			the_icon.addFile("./my_source/NonMode.png", QSize(), QIcon::Normal, QIcon::Off);
			Tool_Mode->setIcon(the_icon);
		}
	}
}

void AUX_UI::SetBrushMode() {
	QIcon the_icon;
	the_icon.addFile("./my_source/cursor1-2.png", QSize(), QIcon::Normal, QIcon::Off);
	Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::BRUSH_SELECT_MODE;
	sty_ovr->SetCurrentMode_AreaPick(0);
}
void AUX_UI::SetAreaMode() {
	QIcon the_icon;
	the_icon.addFile("./my_source/AreaSelect.png", QSize(), QIcon::Normal, QIcon::Off);
	Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::AREA_SELECT_MODE;
	sty_ovr->SetCurrentMode_AreaPick(1);	
}
void AUX_UI::SetNoneMode() {
	QIcon the_icon;
	the_icon.addFile("./my_source/NonMode.png", QSize(), QIcon::Normal, QIcon::Off);
	Tool_Mode->setIcon(the_icon);
	sty_ovr->SetCurrentMode_AreaPick(0);
}