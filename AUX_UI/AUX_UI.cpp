#include "AUX_UI.h"
#include <qcolordialog.h>

#include <qdebug.h>
#include <qtoolbar.h>

//--------import pointcloud-------
#include <qinputdialog.h>
#include "QVTKWidget_controller/include/LayerControl.h"
#include "QVTKWidget_controller/include/CloudPoints_IO.h"
#include "QVTKWidget_controller/include/CloudPoints_Tools.h"

//1000avg
#include <pcl/kdtree/kdtree_flann.h>

AUX_UI::AUX_UI(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//------init tree view------
	ui.treeView->setFocusPolicy(Qt::NoFocus);
	ui.treeView->setContextMenuPolicy(Qt::CustomContextMenu);
	qt_data.standardModel = new QStandardItemModel(ui.treeView);
	ui.treeView->setHeaderHidden(true);
	ui.treeView->setModel(qt_data.standardModel);
	ui.treeView->expandAll();
	ui.treeView->setSelectionMode(QAbstractItemView::ExtendedSelection);
	//----------pcl visualizer----------
	pcl_data.my_interactorStyle = InteractorStyle_override::New();
	pcl_data.viewer.reset(new pcl::visualization::PCLVisualizer(__argc, __argv, "pcl_data.viewer", pcl_data.my_interactorStyle, false));
	ui.qvtkWidget->SetRenderWindow(pcl_data.viewer->getRenderWindow());
	pcl_data.viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	pcl_data.viewer->setBackgroundColor(0, 0, 0);
	//---------------------------------
	ColorScale::SetBaseColor(QColor(100, 100, 100));
	my_ui.message = new QLabel(ui.statusBar);
	ui.treeView->setStyleSheet(QString::fromUtf8(" background-color:  rgb(255,255,255);"));
	//--------top tool bar--------------
	my_ui.Top_toolBar = new QToolBar(this);
	my_ui.Top_toolBar->setMovable(false);
	my_ui.Top_toolBar->setObjectName(QString::fromUtf8("ToolBar"));
	my_ui.Top_toolBar->setAutoFillBackground(true);
	this->addToolBar(Qt::TopToolBarArea, my_ui.Top_toolBar);

	my_ui.Tool_Mode = new my_toolButton(my_ui.Top_toolBar, "Tool_Mode", "./my_source/NonMode.png");
	my_ui.Top_toolBar->addWidget(my_ui.Tool_Mode);

	my_ui.brush_spinbox = new my_spinBox(my_ui.Top_toolBar, "brush_spinbox");
	my_ui.brush_spinbox->setRange(1, 300);
	qt_data.brush_spinBoxAction = my_ui.Top_toolBar->addWidget(my_ui.brush_spinbox);
	qt_data.brush_spinBoxAction->setVisible(false);

	QLabel* sapceLable = new QLabel(NULL);
	my_ui.Top_toolBar->addWidget(sapceLable);

	my_ui.brush_slider = new my_slider(my_ui.Top_toolBar);
	my_ui.brush_slider->setRange(1, 300);
	my_ui.brush_slider->setMaximumWidth(80);
	qt_data.brush_sliderAction = my_ui.Top_toolBar->addWidget(my_ui.brush_slider);
	qt_data.brush_sliderAction->setVisible(false);

	QLabel* sapceLable_1 = new QLabel(NULL);
	my_ui.Top_toolBar->addWidget(sapceLable_1);

	my_ui.confirm_userSeg = new my_button(my_ui.Top_toolBar, QString::fromUtf8("Mark segment confirm"));
	my_ui.confirm_userSeg->set_font_color(QColor(255, 255, 255));
	my_ui.confirm_userSeg->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
	my_ui.Top_toolBar->addWidget(my_ui.confirm_userSeg);

	QWidget* SpaceExpand = new QWidget(my_ui.Top_toolBar);
	SpaceExpand->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	my_ui.Top_toolBar->addWidget(SpaceExpand);

	my_ui.UI_Color_Style = new my_toolButton(my_ui.Top_toolBar, "Color Style", "./my_source/UI_ColorChange.png");
	my_ui.Top_toolBar->addWidget(my_ui.UI_Color_Style);

	my_ui.Viewer_Color_Style = new my_toolButton(my_ui.Top_toolBar, "pcl_data.viewer Color Style", "./my_source/color.jpg");
	my_ui.Top_toolBar->addWidget(my_ui.Viewer_Color_Style);
	//---------left tool bar---------
	my_ui.New_Pointcloud = new my_toolButton(ui.mainToolBar, "New Pointcloud", "./my_source/NewFile.png");
	ui.mainToolBar->addWidget(my_ui.New_Pointcloud);

	my_ui.Exprot_Pointcloud = new my_toolButton(ui.mainToolBar, "Pointcloud Export", "./my_source/export-icon.png");
	ui.mainToolBar->addWidget(my_ui.Exprot_Pointcloud);

	ui.mainToolBar->addSeparator();

	my_ui.Area = new my_toolButton(ui.mainToolBar, "Area", "./my_source/AreaSelect.png");
	ui.mainToolBar->addWidget(my_ui.Area);

	my_ui.Brush = new my_toolButton(ui.mainToolBar, "Brush", "./my_source/cursor1-2.png");
	ui.mainToolBar->addWidget(my_ui.Brush);

	my_ui.Default = new my_toolButton(ui.mainToolBar, "Default", "./my_source/NonMode.png");
	ui.mainToolBar->addWidget(my_ui.Default);

	QWidget* SpaceExpand_2 = new QWidget(my_ui.Top_toolBar);
	SpaceExpand_2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	ui.mainToolBar->addWidget(SpaceExpand_2);

	my_ui.TrashCan = new my_toolButton(ui.mainToolBar, "TrashCan", "./my_source/images.png");
	ui.mainToolBar->addWidget(my_ui.TrashCan);
	//----groupbox(smooth)----
	my_ui.smooth_groupbox = new my_foldGroupBox("Smooth", ui.dockWidgetContents, my_foldGroupBox::STATE_EXPAND);

	my_ui.smooth_spinbox = new my_spinBox(my_ui.smooth_groupbox, "smooth_spinbox");
	my_ui.smooth_spinbox->setRange(1, 70);
	my_ui.smooth_groupbox->addWidget(0, QFormLayout::LabelRole, my_ui.smooth_spinbox);

	my_ui.smooth_slider = new my_slider(my_ui.smooth_groupbox);
	my_ui.smooth_slider->setRange(1, 70);
	my_ui.smooth_groupbox->addWidget(0, QFormLayout::FieldRole, my_ui.smooth_slider);

	my_ui.smooth_confirm = new my_button(my_ui.smooth_groupbox, QString::fromUtf8("confirm"));
	my_ui.smooth_confirm->set_font_color(QColor(255, 255, 255));
	my_ui.smooth_groupbox->addWidget(1, QFormLayout::SpanningRole, my_ui.smooth_confirm);
	//-----groupbox(preSegmentation)----
	my_ui.preSeg_groupbox = new my_foldGroupBox("PreSegment", ui.dockWidgetContents, my_foldGroupBox::STATE_EXPAND);

	//EuclideanClusterExtraction
	my_ui.SegMode_button = new my_button(my_ui.preSeg_groupbox, QString::fromUtf8("Euclidean"));
	my_ui.SegMode_button->set_font_color(QColor(255, 255, 255));
	my_ui.preSeg_groupbox->addWidget(0, QFormLayout::SpanningRole, my_ui.SegMode_button);
	//----
	my_ui.preSeg_spinbox = new my_spinBox(my_ui.smooth_groupbox, "preSeg_spinBox");
	my_ui.preSeg_spinbox->setRange(0, 500);
	my_ui.preSeg_groupbox->addWidget(1, QFormLayout::LabelRole, my_ui.preSeg_spinbox);

	my_ui.preSeg_slider = new my_slider(my_ui.preSeg_groupbox);
	my_ui.preSeg_slider->setRange(0, 500);
	my_ui.preSeg_groupbox->addWidget(1, QFormLayout::FieldRole, my_ui.preSeg_slider);

	my_ui.preSeg_confirm = new my_button(my_ui.preSeg_groupbox, QString::fromUtf8("confirm"));
	my_ui.preSeg_confirm->set_font_color(QColor(255, 255, 255));
	my_ui.preSeg_groupbox->addWidget(2, QFormLayout::SpanningRole, my_ui.preSeg_confirm);
	//----groupbox(Color Filter)----
	my_ui.color_filter_groupbox = new my_foldGroupBox("Color Filter", ui.dockWidgetContents, my_foldGroupBox::STATE_EXPAND);

	my_ui.color_filter_start_button = new my_button(my_ui.color_filter_groupbox, QString::fromUtf8("Start"));
	my_ui.color_filter_start_button->set_font_color(QColor(255, 255, 255));
	my_ui.color_filter_groupbox->addWidget(0, QFormLayout::SpanningRole, my_ui.color_filter_start_button);
	//---------
	ui.formLayout->setWidget(ui.formLayout->count() + 1, QFormLayout::FieldRole, my_ui.smooth_groupbox);
	ui.formLayout->setWidget(ui.formLayout->count() + 1, QFormLayout::FieldRole, my_ui.preSeg_groupbox);
	ui.formLayout->setWidget(ui.formLayout->count() + 1, QFormLayout::FieldRole, my_ui.color_filter_groupbox);
	//------colordialog-----------
	QColorDialog* Qcolordia = new QColorDialog();
	connect(Qcolordia, SIGNAL(colorSelected(const QColor&)), this, SLOT(changeWindowsColor(const QColor&)));
	connect(my_ui.UI_Color_Style, SIGNAL(clicked()), Qcolordia, SLOT(open()));
	//-----------spinbox & slider connect------------
	connect(my_ui.brush_slider, SIGNAL(valueChanged(int)), my_ui.brush_spinbox, SLOT(setValue(int)));
	connect(my_ui.brush_spinbox, SIGNAL(valueChanged(int)), my_ui.brush_slider, SLOT(setValue(int)));
	connect(my_ui.smooth_slider, SIGNAL(valueChanged(int)), my_ui.smooth_spinbox, SLOT(setValue(int)));
	connect(my_ui.smooth_spinbox, SIGNAL(valueChanged(int)), my_ui.smooth_slider, SLOT(setValue(int)));
	connect(my_ui.preSeg_slider, SIGNAL(valueChanged(int)), my_ui.preSeg_spinbox, SLOT(setValue(int)));
	connect(my_ui.preSeg_spinbox, SIGNAL(valueChanged(int)), my_ui.preSeg_slider, SLOT(setValue(int)));
	//---------color segment------------
	connect(my_ui.color_filter_start_button, SIGNAL(clicked()), this, SLOT(Color_Segment()));
	//----------Mode Change------
	connect(my_ui.Brush, SIGNAL(clicked()), this, SLOT(SetBrushMode()));
	connect(my_ui.Area, SIGNAL(clicked()), this, SLOT(SetAreaMode()));
	connect(my_ui.Default, SIGNAL(clicked()), this, SLOT(SetNoneMode()));
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

	my_ui.color_filter_start_button->set_styleSheet_color(ColorScale::Color_struct.colorD, ColorScale::Color_struct.colorB);

	my_ui.message->setText("Color changed!");
	ui.statusBar->addPermanentWidget(my_ui.message);
}

void AUX_UI::Init_Basedata() {
	general_data.nowLayerCloud.reset(new PointCloud<PointXYZRGB>);
	general_data.brush_radius = 20;
	general_data.Selected_cloud.reset(new PointCloud<PointXYZRGB>);

	qt_data.selectionModel = ui.treeView->selectionModel();

	pcl_data.viewer->registerKeyboardCallback(&KeyBoard_eventController);
	pcl_data.viewer->registerMouseCallback(&cursor_BrushSelector);
	pcl_data.viewer->registerAreaPickingCallback(&Area_PointCloud_Selector);

	PointCloud<PointXYZRGB>::Ptr nullCloud(new PointCloud<PointXYZRGB>);
	pcl_data.viewer->addPointCloud(nullCloud, "cld");
	pcl_data.viewer->addPointCloud(nullCloud, "White_BrushCursorPoints");
}

void AUX_UI::Set_ToolConnect() {
	//-------import-------
	connect(my_ui.New_Pointcloud, SIGNAL(clicked()), this, SLOT(Tree_importCloud()));
	//-------export------
	connect(my_ui.Exprot_Pointcloud, SIGNAL(clicked()), this, SLOT(ExportCloud()));
	//-------click layer------
	connect(qt_data.selectionModel, SIGNAL(selectionChanged(const QItemSelection&, const QItemSelection&)), this,
		SLOT(Tree_selectionChangedSlot(const QItemSelection&, const QItemSelection&)));
	//------smooth------
	QObject::connect(my_ui.smooth_confirm, SIGNAL(clicked()), this, SLOT(Tree_Smooth()));
	//------slider pre segmentation----
	QObject::connect(my_ui.preSeg_spinbox, SIGNAL(valueChanged(int)), this, SLOT(Slider_PreSegCloud()));
	//confirm
	QObject::connect(my_ui.preSeg_confirm, SIGNAL(clicked()), this, SLOT(Slider_confirmSegCloud()));
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
	TreeLayerController ly(qt_data.standardModel);
	if (!ly.AddLayer("merge_layer", mergedCloud, searchParent(ui.treeView->selectionModel()->currentIndex().parent())))
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
void AUX_UI::Tree_importCloud() {
	RedSelectClear();
	SetNoneMode();

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
			QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),
				tr("Layer name:"), QLineEdit::Normal,
				IO_Tool.file_name_[i], &ok);
			if (ok && !text.isEmpty()) {
				std::string BaseLayerName = text.toStdString();
				std::string objName = "NONE" + text.toStdString();
				TreeLayerController* tree_layerController = new TreeLayerController(qt_data.standardModel);
				if (!tree_layerController->AddLayer(text, IO_Tool.import_cloud_[i].makeShared()))
					return;
				QString selectedText = "Import success.";
				my_ui.message->setText(selectedText);
			}
		}
	}
	ui.treeView->selectionModel()->clear();
}

void AUX_UI::ExportCloud() {
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

void AUX_UI::ViewCloudUpdate(PointCloud<PointXYZRGB>::Ptr updateCloud, bool resetCamera) {
	pcl_data.viewer->updatePointCloud(updateCloud, "cld");
	if (resetCamera)
		pcl_data.viewer->resetCamera();
	ui.qvtkWidget->update();
}
void AUX_UI::RedSelectClear() {
	select_map.clear();
	general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();
}
void AUX_UI::initModes() {
	SetNoneMode();
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

void AUX_UI::Tree_selectionChangedSlot(const QItemSelection&, const QItemSelection&) {
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
void AUX_UI::Slider_confirmSegCloud() {
	if (ui.treeView->selectionModel()->currentIndex().row() == -1)
		return;
	if (general_data.SegClouds.size() == 0)
		return;

	QModelIndex index = ui.treeView->selectionModel()->currentIndex();
	for (int i = 0; i < general_data.SegClouds.size(); ++i)
	{
		QString segLayer = QString::fromStdString(std::to_string(i));
		TreeLayerController ly(qt_data.standardModel);

		if (!ly.AddLayer(segLayer, general_data.SegClouds[i], searchParent(index)))
			return;
	}
	if (index.parent().row() != -1)
		Tree_deleteLayer();

	QString children_message = general_data.SegClouds.size() <= 1 ?
		QString::fromStdString("Segment " + std::to_string(general_data.SegClouds.size()) + " child") :
		QString::fromStdString("Segment " + std::to_string(general_data.SegClouds.size()) + " children");
	my_ui.message->setText(children_message);
	general_data.SegClouds.clear();

	ui.treeView->selectionModel()->clear();
}

void AUX_UI::Color_Segment() {
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
	seg_cloud_2 = cpTools.CloudSegmentation_RGB(cld);


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
			TreeLayerController ly(qt_data.standardModel);
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
			if (!ly.AddLayer(text, newCloud->makeShared(), searchParent(index)))
				return;
			if (newCloud2->size() > 0)
			{
				if (!ly.AddLayer("base", newCloud2->makeShared(), searchParent(index)))
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
			SetAreaMode();
		}
		else
		{
			SetNoneMode();
		}
	}

	if ((event.getKeySym() == "b" || event.getKeySym() == "B") && event.keyDown()) {
		if (GLOBAL_SELECTMODE != SelectMode::BRUSH_SELECT_MODE)
		{
			SetBrushMode();
			my_ui.brush_spinbox->setValue(general_data.brush_radius);

			QModelIndex index = ui.treeView->selectionModel()->currentIndex();
			if (index.row() == -1)
				return;
		}
		else
		{
			SetNoneMode();
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
#include <qaction.h>
void AUX_UI::SetBrushMode() {
	qt_data.brush_sliderAction->setVisible(true);
	qt_data.brush_spinBoxAction->setVisible(true);

	my_ui.brush_spinbox->setValue(general_data.brush_radius);

	QIcon the_icon;
	the_icon.addFile("./my_source/cursor1-2.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::BRUSH_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(0);

	WhiteCursorUpdate(false);
}
void AUX_UI::SetAreaMode() {
	qt_data.brush_sliderAction->setVisible(false);
	qt_data.brush_spinBoxAction->setVisible(false);

	QIcon the_icon;
	the_icon.addFile("./my_source/AreaSelect.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::AREA_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(1);

	WhiteCursorUpdate(true);
}
void AUX_UI::SetNoneMode() {
	qt_data.brush_sliderAction->setVisible(false);
	qt_data.brush_spinBoxAction->setVisible(false);

	QIcon the_icon;
	the_icon.addFile("./my_source/NonMode.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(0);

	WhiteCursorUpdate(true);
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