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

	//---------slider/spinbox set brush size----
	QObject::connect(my_ui.brush_spinbox, SIGNAL(valueChanged(int)), this, SLOT(Brush_SizeChange()));
	//------segmode change-------
	QObject::connect(my_ui.SegMode_button, SIGNAL(clicked()), this, SLOT(SegMode_Change()));
	//-------pcl_data.viewer/color change----
	QColorDialog* Viewer_Qcolordia = new QColorDialog();
	connect(my_ui.Viewer_Color_Style, SIGNAL(clicked()), Viewer_Qcolordia, SLOT(open()));
	connect(Viewer_Qcolordia, SIGNAL(colorSelected(const QColor&)), this, SLOT(changeViewerColor(const QColor&)));
}

void AUX_UI::changeViewerColor(const QColor& c) {
	pcl_data.viewer->setBackgroundColor((float)c.red() / 255, (float)c.green() / 255, (float)c.blue() / 255);
}

#include <qfiledialog.h>
#include <fstream>
#include <typeinfo>

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
		object_work::ViewCloudUpdate(general_data.Selected_cloud, false);
		if (GLOBAL_SELECTMODE != SelectMode::AREA_SELECT_MODE)
		{
			object_work::SetAreaMode();
			WhiteCursorUpdate(true); 			
		}
		else
		{
			object_work::SetNoneMode();
			WhiteCursorUpdate(true);
		}
	}

	if ((event.getKeySym() == "b" || event.getKeySym() == "B") && event.keyDown()) {
		object_work::ViewCloudUpdate(general_data.Selected_cloud, false);
		if (GLOBAL_SELECTMODE != SelectMode::BRUSH_SELECT_MODE)
		{
			object_work::SetBrushMode();
			WhiteCursorUpdate(false);			
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
					if (key_data.keyBoard_ctrl && map_redSelected->find(nowLayer_selectedID) == map_redSelected->end())
					{
						general_data.Selected_cloud->points.at(nowLayer_selectedID) = general_data.nowLayerCloud->points.at(nowLayer_selectedID);
						general_data.Selected_cloud->points.at(nowLayer_selectedID).r = 255;
						general_data.Selected_cloud->points.at(nowLayer_selectedID).g = 0;
						general_data.Selected_cloud->points.at(nowLayer_selectedID).b = 0;
						map_redSelected->insert(pair<int, PointXYZRGB>(nowLayer_selectedID, general_data.nowLayerCloud->points.at(nowLayer_selectedID)));
					}

					else if (key_data.keyBoard_alt && map_redSelected->find(nowLayer_selectedID) != map_redSelected->end())
					{
						general_data.Selected_cloud->points.at(nowLayer_selectedID).r = general_data.nowLayerCloud->points.at(nowLayer_selectedID).r;
						general_data.Selected_cloud->points.at(nowLayer_selectedID).g = general_data.nowLayerCloud->points.at(nowLayer_selectedID).g;
						general_data.Selected_cloud->points.at(nowLayer_selectedID).b = general_data.nowLayerCloud->points.at(nowLayer_selectedID).b;
						map_redSelected->erase(nowLayer_selectedID);
					}
				}
			}

			if (key_data.keyBoard_ctrl || key_data.keyBoard_alt)
			{
				general_data.SegClouds.clear();
				object_work::ViewCloudUpdate(general_data.Selected_cloud->makeShared(), false);
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
		if (!map_redSelected->empty()) {
			map_redSelected->clear();
			general_data.Selected_cloud->clear();
			general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();
			object_work::ViewCloudUpdate(general_data.nowLayerCloud->makeShared(), false);
		}
		return;
	}

	if (key_data.keyBoard_ctrl) {
		for (int i = 0; i < foundPointID.size(); ++i) {
			int nowLayer_selectedID = foundPointID[i];
			if (map_redSelected->find(nowLayer_selectedID) == map_redSelected->end())
			{
				map_redSelected->insert(pair<int, PointXYZRGB>(nowLayer_selectedID, general_data.nowLayerCloud->points.at(nowLayer_selectedID)));
			}
		}
	}
	else if (key_data.keyBoard_alt)
	{
		for (int i = 0; i < foundPointID.size(); ++i) {
			int nowLayer_selectedID = foundPointID[i];
			if (map_redSelected->find(nowLayer_selectedID) != map_redSelected->end())
			{
				map_redSelected->erase(nowLayer_selectedID);
			}
		}
	}
	else
	{
		map_redSelected->clear();
		general_data.Selected_cloud->clear();
		general_data.Selected_cloud->resize(general_data.nowLayerCloud->size());
		for (int i = 0; i < foundPointID.size(); ++i) {
			int nowLayer_selectedID = foundPointID[i];
			if (map_redSelected->find(nowLayer_selectedID) == map_redSelected->end()) {
				map_redSelected->insert(pair<int, PointXYZRGB>(nowLayer_selectedID, general_data.nowLayerCloud->points.at(nowLayer_selectedID)));
			}
		}
	}
	general_data.Selected_cloud = general_data.nowLayerCloud->makeShared();
	for (map<int, PointXYZRGB>::iterator iter = map_redSelected->begin(); iter != map_redSelected->end(); ++iter)
	{
		general_data.Selected_cloud->points.at(iter->first).r = 255;
		general_data.Selected_cloud->points.at(iter->first).g = 0;
		general_data.Selected_cloud->points.at(iter->first).b = 0;
	}
	general_data.SegClouds.clear();
	object_work::ViewCloudUpdate(general_data.Selected_cloud, false);
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