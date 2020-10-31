#pragma once

#include "common_data.h"
#include "ui_AUX_UI.h" 

#include <QtWidgets/QMainWindow>
#include <qstandarditemmodel.h>

#include "my_Widgets/include/my_foldGroupBox.h"
#include "my_Widgets/include/my_button.h"
#include "my_Widgets/include/my_toolButton.h"
#include "my_Widgets/include/my_slider.h"
#include "my_Widgets/include/my_spinBox.h"

#include <pcl/visualization/pcl_visualizer.h>
#include "InteractorStyle_override.h"
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>



using namespace pcl;

class AUX_UI : public QMainWindow
{
	Q_OBJECT
		//features for control UI
public 	Q_SLOTS:
	void changeWindowsColor(const QColor& c);
	static void SetAreaMode();
	static void SetBrushMode();
	static void SetNoneMode();
public:
	AUX_UI(QWidget* parent = Q_NULLPTR);
	//鍵盤事件
	static void KeyBoard_eventController(const pcl::visualization::KeyboardEvent& event);
	//滑鼠事件
	static void cursor_BrushSelector(const pcl::visualization::MouseEvent& event);
	static void Area_PointCloud_Selector(const pcl::visualization::AreaPickingEvent& event);

private:
	static Ui::AUX_UIClass ui;
	QLabel* message;

	static QToolBar* Top_toolBar;
	static my_toolButton* Tool_Mode;
	static my_spinBox* brush_spinbox;
	static my_slider* brush_slider;
	my_button* confirm_userSeg;
	my_toolButton* UI_Color_Style;
	my_toolButton* Viewer_Color_Style;

	my_toolButton* New_Pointcloud;
	my_toolButton* Exprot_Pointcloud;
	my_toolButton* Area;
	my_toolButton* Brush;
	my_toolButton* Default;
	my_toolButton* TrashCan;

	my_foldGroupBox* smooth_groupbox;
	my_spinBox* smooth_spinbox;
	my_slider* smooth_slider;
	my_button* smooth_confirm;

	my_foldGroupBox* preSeg_groupbox;
	my_spinBox* preSeg_spinbox;
	my_slider* preSeg_slider;
	my_button* preSeg_confirm;
	QStandardItem* item;

	//features for UI control PCL 
public Q_SLOTS:
	void Tree_importCloud();
	void Tree_selectionChangedSlot(const QItemSelection&, const QItemSelection&);
	void Tree_Smooth();
	void Slider_PreSegCloud();
	void Slider_confirmSegCloud();
	void Tree_UserSegmentation();
	void Tree_deleteLayer();
	void changeViewerColor(const QColor& c);
	void  Brush_change();
public:
	static void WhiteCursorUpdate();
	static void ViewCloudUpdate(PointCloud<PointXYZRGB>::Ptr updateCloud, bool resetCamera);
	void RedSelectClear();
	static void initModes();

	void Init_Basedata();
	void Set_ToolConnect();
	QModelIndex  searchParent(QModelIndex index);
private:
	//-----pcl viewer------
	static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	static InteractorStyle_override* my_interactorStyle;
	//----------Qt MVC------
	static QStandardItemModel* standardModel;
	static QItemSelectionModel* selectionModel;
	//database
private:
	static double nowCloud_avg_distance;
	std::vector<PointCloud<PointXYZRGB>::Ptr> SegClouds;
	//已選點雲
	static 	PointCloud<PointXYZRGB>::Ptr Selected_cloud;
private:
	static float brush_radius;
	static bool keyBoard_ctrl;
	static bool keyBoard_alt;

	static PointCloud<PointXYZRGB>::Ptr nowLayerCloud;

	static QAction* brush_spinBoxAction;
	static QAction* brush_sliderAction;
};
Ui::AUX_UIClass AUX_UI::ui;

QStandardItemModel* AUX_UI::standardModel;
QItemSelectionModel* AUX_UI::selectionModel;

boost::shared_ptr<pcl::visualization::PCLVisualizer> AUX_UI::viewer;
InteractorStyle_override* AUX_UI::my_interactorStyle;
my_toolButton* AUX_UI::Tool_Mode;

QToolBar* AUX_UI::Top_toolBar;
my_spinBox* AUX_UI::brush_spinbox;
my_slider* AUX_UI::brush_slider;

double AUX_UI::nowCloud_avg_distance;
float AUX_UI::brush_radius;
bool AUX_UI::keyBoard_ctrl;
bool AUX_UI::keyBoard_alt;

PointCloud<PointXYZRGB>::Ptr AUX_UI::nowLayerCloud;

//已選點雲
PointCloud<PointXYZRGB>::Ptr AUX_UI::Selected_cloud;
static std::map<int, PointXYZRGB> select_map;

 QAction* AUX_UI::brush_spinBoxAction;
 QAction* AUX_UI::brush_sliderAction;