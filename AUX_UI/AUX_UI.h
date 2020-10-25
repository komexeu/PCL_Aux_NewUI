#pragma once
#include <vtkSmartPointer.h>
#include "common_data.h"

#include <QtWidgets/QMainWindow>
#include "ui_AUX_UI.h" 

#include "my_foldGroupBox.h"
#include "my_button.h"
#include "my_toolButton.h"
#include "my_slider.h"
#include "my_spinBox.h"

#include <pcl/visualization/pcl_visualizer.h>
#include "InteractorStyle_override.h"
#include <vtkRenderWindow.h>

#include <qstandarditemmodel.h>


using namespace pcl;

class AUX_UI : public QMainWindow
{
	Q_OBJECT
		//features for control UI
public 	Q_SLOTS:
	void changeWindowsColor(const QColor& c);
	void SetAreaMode();
	void SetBrushMode();
	void SetNoneMode();
public:
	AUX_UI(QWidget* parent = Q_NULLPTR);
	//鍵盤事件
	static void KeyBoard_eventController(const pcl::visualization::KeyboardEvent& event);
	//滑鼠事件
	static void cursor_BrushSelector(const pcl::visualization::MouseEvent& event);

private:
	Ui::AUX_UIClass ui;
	QLabel* message;

	QToolBar* toolBar;
	static my_toolButton* Tool_Mode;
	my_spinBox* brush_spinbox;
	my_slider* brush_slider;
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
	void Tree_Slider_PreSegCloud();
	void Tree_confirmSegCloud();
	void Tree_deleteLayer();
	void changeViewerColor(const QColor& c);
public:
	void ViewCloudUpdate(PointCloud<PointXYZRGB>::Ptr updateCloud, bool resetCamera);
	void Init_Basedata();
	void Set_ToolConnect();
private:
	//-----pcl viewer------
	static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	InteractorStyle_override* sty_ovr;
	//----------Qt MVC------
	QStandardItemModel* standardModel;
	QItemSelectionModel* selectionModel;

private:
	double nowCloud_avg_distance;
	std::vector<PointCloud<PointXYZRGB>::Ptr> SegClouds;
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> AUX_UI::viewer;
my_toolButton* AUX_UI::Tool_Mode;