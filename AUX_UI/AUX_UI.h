#pragma once

#include "common_data.h"
#include "ui_AUX_UI.h" 
#include "Obj_UI.h"

#include <QtWidgets/QMainWindow>
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

	void SegMode_Change();
public:
	AUX_UI(QWidget* parent = Q_NULLPTR);
	//鍵盤事件
	static void KeyBoard_eventController(const pcl::visualization::KeyboardEvent& event);
	//滑鼠事件
	static void cursor_BrushSelector(const pcl::visualization::MouseEvent& event);
	static void Area_PointCloud_Selector(const pcl::visualization::AreaPickingEvent& event);

private:
	static Ui::AUX_UIClass ui;
	static my_UI::Obj_UIClass my_ui;
	static Data_Class::General_Data general_data;
	static Data_Class::Key_Data key_data;
	static Data_Class::PCL_Data pcl_data;
	static Data_Class::QT_Data qt_data;

public Q_SLOTS:
	void Tree_importCloud();
	void ExportCloud();
	void Tree_selectionChangedSlot(const QItemSelection&, const QItemSelection&);
	void Tree_Smooth();
	void Slider_PreSegCloud();
	void confirm_colors_segment();
	void Tree_UserSegmentation();
	void Tree_deleteLayer();
	void changeViewerColor(const QColor& c);
	void Brush_SizeChange();
	void Set_lightRange(const QColor& c);
	void Color_PreSegment();
	void Color_Segment();
	void reset_point_color();
	void voxelFilter();

	void onCustomContextMenu(const QPoint& point);
	void mergeLayer();
public:
	//if true only delete white cursor,false for update position of white cursor.
	static void WhiteCursorUpdate(bool whiteCursor_clear);
	static void ViewCloudUpdate(PointCloud<PointXYZRGB>::Ptr updateCloud, bool resetCamera);
	void RedSelectClear();
	static void initModes();

	void Init_Basedata();
	void Set_ToolConnect();
	QModelIndex  searchParent(QModelIndex index);
};
Ui::AUX_UIClass AUX_UI::ui;
my_UI::Obj_UIClass AUX_UI::my_ui;
Data_Class::General_Data AUX_UI::general_data;
Data_Class::Key_Data AUX_UI::key_data;
Data_Class::PCL_Data AUX_UI::pcl_data;
Data_Class::QT_Data AUX_UI::qt_data;

static std::map<int, PointXYZRGB> select_map;