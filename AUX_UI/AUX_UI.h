#pragma once

#include "common_data.h"

#include <QtWidgets/QMainWindow>

#include "object_work.h"
#include "ui_work.h"
#include "ui_connect.h"

using namespace pcl;

class AUX_UI : public QMainWindow
{
	Q_OBJECT
	//features for control UI
public 	Q_SLOTS:
	void changeWindowsColor(const QColor& c);
	void SegMode_Change();
public:
	AUX_UI(QWidget* parent = Q_NULLPTR);
	//鍵盤事件
	static void KeyBoard_eventController(const pcl::visualization::KeyboardEvent& event);
	//滑鼠事件
	static void cursor_BrushSelector(const pcl::visualization::MouseEvent& event);
	static void Area_PointCloud_Selector(const pcl::visualization::AreaPickingEvent& event);

private:	

public Q_SLOTS:
	void Tree_UserSegmentation();
	void changeViewerColor(const QColor& c);
	void Brush_SizeChange();	

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