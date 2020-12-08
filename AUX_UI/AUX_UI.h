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
	//��L�ƥ�
	static void KeyBoard_eventController(const pcl::visualization::KeyboardEvent& event);
	//�ƹ��ƥ�
	static void cursor_BrushSelector(const pcl::visualization::MouseEvent& event);
	static void Area_PointCloud_Selector(const pcl::visualization::AreaPickingEvent& event);

private:	

public Q_SLOTS:	
	void Tree_UserSegmentation();
	void Tree_deleteLayer();
	void changeViewerColor(const QColor& c);
	void Brush_SizeChange();
	void Set_lightRange(const QColor& c);
	void Color_PreSegment();
	void Color_Segment();
	void reset_point_color();

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