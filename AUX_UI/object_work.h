#pragma once

#include "common_data.h"
#include "ui_work.h"
#include <qobject.h>

//-----tool----
#include "QVTKWidget_controller/include/LayerControl.h"
TreeLayerController* tree_layerController;
static std::map<int, PointXYZRGB> select_map;

Data_Class::General_Data general_data;
Data_Class::Key_Data key_data;
Data_Class::PCL_Data pcl_data;

extern Ui::AUX_UIClass ui;
extern my_UI::Obj_UIClass my_ui;
extern Data_Class::QT_Data qt_data;

#include <qdebug.h>
class object_work :public QObject
{
	Q_OBJECT
public Q_SLOTS:
	void ViewCloudUpdate(PointCloud<PointXYZRGB>::Ptr updateCloud, bool resetCamera);
	void RedSelectClear();
	void reset_point_color();
	void Tree_selectionChangedSlot(const QItemSelection&, const QItemSelection&);

	void ImportCloud();
	void ExportCloud();

	void voxelFilter();
	void VoxelWork();

	void Tree_Smooth();
	void Slider_PreSegCloud();
	void confirm_colors_segment();

	void Set_lightRange(const QColor& c);
	void Color_PreSegment();

	void SetBrushMode();
	void SetAreaMode();
	void SetNoneMode();
};
