#pragma once

#include "ui_AUX_UI.h" 
#include "object_work.h"
#include "ui_work.h"
#include <qobject.h>
#include <qcolordialog.h>

extern Ui::AUX_UIClass ui;
extern my_UI::Obj_UIClass my_ui;

extern Data_Class::General_Data general_data;
extern Data_Class::Key_Data key_data;
extern Data_Class::PCL_Data pcl_data;
extern Data_Class::QT_Data qt_data;

#include <qdebug.h>
class ui_connect :public QObject
{
	Q_OBJECT
public:
	void Init_Ui_connect() {
		//-----------spinbox & slider connect------------
		QObject::connect(my_ui.brush_slider, SIGNAL(valueChanged(int)), my_ui.brush_spinbox, SLOT(setValue(int)));
		QObject::connect(my_ui.brush_spinbox, SIGNAL(valueChanged(int)), my_ui.brush_slider, SLOT(setValue(int)));
		QObject::connect(my_ui.smooth_slider, SIGNAL(valueChanged(int)), my_ui.smooth_spinbox, SLOT(setValue(int)));
		QObject::connect(my_ui.smooth_spinbox, SIGNAL(valueChanged(int)), my_ui.smooth_slider, SLOT(setValue(int)));
		QObject::connect(my_ui.preSeg_slider, SIGNAL(valueChanged(int)), my_ui.preSeg_spinbox, SLOT(setValue(int)));
		QObject::connect(my_ui.preSeg_spinbox, SIGNAL(valueChanged(int)), my_ui.preSeg_slider, SLOT(setValue(int)));
		//---------color segment------------
		QObject::connect(my_ui.H_range_slider, SIGNAL(valueChanged(int)), my_ui.H_range_spinbox, SLOT(setValue(int)));
		QObject::connect(my_ui.H_range_spinbox, SIGNAL(valueChanged(int)), my_ui.H_range_slider, SLOT(setValue(int)));
		QObject::connect(my_ui.V_range_slider, SIGNAL(valueChanged(int)), my_ui.V_range_spinbox, SLOT(setValue(int)));
		QObject::connect(my_ui.V_range_spinbox, SIGNAL(valueChanged(int)), my_ui.V_range_slider, SLOT(setValue(int)));
		//--------point density -----------
		QObject::connect(my_ui.leaf_slider, SIGNAL(valueChanged(int)), my_ui.leaf_spinbox, SLOT(setValue(int)));
		QObject::connect(my_ui.leaf_spinbox, SIGNAL(valueChanged(int)), my_ui.leaf_slider, SLOT(setValue(int)));
		//----------Mode Change------
		object_work* obw = new object_work();
		QObject::connect(my_ui.Brush, SIGNAL(clicked()), obw, SLOT(SetBrushMode()));
		QObject::connect(my_ui.Area, SIGNAL(clicked()), obw, SLOT(SetAreaMode()));
		QObject::connect(my_ui.Default, SIGNAL(clicked()), obw, SLOT(SetNoneMode()));
		//------------^UI^------------

		//----------IO pointcloud-----
		QObject::connect(my_ui.New_Pointcloud, SIGNAL(clicked()), obw, SLOT(ImportCloud()));
		QObject::connect(my_ui.Exprot_Pointcloud, SIGNAL(clicked()), obw, SLOT(ExportCloud()));
		QObject::connect(qt_data.selectionModel, SIGNAL(selectionChanged(const QItemSelection&, const QItemSelection&)), obw,
			SLOT(Tree_selectionChangedSlot(const QItemSelection&, const QItemSelection&)));
		//---------point density--------
		QObject::connect(my_ui.leaf_spinbox, SIGNAL(valueChanged(int)), obw, SLOT(voxelFilter()));
		QObject::connect(my_ui.pointDensity_start_button, SIGNAL(clicked()), obw, SLOT(VoxelWork()));
		//---------smooth---------------
		QObject::connect(my_ui.smooth_confirm, SIGNAL(clicked()), obw, SLOT(Tree_Smooth()));
		//------slider pre segmentation----
		QObject::connect(my_ui.preSeg_spinbox, SIGNAL(valueChanged(int)), obw, SLOT(Slider_PreSegCloud()));
		//confirm
		QObject::connect(my_ui.preSeg_confirm, SIGNAL(clicked()), obw, SLOT(confirm_colors_segment()));
		//--------color segment--------
		QColorDialog* Qcolordia_SegColor = new QColorDialog();
		QObject::connect(Qcolordia_SegColor, SIGNAL(colorSelected(const QColor&)), obw, SLOT(Set_lightRange(const QColor&)));
		QObject::connect(my_ui.color_widget, SIGNAL(clicked()), Qcolordia_SegColor, SLOT(open()));
		QObject::connect(my_ui.color_filter_start_button, SIGNAL(clicked()), obw, SLOT(confirm_colors_segment()));

		QObject::connect(my_ui.color_widget, SIGNAL(clicked()), obw, SLOT(reset_point_color()));
		QObject::connect(my_ui.V_range_spinbox, SIGNAL(valueChanged(int)), obw, SLOT(Color_PreSegment()));
		QObject::connect(my_ui.H_range_spinbox, SIGNAL(valueChanged(int)), obw, SLOT(Color_PreSegment()));
	}
};

