#pragma once

#include "ui_AUX_UI.h" 
#include "object_work.h"
#include "ui_work.h"
#include <qobject.h>

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
	}
};

