#pragma once

#include "common_data.h"
#include "ui_work.h"
#include <qobject.h>

//-----tool----
#include "QVTKWidget_controller/include/LayerControl.h"
static TreeLayerController* tree_layerController;

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
	void ImportCloud();
	void ExportCloud();

	void SetBrushMode();
	void SetAreaMode();
	void SetNoneMode();
};
