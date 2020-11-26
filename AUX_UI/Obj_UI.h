#pragma once
#include "my_Widgets/include/my_foldGroupBox.h"
#include "my_Widgets/include/my_button.h"
#include "my_Widgets/include/my_toolButton.h"
#include "my_Widgets/include/my_slider.h"
#include "my_Widgets/include/my_spinBox.h"

class Obj_UI_Class {
public:
	QLabel* message;

	QToolBar* Top_toolBar;
	my_toolButton* Tool_Mode;
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
	my_button* SegMode_button;
	my_spinBox* preSeg_spinbox;
	my_slider* preSeg_slider;
	my_button* preSeg_confirm;

	my_foldGroupBox* color_filter_groupbox;
	my_button* color_filter_start_button;
};

namespace my_UI {
	class Obj_UIClass :public Obj_UI_Class {};
}