#include "object_work.h"

void object_work::SetBrushMode() {
	qt_data.brush_sliderAction->setVisible(true);
	qt_data.brush_spinBoxAction->setVisible(true);

	my_ui.brush_spinbox->setValue(general_data.brush_radius);

	QIcon the_icon;
	the_icon.addFile("./my_source/cursor1-2.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::BRUSH_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(0);
	//WhiteCursorUpdate(false);
}

void object_work::SetAreaMode() {
	qt_data.brush_sliderAction->setVisible(false);
	qt_data.brush_spinBoxAction->setVisible(false);

	QIcon the_icon;
	the_icon.addFile("./my_source/AreaSelect.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::AREA_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(1);
	//WhiteCursorUpdate(true);
}

void object_work::SetNoneMode() {

	qt_data.brush_sliderAction->setVisible(false);
	qt_data.brush_spinBoxAction->setVisible(false);

	QIcon the_icon;
	the_icon.addFile("./my_source/NonMode.png", QSize(), QIcon::Normal, QIcon::Off);
	my_ui.Tool_Mode->setIcon(the_icon);
	GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;
	pcl_data.my_interactorStyle->SetCurrentMode_AreaPick(0);
	//WhiteCursorUpdate(true);
}