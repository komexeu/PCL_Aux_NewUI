#pragma once

#include "ui_AUX_UI.h" 
#include "Obj_UI.h"
#include "CycleProgram.h"

#include "InteractorStyle_override.h"
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>

#include <QtWidgets/QMainWindow>
#include <qlayout.h>

Ui::AUX_UIClass ui;
my_UI::Obj_UIClass my_ui;
Data_Class::QT_Data qt_data;

//this is for ui
class ui_work
{
public:
	void Init_UI(QMainWindow* w) {
		qt_data.standardModel = new QStandardItemModel(ui.treeView);
		ui.treeView->setModel(qt_data.standardModel);
		qt_data.selectionModel = ui.treeView->selectionModel();
		//------init progressbar---
		my_ui.cy_program = new CycleProgram(ui.dockWidgetContents_2);
		//------init tree view------
		ui.treeView->setFocusPolicy(Qt::NoFocus);
		ui.treeView->setContextMenuPolicy(Qt::CustomContextMenu);
		ui.treeView->setHeaderHidden(true);
		ui.treeView->expandAll();
		ui.treeView->setSelectionMode(QAbstractItemView::ExtendedSelection);
		//---------------------------------
		ColorScale::SetBaseColor(QColor(100, 100, 100));
		my_ui.message = new QLabel(ui.statusBar);
		ui.treeView->setStyleSheet(QString::fromUtf8(" background-color:  rgb(255,255,255);"));
		//--------top tool bar--------------
		my_ui.Top_toolBar = new QToolBar(w);
		my_ui.Top_toolBar->setMovable(false);
		my_ui.Top_toolBar->setObjectName(QString::fromUtf8("ToolBar"));
		my_ui.Top_toolBar->setAutoFillBackground(true);
		w->addToolBar(Qt::TopToolBarArea, my_ui.Top_toolBar);

		my_ui.Tool_Mode = new my_toolButton(my_ui.Top_toolBar, "Tool_Mode", "./my_source/NonMode.png");
		my_ui.Top_toolBar->addWidget(my_ui.Tool_Mode);

		my_ui.brush_spinbox = new my_spinBox(my_ui.Top_toolBar, "brush_spinbox");
		my_ui.brush_spinbox->setRange(1, 300);
		qt_data.brush_spinBoxAction = my_ui.Top_toolBar->addWidget(my_ui.brush_spinbox);
		qt_data.brush_spinBoxAction->setVisible(false);

		QLabel* sapceLable = new QLabel(NULL);
		my_ui.Top_toolBar->addWidget(sapceLable);

		my_ui.brush_slider = new my_slider(my_ui.Top_toolBar);
		my_ui.brush_slider->setRange(1, 300);
		my_ui.brush_slider->setMaximumWidth(80);
		qt_data.brush_sliderAction = my_ui.Top_toolBar->addWidget(my_ui.brush_slider);
		qt_data.brush_sliderAction->setVisible(false);

		QLabel* sapceLable_1 = new QLabel(NULL);
		my_ui.Top_toolBar->addWidget(sapceLable_1);

		my_ui.confirm_userSeg = new my_button(my_ui.Top_toolBar, QString::fromUtf8("Mark segment confirm"));
		my_ui.confirm_userSeg->set_font_color(QColor(255, 255, 255));
		my_ui.confirm_userSeg->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
		my_ui.Top_toolBar->addWidget(my_ui.confirm_userSeg);

		QWidget* SpaceExpand = new QWidget(my_ui.Top_toolBar);
		SpaceExpand->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
		my_ui.Top_toolBar->addWidget(SpaceExpand);

		my_ui.UI_Color_Style = new my_toolButton(my_ui.Top_toolBar, "Color Style", "./my_source/UI_ColorChange.png");
		my_ui.Top_toolBar->addWidget(my_ui.UI_Color_Style);

		my_ui.Viewer_Color_Style = new my_toolButton(my_ui.Top_toolBar, "pcl_data.viewer Color Style", "./my_source/color.jpg");
		my_ui.Top_toolBar->addWidget(my_ui.Viewer_Color_Style);
		//---------left tool bar---------
		my_ui.New_Pointcloud = new my_toolButton(ui.mainToolBar, "New Pointcloud", "./my_source/NewFile.png");
		ui.mainToolBar->addWidget(my_ui.New_Pointcloud);

		my_ui.Exprot_Pointcloud = new my_toolButton(ui.mainToolBar, "Pointcloud Export", "./my_source/export-icon.png");
		ui.mainToolBar->addWidget(my_ui.Exprot_Pointcloud);

		ui.mainToolBar->addSeparator();

		my_ui.Area = new my_toolButton(ui.mainToolBar, "Area", "./my_source/AreaSelect.png");
		ui.mainToolBar->addWidget(my_ui.Area);

		my_ui.Brush = new my_toolButton(ui.mainToolBar, "Brush", "./my_source/cursor1-2.png");
		ui.mainToolBar->addWidget(my_ui.Brush);

		my_ui.Default = new my_toolButton(ui.mainToolBar, "Default", "./my_source/NonMode.png");
		ui.mainToolBar->addWidget(my_ui.Default);

		QWidget* SpaceExpand_2 = new QWidget(my_ui.Top_toolBar);
		SpaceExpand_2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
		ui.mainToolBar->addWidget(SpaceExpand_2);

		my_ui.TrashCan = new my_toolButton(ui.mainToolBar, "TrashCan", "./my_source/images.png");
		ui.mainToolBar->addWidget(my_ui.TrashCan);
		//----groupbox(smooth)----
		my_ui.smooth_groupbox = new my_foldGroupBox("Smooth", ui.dockWidgetContents, my_foldGroupBox::STATE_EXPAND);

		my_ui.smooth_spinbox = new my_spinBox(my_ui.smooth_groupbox, "smooth_spinbox");
		my_ui.smooth_spinbox->setRange(1, 70);
		my_ui.smooth_groupbox->addWidget(0, QFormLayout::LabelRole, my_ui.smooth_spinbox);

		my_ui.smooth_slider = new my_slider(my_ui.smooth_groupbox);
		my_ui.smooth_slider->setRange(1, 70);
		my_ui.smooth_groupbox->addWidget(0, QFormLayout::FieldRole, my_ui.smooth_slider);

		my_ui.smooth_confirm = new my_button(my_ui.smooth_groupbox, QString::fromUtf8("confirm"));
		my_ui.smooth_confirm->set_font_color(QColor(255, 255, 255));
		my_ui.smooth_groupbox->addWidget(1, QFormLayout::SpanningRole, my_ui.smooth_confirm);
		//-----groupbox(preSegmentation)----
		my_ui.preSeg_groupbox = new my_foldGroupBox("PreSegment", ui.dockWidgetContents, my_foldGroupBox::STATE_EXPAND);

		//EuclideanClusterExtraction
		my_ui.SegMode_button = new my_button(my_ui.preSeg_groupbox, QString::fromUtf8("Euclidean"));
		my_ui.SegMode_button->set_font_color(QColor(255, 255, 255));
		my_ui.preSeg_groupbox->addWidget(0, QFormLayout::SpanningRole, my_ui.SegMode_button);
		//----
		my_ui.preSeg_spinbox = new my_spinBox(my_ui.smooth_groupbox, "preSeg_spinBox");
		my_ui.preSeg_spinbox->setRange(0, 500);
		my_ui.preSeg_groupbox->addWidget(1, QFormLayout::LabelRole, my_ui.preSeg_spinbox);

		my_ui.preSeg_slider = new my_slider(my_ui.preSeg_groupbox);
		my_ui.preSeg_slider->setRange(0, 500);
		my_ui.preSeg_groupbox->addWidget(1, QFormLayout::FieldRole, my_ui.preSeg_slider);

		my_ui.preSeg_confirm = new my_button(my_ui.preSeg_groupbox, QString::fromUtf8("confirm"));
		my_ui.preSeg_confirm->set_font_color(QColor(255, 255, 255));
		my_ui.preSeg_groupbox->addWidget(2, QFormLayout::SpanningRole, my_ui.preSeg_confirm);
		//----groupbox(Color Filter)----
		my_ui.color_filter_groupbox = new my_foldGroupBox("Color Filter", ui.dockWidgetContents, my_foldGroupBox::STATE_EXPAND);

		my_ui.color_widget = new QPushButton(my_ui.color_filter_groupbox);
		my_ui.color_widget->setFocusPolicy(Qt::NoFocus);
		my_ui.color_widget->setStyleSheet(QString("background-color:"
			"qlineargradient("
			"spread:"
			"pad, x1:0, y1:0.5,x2:1, y2:0.5,"
			"stop:0 rgb(%1, %2, %3),"
			"stop:0.5 rgb(%4, %5, %6),"
			"stop:1 rgb(%7, %8, %9));")
			.arg(QString::number(0), QString::number(0), QString::number(0),
				QString::number(128), QString::number(128), QString::number(128),
				QString::number(255), QString::number(255), QString::number(255)));
		my_ui.color_filter_groupbox->addWidget(0, QFormLayout::SpanningRole, my_ui.color_widget);

		QLabel* H_range_label = new QLabel("Hue Range", my_ui.color_filter_groupbox);
		QFont font;
		font.setFamily(QString::fromUtf8("Taipei Sans TC Beta"));
		H_range_label->setFont(font);
		my_ui.color_filter_groupbox->addWidget(1, QFormLayout::LabelRole, H_range_label);
		my_ui.H_range_spinbox = new my_spinBox(my_ui.color_filter_groupbox, "V_range_spinBox");
		my_ui.H_range_spinbox->setRange(0, 120);
		my_ui.color_filter_groupbox->addWidget(2, QFormLayout::LabelRole, my_ui.H_range_spinbox);
		my_ui.H_range_slider = new my_slider(my_ui.color_filter_groupbox);
		my_ui.H_range_slider->setRange(0, 120);
		my_ui.color_filter_groupbox->addWidget(2, QFormLayout::FieldRole, my_ui.H_range_slider);

		QLabel* V_range_label = new QLabel("Value Range", my_ui.color_filter_groupbox);
		V_range_label->setFont(font);
		my_ui.color_filter_groupbox->addWidget(3, QFormLayout::LabelRole, V_range_label);
		my_ui.V_range_spinbox = new my_spinBox(my_ui.color_filter_groupbox, "V_range_spinBox");
		my_ui.V_range_spinbox->setRange(1, 255);
		my_ui.color_filter_groupbox->addWidget(4, QFormLayout::LabelRole, my_ui.V_range_spinbox);
		my_ui.V_range_slider = new my_slider(my_ui.color_filter_groupbox);
		my_ui.V_range_slider->setRange(1, 255);
		my_ui.color_filter_groupbox->addWidget(4, QFormLayout::FieldRole, my_ui.V_range_slider);

		my_ui.color_filter_start_button = new my_button(my_ui.color_filter_groupbox, QString::fromUtf8("Start"));
		my_ui.color_filter_start_button->set_font_color(QColor(255, 255, 255));
		my_ui.color_filter_groupbox->addWidget(5, QFormLayout::SpanningRole, my_ui.color_filter_start_button);
		//--------point density groupbox-----------
		my_ui.pointDensity_groupbox = new my_foldGroupBox("Point Density", ui.dockWidgetContents, my_foldGroupBox::STATE_EXPAND);
		my_ui.leaf_spinbox = new my_spinBox(my_ui.pointDensity_groupbox, "leaf_spinBox");
		my_ui.leaf_spinbox->setRange(1, 40);
		my_ui.pointDensity_groupbox->addWidget(0, QFormLayout::LabelRole, my_ui.leaf_spinbox);
		my_ui.leaf_slider = new my_slider(my_ui.color_filter_groupbox);
		my_ui.leaf_slider->setRange(1, 40);
		my_ui.pointDensity_groupbox->addWidget(0, QFormLayout::FieldRole, my_ui.leaf_slider);

		my_ui.pointDensity_start_button = new my_button(my_ui.pointDensity_groupbox, QString::fromUtf8("Start"));
		my_ui.pointDensity_start_button->set_font_color(QColor(255, 255, 255));
		my_ui.pointDensity_groupbox->addWidget(1, QFormLayout::SpanningRole, my_ui.pointDensity_start_button);
		//---------
		ui.formLayout->setWidget(ui.formLayout->count() + 1, QFormLayout::FieldRole, my_ui.smooth_groupbox);
		ui.formLayout->setWidget(ui.formLayout->count() + 1, QFormLayout::FieldRole, my_ui.preSeg_groupbox);
		ui.formLayout->setWidget(ui.formLayout->count() + 1, QFormLayout::FieldRole, my_ui.color_filter_groupbox);
		ui.formLayout->setWidget(ui.formLayout->count() + 1, QFormLayout::FieldRole, my_ui.pointDensity_groupbox);
	}
};