#pragma once
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindowInteractor.h>
#include <qdebug.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/area_picking_event.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/mouse_event.h>
#include <vtkInteractorStyleRubberBandPick.h>

#include <vtkPointPicker.h>
#include <vtkAreaPicker.h>

class InteractorStyle_override :public pcl::visualization::PCLVisualizerInteractorStyle
{
public:
	static InteractorStyle_override* New();
	vtkTypeMacro(InteractorStyle_override, PCLVisualizerInteractorStyle);

	InteractorStyle_override() {
		point_picker_ = vtkSmartPointer<vtkPointPicker>::New();
		PCLVisualizerInteractorStyle();
	}

	void SetCurrentMode_AreaPick(bool mode) {
		CurrentMode = mode;
		if (CurrentMode)
		{
			vtkSmartPointer<vtkAreaPicker> area_picker = vtkSmartPointer<vtkAreaPicker>::New();
			Interactor->SetPicker(area_picker);
		}
		else {

			vtkSmartPointer<vtkPointPicker> point_picker = vtkSmartPointer<vtkPointPicker>::New();;
			Interactor->SetPicker(point_picker);
		}
	}

	void OnChar() override {
		FindPokedRenderer(Interactor->GetEventPosition()[0], Interactor->GetEventPosition()[1]);

		switch (Interactor->GetKeyCode()) {
		case 'r': case 'R':
		case 'x': case 'X':
		case 'e': case 'E':
		case 'b': case 'B': {
			break;
		}
		default:
		{
			//Superclass::OnChar();
			break;
		}
		}
	}

	// Keyboard events
	void OnKeyDown() override {
		switch (Interactor->GetKeyCode()) {
		case 'x':case'X': {
			CurrentMode = !CurrentMode;
			if (CurrentMode)
			{
				point_picker_ = static_cast<vtkPointPicker*> (Interactor->GetPicker());
				vtkSmartPointer<vtkAreaPicker> area_picker = vtkSmartPointer<vtkAreaPicker>::New();
				Interactor->SetPicker(area_picker);
			}
			else
			{
				Interactor->SetPicker(point_picker_);
			}
			break;
		}
		case 'b':case'B': {
			CurrentMode = 0;
			Interactor->SetPicker(point_picker_);
			break;
		}
		case 'r':case'R': {

			break;
		}
		default:
		{
			//CurrentMode = 0;
			break;
		}
		}

		pcl::visualization::KeyboardEvent event(true, Interactor->GetKeySym(),
			Interactor->GetKeyCode(), Interactor->GetAltKey(), Interactor->GetControlKey(), Interactor->GetShiftKey());

		qDebug() << event.getKeySym().c_str();
		keyboard_signal_(event);
	}

	void
		OnKeyUp() override {
		Superclass::OnKeyUp();
	}

	// mouse button events
	void
		OnMouseMove() override {
		//qDebug() << "mouse MOVE.";
		Superclass::OnMouseMove();
	}

	void
		OnLeftButtonDown() override {
		Superclass::OnLeftButtonDown();
	}

	void
		OnLeftButtonUp() override {
		Superclass::OnLeftButtonUp();
	}

	void
		OnMiddleButtonDown() override {
		Superclass::OnMiddleButtonDown();
	}

	void
		OnMiddleButtonUp() override { Superclass::OnMiddleButtonUp(); }
	void
		OnRightButtonDown() override { Superclass::OnRightButtonDown(); }
	void
		OnRightButtonUp() override { Superclass::OnRightButtonUp(); }
	void
		OnMouseWheelForward() override { Superclass::OnMouseWheelForward(); }
	void
		OnMouseWheelBackward() override { Superclass::OnMouseWheelBackward(); }

};

vtkStandardNewMacro(InteractorStyle_override);