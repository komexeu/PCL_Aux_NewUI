#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <qcolor.h>

using namespace pcl;

enum class SelectMode :int
{
	NO_SELECT_MODE,
	BRUSH_SELECT_MODE,
	AREA_SELECT_MODE
};
static SelectMode GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;

enum class SegmentMode :int
{
	EUCLIDEAN_CLUSTER_EXTRACTION,
	REGION_GROWING
};
static SegmentMode GLOBAL_SEGMENTMODE = SegmentMode::EUCLIDEAN_CLUSTER_EXTRACTION;

struct HSV {
	int h;
	int s;
	int v;
};
HSV rgb2hsv(int r, int g, int b) {
	HSV hsv_data;
	int Max = std::max(r, std::max(g, b));
	int Min = std::min(r, std::min(g, b));
	int delta = Max - Min;

	if (Max == Min)
		hsv_data.h = 0;
	else if (r == Max) {
		if (g > b)
			hsv_data.h = 60 * ((g - b) / delta) + 0;
		else
			hsv_data.h = 60 * ((g - b) / delta) + 360;
	}
	else if (g == Max)
		hsv_data.h = 60 * ((b - r) / delta) + 120;
	else if (b == Max)
		hsv_data.h = 60 * ((r - g) / delta) + 240;

	if (Max == 0)
		hsv_data.s = 0;
	else
		hsv_data.s = delta * 255 / Max;

	hsv_data.v = Max;
	return hsv_data;
}
struct M_RGB {
	int r;
	int g;
	int b;
};
M_RGB hsv2rgb(int h, int s, int v) {
	M_RGB rgb;
	if (s == 0)
	{
		rgb = M_RGB{ v,v,v };
		return rgb;
	}

	float hi, f, p, q, t;
	s /= 255;
	hi = (h / 60) % 6;
	f = (h / 60) - hi;
	p = v * (1 - s);
	q = v * (1 - f * s);
	t = v * (1 - (1 - f) * s);

	switch ((int)hi)
	{
	case 0:
		rgb.r = v; rgb.g = t ; rgb.b = p ;
		break;
	case 1:
		rgb.r = q ; rgb.g = v; rgb.b = p ;
		break;
	case 2:
		rgb.r = p ; rgb.g = v; rgb.b = t ;
		break;
	case 3:
		rgb.r = p ; rgb.g = q ; rgb.b = v;
		break;
	case 4:
		rgb.r = t ; rgb.g = p ; rgb.b = v;
		break;
	case 5:
		rgb.r = v; rgb.g = p ; rgb.b = q ;
		break;
	default:
		rgb.r = 0; rgb.g = 0; rgb.b = 0;
		break;
	}
	return rgb;
}

namespace ColorScale {
	struct Color_5Level {
		QColor colorE;
		QColor colorD;
		QColor colorC;
		QColor colorB;
		QColor colorA;
	};

	static Color_5Level Color_struct;

	static void SetBaseColor(const QColor& c) {
		QColor BaseColor = c;
		int baseR = BaseColor.red();
		int baseIR = (255 - baseR) * 0.2;

		int baseG = BaseColor.green();
		int baseIG = (255 - baseG) * 0.2;

		int baseB = BaseColor.blue();
		int baseIB = (255 - baseB) * 0.2;

		Color_struct.colorE = QColor(baseR + baseIR * 2, baseG + baseIG * 2, baseB + baseIB * 2);
		Color_struct.colorD = QColor(baseR + baseIR, baseG + baseIG, baseB + baseIB);
		Color_struct.colorC = BaseColor;
		Color_struct.colorB = QColor(
			(baseR - baseIR) < 0 ? 0 : (baseR - baseIR),
			(baseG - baseIG) < 0 ? 0 : (baseG - baseIG),
			(baseB - baseIB) < 0 ? 0 : (baseB - baseIB));
		Color_struct.colorA = QColor(
			(baseR - baseIR * 2) < 0 ? 0 : (baseR - baseIR * 2),
			(baseG - baseIG * 2) < 0 ? 0 : (baseG - baseIG * 2),
			(baseB - baseIB * 2) < 0 ? 0 : (baseB - baseIB * 2));
	}
}

#include <qstandarditemmodel.h>
#include <qitemselectionmodel.h>
#include <qaction.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "InteractorStyle_override.h"
class QT_DataClass {
public:
	QAction* brush_spinBoxAction;
	QAction* brush_sliderAction;

	QStandardItemModel* standardModel;
	QItemSelectionModel* selectionModel;
};
class Key_DataClass {
public:
	bool keyBoard_ctrl;
	bool keyBoard_alt;
};
class PCL_DataClass {
public:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	InteractorStyle_override* my_interactorStyle;
};
class General_DataClass {
public:
	double nowCloud_avg_distance;
	float brush_radius;
	PointCloud<PointXYZRGB>::Ptr nowLayerCloud;
	std::vector<PointCloud<PointXYZRGB>::Ptr> SegClouds;
	PointCloud<PointXYZRGB>::Ptr Selected_cloud;

	M_RGB rgb_data;
};
namespace Data_Class {
	class QT_Data :public QT_DataClass {};
	class Key_Data :public Key_DataClass {};
	class PCL_Data :public PCL_DataClass {};
	class General_Data :public General_DataClass {};
}