#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <qcolor.h>

using namespace pcl;

struct complax_cloudInformation
{
	pcl::PointCloud<PointXYZRGB>::Ptr cloud_data;
	std::vector<int> points_id;
};

enum class SelectMode :int
{
	NO_SELECT_MODE,
	BRUSH_SELECT_MODE,
	AREA_SELECT_MODE
};
static SelectMode GLOBAL_SELECTMODE = SelectMode::NO_SELECT_MODE;

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
