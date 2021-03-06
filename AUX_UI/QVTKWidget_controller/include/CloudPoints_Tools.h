#pragma once

#include <vector>
#include "../../common_data.h"
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <qcolor.h>
using namespace pcl;
using namespace std;

//點雲處理工具，用於對點雲進行分割、平滑化、顏色變換等處理
class CloudPoints_Tools
{
public:
	CloudPoints_Tools() {}

	vector<PointIndices> CloudSegmentation(PointCloud<PointXYZRGB>::Ptr nowLayerCloud, int sliderValue, float nowCloud_avg_distance);
	vector<PointIndices> CloudSegmentation_regionGrowing(PointCloud<PointXYZRGB>::Ptr nowLayerCloud, int sliderValue, float nowCloud_avg_distance);
	vector<PointIndices> CloudSegmentation_RGB(PointCloud<PointXYZRGB>::Ptr nowLayerCloud, QColor base_rgb, int h_range,int v_range);
	PointCloud<PointXYZRGB>::Ptr CloudSmooth(PointCloud<PointXYZRGB>::Ptr nowLayerCloud, float smooth_strength);
	PointCloud<PointXYZRGB>::Ptr CloudDensity(PointCloud<PointXYZRGB>::Ptr nowLayerCloud, int leafValue, float nowCloud_avg_distance);
private:
};