#include <CloudPoints_Tools.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

enum RGB_TYPE
{
	RGB, RBG, BGR, BRG, GRB, GBR
};

struct HSV {
	int h;
	int s;
	int v;
};
#include <qdebug.h>
HSV rgb2hsv(int r, int g, int b) {
	HSV hsv_data;
	int Max, Min;
	if (r >= g && r >= b) {
		Max = r;
		Min = (g >= b) ? b : g;
	}
	else if (g >= r && g >= b)
	{
		Max = g;
		Min = (r >= b) ? b : r;
	}
	else if (b >= r && b >= g) {
		Max = b;
		Min = (r >= g) ? g : r;
	}

	if ((Max - Min) <= 0)
	{
		hsv_data.h = -1;
		hsv_data.s = -1;
		hsv_data.v = -1;
		return hsv_data;
	}

	if (r == Max)
		hsv_data.h = (g - b) / (Max - Min);
	else if (g == Max)
		hsv_data.h = 2 + (b - r) / (Max - Min);
	else if (g == Max)
		hsv_data.h = 4 + (r - g) / (Max - Min);

	hsv_data.h *= 60;
	if (hsv_data.h < 0)
		hsv_data.h += 360;
	hsv_data.s = (Max - Min) / Max;
	hsv_data.v = Max;
	//qDebug() << hsv_data.h<< "," << hsv_data.s<<","<< hsv_data.v;
	return hsv_data;
}

vector<PointIndices> CloudPoints_Tools::CloudSegmentation(PointCloud<PointXYZRGB>::Ptr nowLayerCloud
	, int sliderValue, float nowCloud_avg_distance) {
	vector<PointCloud<PointXYZRGB>::Ptr> cluster_clouds_;
	vector<PointIndices> cluster_indice;
	//user set
	if (nowLayerCloud->size() > 700000)
		return cluster_indice;

	PointCloud<PointXYZRGB>::Ptr nowLayrCloudClone(new PointCloud<PointXYZRGB>);
	copyPointCloud(*nowLayerCloud, *nowLayrCloudClone);

	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
	tree->setInputCloud(nowLayrCloudClone);

	if (nowCloud_avg_distance == 0)
		return cluster_indice;

	EuclideanClusterExtraction<PointXYZRGB> ec;
	ec.setClusterTolerance(double(sliderValue * nowCloud_avg_distance * 0.01));
	//user set
	ec.setMinClusterSize(300);
	ec.setSearchMethod(tree);
	ec.setInputCloud(nowLayrCloudClone);
	ec.extract(cluster_indice);

	return cluster_indice;
}

vector<PointIndices> CloudPoints_Tools::CloudSegmentation_regionGrowing(PointCloud<PointXYZRGB>::Ptr nowLayerCloud,
	int sliderValue, float nowCloud_avg_distance) {
	search::KdTree<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(nowLayerCloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> region;
	region.setMinClusterSize(300);
	region.setSearchMethod(tree);
	region.setNumberOfNeighbours(sliderValue);
	region.setInputCloud(nowLayerCloud);
	region.setInputNormals(normals);
	region.setSmoothnessThreshold(20.0 / 180.0 * M_PI);
	region.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> cluster_indice;
	region.extract(cluster_indice);

	return cluster_indice;
}

vector<PointIndices> CloudPoints_Tools::CloudSegmentation_RGB(
	PointCloud<PointXYZRGB>::Ptr nowLayerCloud, int r, int g, int b) {
	//int type;
	HSV hsv_data = rgb2hsv(r, g, b);

	/*if (max == r && min == b)
		type = RGB_TYPE::RGB;
	else if (max == r && min == g)
		type = RGB_TYPE::RBG;
	else if (max == g && min == b)
		type = RGB_TYPE::GRB;
	else if (max == g && min == r)
		type = RGB_TYPE::GBR;
	else if (max == b && min == g)
		type = RGB_TYPE::BRG;
	else if (max == b && min == r)
		type = RGB_TYPE::BGR;*/

	PointCloud<PointXYZRGB>::Ptr nowLayrCloudClone(new PointCloud<PointXYZRGB>);
	copyPointCloud(*nowLayerCloud, *nowLayrCloudClone);
	vector<PointIndices> cluster_indice;
	PointIndices p;
	for (int i = 0; i < nowLayerCloud->size(); i++)
	{
		HSV point_hsv = rgb2hsv(nowLayerCloud->points[i].r, nowLayerCloud->points[i].g, nowLayerCloud->points[i].b);
		int range = 20;
		if (point_hsv.h == -1 || point_hsv.s == -1 || point_hsv.v == -1)
			continue;

		if (abs(hsv_data.h - point_hsv.h) < 5 &&
			abs(hsv_data.s - point_hsv.s) < 5 &&
			abs(hsv_data.v - point_hsv.v) < 200)
		{
			p.indices.push_back(i);
		}

		/*switch (type)
		{
		case RGB_TYPE::RGB:
			if ((nowLayerCloud->points[i].r >= nowLayerCloud->points[i].g) &&
				(nowLayerCloud->points[i].r >= nowLayerCloud->points[i].b) &&
				(nowLayerCloud->points[i].g >= nowLayerCloud->points[i].b))
				p.indices.push_back(i);
			break;
		case RGB_TYPE::RBG:
			if ((nowLayerCloud->points[i].r >= nowLayerCloud->points[i].g) &&
				(nowLayerCloud->points[i].r >= nowLayerCloud->points[i].b) &&
				(nowLayerCloud->points[i].b >= nowLayerCloud->points[i].g))
				p.indices.push_back(i);
			break;
		case RGB_TYPE::GRB:
			if ((nowLayerCloud->points[i].g >= nowLayerCloud->points[i].r) &&
				(nowLayerCloud->points[i].g >= nowLayerCloud->points[i].b) &&
				(nowLayerCloud->points[i].r >= nowLayerCloud->points[i].b))
				p.indices.push_back(i);
			break;
		case RGB_TYPE::GBR:
			if ((nowLayerCloud->points[i].g >= nowLayerCloud->points[i].r) &&
				(nowLayerCloud->points[i].g >= nowLayerCloud->points[i].b) &&
				(nowLayerCloud->points[i].b >= nowLayerCloud->points[i].r))
				p.indices.push_back(i);
			break;
		case RGB_TYPE::BRG:
			if ((nowLayerCloud->points[i].b >= nowLayerCloud->points[i].r) &&
				(nowLayerCloud->points[i].b >= nowLayerCloud->points[i].g) &&
				(nowLayerCloud->points[i].r >= nowLayerCloud->points[i].g))
				p.indices.push_back(i);
			break;
		case RGB_TYPE::BGR:
			if ((nowLayerCloud->points[i].b >= nowLayerCloud->points[i].r) &&
				(nowLayerCloud->points[i].b >= nowLayerCloud->points[i].g) &&
				(nowLayerCloud->points[i].g >= nowLayerCloud->points[i].r))
				p.indices.push_back(i);
			break;
		default:
			break;
		}*/
	}
	cluster_indice.push_back(p);
	return cluster_indice;
}

PointCloud<PointXYZRGB>::Ptr CloudPoints_Tools::CloudSmooth(PointCloud<PointXYZRGB>::Ptr nowLayerCloud, float smooth_strength) {
	PointCloud<PointXYZRGB>::Ptr smoothCloud(new PointCloud<PointXYZRGB>);
	MovingLeastSquares<PointXYZRGB, PointXYZRGB>::Ptr mls(new MovingLeastSquares<PointXYZRGB, PointXYZRGB>);
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);

	tree->setInputCloud(nowLayerCloud);
	mls->setComputeNormals(true);
	mls->setInputCloud(nowLayerCloud);
	mls->setSearchMethod(tree);
	mls->setSearchRadius(smooth_strength);
	mls->process(*smoothCloud);

	return smoothCloud;
}