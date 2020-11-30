#include <CloudPoints_Tools.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

#include <qdebug.h>

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
	PointCloud<PointXYZRGB>::Ptr nowLayerCloud, QColor base_rgb, int h_range, int v_range) {
	
	PointCloud<PointXYZRGB>::Ptr nowLayrCloudClone(new PointCloud<PointXYZRGB>);
	copyPointCloud(*nowLayerCloud, *nowLayrCloudClone);
	vector<PointIndices> cluster_indice;
	PointIndices p;
	for (int i = 0; i < nowLayerCloud->size(); i++)
	{
		QColor point_hsv{ nowLayerCloud->points[i].r, nowLayerCloud->points[i].g, nowLayerCloud->points[i].b };

		if (abs(base_rgb.hue() - point_hsv.hue()) < h_range &&
			abs(base_rgb.saturation() - point_hsv.saturation()) < 255 &&
			abs(base_rgb.value() - point_hsv.value()) < v_range)
			p.indices.push_back(i);
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