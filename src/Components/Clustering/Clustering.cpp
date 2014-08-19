/*!
 * \file
 * \brief
 * \author Maciej
 */

#include <memory>
#include <string>

#include "Clustering.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


namespace Processors {
namespace Clustering {

Clustering::Clustering(const std::string & name) :
		Base::Component(name)  {

}

Clustering::~Clustering() {
}

void Clustering::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_segments", &out_segments);
	registerStream("out_colored", &out_colored);
	// Register handlers
	h_onNewData.setup(boost::bind(&Clustering::onNewData, this));
	registerHandler("onNewData", &h_onNewData);
	addDependency("onNewData", &in_cloud_xyzrgb);

}

bool Clustering::onInit() {

	return true;
}

bool Clustering::onFinish() {
	return true;
}

bool Clustering::onStop() {
	return true;
}

bool Clustering::onStart() {
	return true;
}

void Clustering::onNewData() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	CLOG(LINFO) << "PointCloud before filtering has: " << cloud->points.size() << " data points.";
	// Create the filtering object: downsample the dataset using a leaf size of 1cm

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.04); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(10000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	int j = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

		int r = rand()%128 + 128;
		int g = rand()%128 + 128;
		int b = rand()%128 + 128;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			pcl::PointXYZRGB pt = cloud->points[*pit];
			cloud_cluster->points.push_back(pt);
			pt.r = r; pt.g = g; pt.b = b;
			cloud_colored->push_back(pt);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_colored->width += cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_colored->height = 1;
		cloud_cluster->is_dense = true;
		cloud_colored->is_dense = true;

		CLOG(LINFO) << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points.";
		j++;

		out_segments.write(cloud_cluster);
	}
	out_colored.write(cloud_colored);

}



} //: namespace Clustering
} //: namespace Processors
