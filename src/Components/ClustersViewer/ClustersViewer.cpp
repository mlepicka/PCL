/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "ClustersViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ClustersViewer {

ClustersViewer::ClustersViewer(const std::string & name) :
		Base::Component(name),
		title("title", std::string("ClustersViewer")),
		prop_coordinate_system("coordinate_system", true)
{
			registerProperty(title);
			registerProperty(prop_coordinate_system);
}

ClustersViewer::~ClustersViewer() {
}

void ClustersViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_clouds", &in_clouds);
	// Register handlers
	h_on_clouds.setup(boost::bind(&ClustersViewer::on_clouds, this));
	registerHandler("on_clouds", &h_on_clouds);
	addDependency("on_clouds", &in_clouds);
	
	// Register spin handler.
	h_on_spin.setup(boost::bind(&ClustersViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);

}

bool ClustersViewer::onInit() {
	LOG(LTRACE) << "ClustersViewer::onInit";
	// Create visualizer.
	viewer = new pcl::visualization::PCLVisualizer (title);
	viewer->initCameraParameters ();
	viewer->setBackgroundColor (0, 0, 0);
	// Add visible coortinate system.
	if(prop_coordinate_system) {
#if PCL_VERSION_COMPARE(>=,1,7,1)
	    viewer->addCoordinateSystem (1.0, "ClustersViewer", 0);
#else
	    viewer->addCoordinateSystem (1.0);
#endif
	}
	count = 0;
 	return true;
}

bool ClustersViewer::onFinish() {
	return true;
}

bool ClustersViewer::onStop() {
	return true;
}

bool ClustersViewer::onStart() {
	return true;
}

void ClustersViewer::on_clouds() {
	LOG(LTRACE) << "ClustersViewer::on_clouds";
	cout << "ClustersViewer::on_clouds"<<endl;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = in_clouds.read();
	
	unsigned char colors[ 10 ][ 3 ] = {
        { 255, 255, 255 },
		{ 255, 0, 0 },
		{ 0, 255, 0 },
		{ 0, 255, 255 },
		{ 255, 255, 0 },
		{ 255, 0, 255 },
		{ 255, 128, 0 },
		{ 128, 0, 255 },
		{ 0, 0, 255 },
		{ 128, 128, 128 }
    };
	
	if (clouds.size()>count)
		for(int i = count; i < clouds.size() && i<10; i++){
			char id = '0' + i;
			viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), std::string("cloud_xyz") + id);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, std::string("cloud_xyz") + id);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 
														colors[i][0],
														colors[i][1],
														colors[i][2],
														std::string("cloud_xyz") + id); 
			cout << "addPointCloud "<< i <<endl;
		}
	else if (clouds.size()<count)
		for(int i = clouds.size(); i < count; i++){
			char id = '0' + i;
			viewer->removePointCloud(std::string("cloud_xyz") + id);
			cout << "removePointCloud "<< i <<endl;
		}
		
	count = clouds.size();
	if (count>10)
		count = 10;
	

	for(int i = 0; i < count; i++){
		char id = '0' + i;
		viewer->updatePointCloud(clouds[i],std::string("cloud_xyz") + id);	
		cout << "updatePointCloud "<< i <<endl;
	}
	
}

void ClustersViewer::on_spin() {
	viewer->spinOnce (100);
}



} //: namespace ClustersViewer
} //: namespace Processors
