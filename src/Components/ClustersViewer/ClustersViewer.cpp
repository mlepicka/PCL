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
    registerStream("in_projections", &in_projections);
	// Register handlers
    registerHandler("on_clouds", boost::bind(&ClustersViewer::on_clouds, this));
	addDependency("on_clouds", &in_clouds);
    registerHandler("on_projections", boost::bind(&ClustersViewer::on_projections, this));
    addDependency("on_projections", &in_projections);
	
	// Register spin handler.
    registerHandler("on_spin", boost::bind(&ClustersViewer::on_spin, this));
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
		viewer->addCoordinateSystem ();
#endif
	// TODO: Currently only 1.7.1 is available in the 012/031 laboratories.
	// TODO: Fix for other versions of PCL.
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
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = in_clouds.read();
	
//    unsigned char colors[ 10 ][ 3 ] = {
//        { 255, 255, 255 },
//        { 255, 0, 0 },
//        { 0, 255, 0 },
//        { 0, 255, 255 },
//        { 255, 255, 0 },
//        { 255, 0, 255 },
//        { 255, 128, 0 },
//        { 128, 0, 255 },
//        { 0, 0, 255 },
//        { 128, 128, 128 }
//    };
	
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
            LOG(LTRACE) << "addPointCloud "<< i <<endl;
		}
	else if (clouds.size()<count)
		for(int i = clouds.size(); i < count; i++){
			char id = '0' + i;
			viewer->removePointCloud(std::string("cloud_xyz") + id);
            LOG(LTRACE) << "removePointCloud "<< i <<endl;
		}
		
	count = clouds.size();
	if (count>10)
		count = 10;
	

	for(int i = 0; i < count; i++){
		char id = '0' + i;
		viewer->updatePointCloud(clouds[i],std::string("cloud_xyz") + id);	
        LOG(LTRACE) << "updatePointCloud "<< i <<endl;
	}
	
}

void ClustersViewer::on_projections() {
    LOG(LTRACE) << "ClustersViewer::on_projections";
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> projections = in_projections.read();
    viewer->removeAllShapes();

    for(int i = 0; i < projections.size() && i < 10; i++){
        LOG(LTRACE) << "draw projection "<< i <<endl;
        char id = '0' + i;

        viewer->addLine(projections[i]->at(0), projections[i]->at(1), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("01"));
        viewer->addLine(projections[i]->at(1), projections[i]->at(2), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("12"));
        viewer->addLine(projections[i]->at(2), projections[i]->at(3), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("23"));
        viewer->addLine(projections[i]->at(3), projections[i]->at(0), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("30"));
        viewer->addLine(projections[i]->at(0), projections[i]->at(4), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("04"));
        viewer->addLine(projections[i]->at(1), projections[i]->at(5), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("15"));
        viewer->addLine(projections[i]->at(2), projections[i]->at(6), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("26"));
        viewer->addLine(projections[i]->at(3), projections[i]->at(7), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("37"));
        viewer->addLine(projections[i]->at(4), projections[i]->at(5), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("45"));
        viewer->addLine(projections[i]->at(5), projections[i]->at(6), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("56"));
        viewer->addLine(projections[i]->at(6), projections[i]->at(7), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("67"));
        viewer->addLine(projections[i]->at(7), projections[i]->at(4), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("74"));
    }
}

void ClustersViewer::on_spin() {
	viewer->spinOnce (100);
}



} //: namespace ClustersViewer
} //: namespace Processors
