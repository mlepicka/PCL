/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#include <memory>
#include <string>

#include "CloudViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/filter.h>

namespace Processors {
namespace CloudViewer {

CloudViewer::CloudViewer(const std::string & name) :
		Base::Component(name),
		prop_window_name("window_name",std::string("3D PC Viewer")),
		prop_coordinate_system("coordinate_system",boost::bind(&CloudViewer::onCSShowClick, this, _2), true),
		prop_background_color("background_color", boost::bind(&CloudViewer::onBackgroundColorChange, this, _2), std::string("0,0,0")),
		normals_scale("normals.scale", 0.1),
		normals_level("normals.level", 1),
		prop_sift_size("sift.size", 1),
		prop_sift_color("sift.color", boost::bind(&CloudViewer::onSIFTColorChange, this, _2), std::string("255,0,0"))

{
	registerProperty(prop_window_name);
	registerProperty(prop_coordinate_system);
	registerProperty(prop_background_color);

	registerProperty(prop_sift_size);
	registerProperty(prop_sift_color);

	registerProperty(normals_scale);
	registerProperty(normals_level);

}

void CloudViewer::onCSShowClick(const bool & new_show_cs_) {
	CLOG(LDEBUG) << "CloudViewer::onCSShowClick show=" << new_show_cs_;
    if (!viewer)
    	return;

	if (new_show_cs_) {
#if PCL_VERSION_COMPARE(>=,1,7,1)
		viewer->addCoordinateSystem(1.0, "CloudViewer", 0);
#else
		viewer->addCoordinateSystem (1.0);
#endif
	} else {
#if PCL_VERSION_COMPARE(>=,1,7,2)
		viewer->removeCoordinateSystem ("CloudViewer", 0);
#elif PCL_VERSION_COMPARE(>=,1,7,1)
		viewer->removeCoordinateSystem("CloudViewer");
#else
		viewer->removeCoordinateSystem (1.0);
#endif
	}

	prop_coordinate_system = new_show_cs_;
}

void CloudViewer::onBackgroundColorChange(std::string color_) {
	CLOG(LDEBUG) << "CloudViewer::onBackgroundColorChange color=" << color_;
	try {
		// Parse string.
		vector<std::string> strs;
		boost::split(strs, color_, boost::is_any_of(","));
		if (strs.size() != 3)
			throw std::exception();

		// Try to cast to double and divide by 255.
		double r = boost::lexical_cast<double>(strs[0]) /255;
		double g = boost::lexical_cast<double>(strs[1]) /255;
		double b = boost::lexical_cast<double>(strs[2]) /255;

		CLOG(LINFO) << "CloudViewer::onBackgroundColorChange r=" << r << " g=" << g << " b=" << b;
		// Change background color.
		if (viewer)
			viewer->setBackgroundColor(r, g, b);
	} catch (...) {
		CLOG(LWARNING)
				<< "CloudViewer::onBackgroundColorChange failed - invalid color format. Accepted format: r,g,b";
	}

}

void CloudViewer::onSIFTColorChange(std::string color_) {
	CLOG(LDEBUG) << "CloudViewer::onSIFTColorChange color=" << color_;
	try {
		// Parse string.
		vector<std::string> strs;
		boost::split(strs, color_, boost::is_any_of(","));
		if (strs.size() != 3)
			throw std::exception();

		// Try to cast to double and divide by 255.
		double r = boost::lexical_cast<double>(strs[0]) /255;
		double g = boost::lexical_cast<double>(strs[1]) /255;
		double b = boost::lexical_cast<double>(strs[2]) /255;

		CLOG(LINFO) << "CloudViewer::onSIFTColorChange r=" << r << " g=" << g << " b=" << b;
		// Change SIFT color.
		if (viewer)
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "xyzsift");
	} catch (...) {
		CLOG(LWARNING)
				<< "CloudViewer::onSIFTColorChange failed - invalid color format. Accepted format: r,g,b";
	}

}


CloudViewer::~CloudViewer() {
}

void CloudViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_normals", &in_cloud_normals);
	registerStream("in_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);

	// Register handlers
	registerHandler("on_cloud_xyz",
			boost::bind(&CloudViewer::on_cloud_xyz, this));
	addDependency("on_cloud_xyz", &in_cloud_xyz);

	registerHandler("on_cloud_xyzrgb",
			boost::bind(&CloudViewer::on_cloud_xyzrgb, this));
	addDependency("on_cloud_xyzrgb", &in_cloud_xyzrgb);

	registerHandler("on_cloud_xyzsift",
			boost::bind(&CloudViewer::on_cloud_xyzsift, this));
	addDependency("on_cloud_xyzsift", &in_cloud_xyzsift);

	registerHandler("on_cloud_normals",
			boost::bind(&CloudViewer::on_cloud_normals, this));
	addDependency("on_cloud_normals", &in_cloud_normals);

	registerHandler("on_spin", boost::bind(&CloudViewer::on_spin, this));
	addDependency("on_spin", NULL);

	registerHandler("on_cloud_xyzrgb_normals",
			boost::bind(&CloudViewer::on_cloud_xyzrgb_normals, this));
	addDependency("on_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);
}

bool CloudViewer::onInit() {

	CLOG(LTRACE) << "CloudViewer::onInit, prop_two_viewports==false\n";
	viewer = new pcl::visualization::PCLVisualizer(prop_window_name);
	//viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);

	// Try to change background color.
	onBackgroundColorChange(prop_background_color);

	// Display/hide coordinate system.
	onCSShowClick(prop_coordinate_system);

	// Init camera parameters.
	viewer->initCameraParameters();

	// Add cloud of XYZ type.
	viewer->addPointCloud<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "xyz");

	// Add cloud of XYZRBG type.
	viewer->addPointCloud<pcl::PointXYZRGB>(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>), "xyzrgb");

	// Add cloud of XYZ type - for SIFTs.
	viewer->addPointCloud<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "xyzsift") ;
	// Set SIFT's properties.
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, prop_sift_size, "xyzsift");
	onSIFTColorChange(prop_sift_color);

	// Add cloud of XYZRBGNormals type.
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>()),
			pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>()),
			normals_level, normals_scale, "xyzrgbnormals");

//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "xyz");
	return true;
}

bool CloudViewer::onFinish() {
	return true;
}

bool CloudViewer::onStop() {
	return true;
}

bool CloudViewer::onStart() {
	return true;
}

void CloudViewer::on_cloud_xyz() {
	CLOG(LTRACE) << "CloudViewer::on_cloud_xyz\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();

	viewer->updatePointCloud<pcl::PointXYZ> (cloud,"xyz");
}

void CloudViewer::on_cloud_xyzrgb() {
	CLOG(LTRACE) << "CloudViewer::on_cloud_xyzrgb\n";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	// Filter the NaN points.
	std::vector<int> indices;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution(cloud);
	viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, color_distribution, "xyzrgb");
//	viewer->removePointCloud("xyzrgb");
//	viewer->addPointCloud<pcl::PointXYZRGB> (cloud,"xyzrgb");
}

void CloudViewer::on_cloud_xyzsift() {
	CLOG(LTRACE) << "CloudViewer::on_cloud_xyzsift";
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();

	// Filter the NaN points.
	std::vector<int> indices;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	// Transform to XYZ cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, *cloud_xyz);

	// Update cloud.
	viewer->updatePointCloud<pcl::PointXYZ> (cloud_xyz,"xyzsift");

	// Update cloud properties.
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, prop_sift_size,"xyzsift");
}

void CloudViewer::on_cloud_normals() {
	CLOG(LERROR) << "CloudViewer::on_cloud_normals not implemented!";
}

void CloudViewer::on_cloud_xyzrgb_normals() {
	CLOG(LWARNING) << "CloudViewer::on_cloud_xyzrgb_normals";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = in_cloud_xyzrgb_normals.read();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgb(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	pcl::copyPointCloud(*cloud, *cloudRgb);
	pcl::copyPointCloud(*cloud, *normals);

	// TODO: optimize! why remove all points and shapes, why RBGXYZ points are displayed separately?
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudRgb);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloudRgb, rgb, "xyzrgb");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloudRgb, normals, normals_level, normals_scale, "xyzrgbnormals");
}

void CloudViewer::on_spin() {
	viewer->spinOnce(100);
}

} //: namespace CloudViewer
} //: namespace Processors
