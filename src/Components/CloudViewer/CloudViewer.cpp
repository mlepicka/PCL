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
	prop_title("title",std::string("Point Cloud Viewer")),
	prop_coordinate_system("coordinate_system",boost::bind(&CloudViewer::onCSShowClick, this, _2), true),
	prop_background_color("background_color", boost::bind(&CloudViewer::onBackgroundColorChange, this, _2), std::string("0,0,0")),
	prop_xyz_display("xyz.display", true),
	prop_xyzrgb_display("xyzrgb.display", true),
	prop_xyznormals_display("xyznormals.display", true),
	prop_xyznormals_scale("xyznormals.scale", 0.1),
	prop_xyznormals_level("xyznormals.level", 1),
	prop_xyzsift_display("xyzsift.display", true),
	prop_xyzsift_size("xyzsift.size", 1),
	prop_xyzsift_color("xyzsift.color", std::string("255,0,0"))
{
	// General properties.
	registerProperty(prop_title);
	registerProperty(prop_coordinate_system);
	registerProperty(prop_background_color);

	// XYZ properties.
	registerProperty(prop_xyz_display);

	// XYZRGB properties.
	registerProperty(prop_xyzrgb_display);

	// XYZNormals properties.
	registerProperty(prop_xyznormals_display);
	registerProperty(prop_xyznormals_scale);
	registerProperty(prop_xyznormals_level);

	// XYZSIFT properties.
	registerProperty(prop_xyzsift_display);
	registerProperty(prop_xyzsift_size);
	registerProperty(prop_xyzsift_color);

	viewer = NULL;
}

void CloudViewer::onCSShowClick(const bool & new_show_cs_) {
	CLOG(LTRACE) << "onCSShowClick show=" << new_show_cs_;
    if (!viewer)
    	return;

	if (new_show_cs_) {
//#if PCL_VERSION_COMPARE(>=,1,7,1)
		viewer->addCoordinateSystem (); 
//#endif
	// TODO: Currently only 1.7.1 is available in the 012/031 laboratories.
	// TODO: Fix for other versions of PCL.
	} else {
//#if PCL_VERSION_COMPARE(>=,1,7,1)
		viewer->removeCoordinateSystem ();
//#endif
	// TODO: Currently only 1.7.1 is available in the 012/031 laboratories.
	// TODO: Fix for other versions of PCL.
	}

	prop_coordinate_system = new_show_cs_;
}


bool CloudViewer::parseColor(std::string color_, double & r_, double & g_, double & b_) {
	CLOG(LTRACE) << "parseColor";
	CLOG(LDEBUG) << "color=" << color_;
	try {
		vector<std::string> strs;
		boost::split(strs, color_, boost::is_any_of(","));
		if (strs.size() != 3)
			throw std::exception();
		// Try to cast to double and divide by 255.
		r_ = boost::lexical_cast<double>(strs[0]) /255;
		g_ = boost::lexical_cast<double>(strs[1]) /255;
		b_ = boost::lexical_cast<double>(strs[2]) /255;

		CLOG(LINFO) << "r=" << r_ << " g=" << g_ << " b=" << b_;
		return true;
	} catch (...) {
		CLOG(LWARNING) << "parseColor failure - invalid color format. Accepted format: r,g,b";
		return false;
	}
}


void CloudViewer::onBackgroundColorChange(std::string color_) {
	CLOG(LTRACE) << "onBackgroundColorChange";
	double r=0, g=0, b=0;
	parseColor(color_, r, g, b);
	// Change background color.
	if (viewer)
		viewer->setBackgroundColor(r, g, b);
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

	CLOG(LTRACE) << "onInit\n";
	viewer = new pcl::visualization::PCLVisualizer(prop_title);

	// Try to change background color.
	onBackgroundColorChange(prop_background_color);

	// Display/hide coordinate system.
	onCSShowClick(prop_coordinate_system);

	// Init camera parameters.
	viewer->initCameraParameters();

/*	// Add cloud of XYZ type.
	viewer->addPointCloud<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "xyz");

	// Add cloud of XYZRBG type.
	viewer->addPointCloud<pcl::PointXYZRGB>(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>), "xyzrgb");

	// Add cloud of XYZ type - for SIFTs.
	viewer->addPointCloud<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "xyzsift") ;

	// Add cloud of XYZRBGNormals type.
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>()),
			pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>()),
			prop_xyznormals_level, prop_xyznormals_scale, "xyzrgbnormals");
*/
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
	CLOG(LTRACE) << "on_cloud_xyz\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();

	// UPDATE: Display XYZ cloud if required.
	if (!prop_xyz_display)
		viewer->removePointCloud ("xyz");
	else{
		if (!viewer->updatePointCloud<pcl::PointXYZ> (cloud,"xyz"))
			viewer->addPointCloud<pcl::PointXYZ> (cloud,"xyz");
	}//: else

}

void CloudViewer::on_cloud_xyzrgb() {
	CLOG(LTRACE) << "on_cloud_xyzrgb\n";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	// Filter the NaN points.
	std::vector<int> indices;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution(cloud);

	// UPDATE: Display cloud only if required.
	if (!prop_xyzrgb_display)
		viewer->removePointCloud ("xyzrgb");
	else{
		if (!viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, color_distribution, "xyzrgb"))
			viewer->addPointCloud<pcl::PointXYZRGB> (cloud, color_distribution, "xyzrgb");
	}//: else

}

void CloudViewer::on_cloud_xyzsift() {
	CLOG(LTRACE) << "on_cloud_xyzsift";
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();

	// Filter the NaN points.
	std::vector<int> indices;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	// Transform to XYZ cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, *cloud_xyz);


	// UPDATE: Display cloud only if required.
	if (!prop_xyzsift_display)
		viewer->removePointCloud ("xyzsift");
	else{
		if (!viewer->updatePointCloud<pcl::PointXYZ> (cloud_xyz, "xyzsift"))
			viewer->addPointCloud<pcl::PointXYZ> (cloud_xyz, "xyzsift");
		// Update cloud properties.
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, prop_xyzsift_size,"xyzsift");
		double r=255, g=0, b=0;
		if(parseColor(prop_xyzsift_color, r, g, b))
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "xyzsift");
	}//: else
}

void CloudViewer::on_cloud_normals() {
	CLOG(LERROR) << "on_cloud_normals not implemented!";
}

void CloudViewer::on_cloud_xyzrgb_normals() {
	CLOG(LWARNING) << "on_cloud_xyzrgb_normals";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = in_cloud_xyzrgb_normals.read();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgb(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	pcl::copyPointCloud(*cloud, *cloudRgb);
	pcl::copyPointCloud(*cloud, *normals);

	// TODO: optimize! why remove all points and shapes, why RBGXYZ points are displayed separately?
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	if (!prop_xyznormals_display) {
/*		viewer->removePointCloud ("xyzrgb");
		viewer->removePointCloud ("xyzrgbnormals");*/
	} else {
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudRgb);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloudRgb, rgb, "xyzrgb");
		viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloudRgb, normals, prop_xyznormals_level, prop_xyznormals_scale, "xyzrgbnormals");
	}//: else
}

void CloudViewer::on_spin() {
	viewer->spinOnce(100);
}

} //: namespace CloudViewer
} //: namespace Processors
