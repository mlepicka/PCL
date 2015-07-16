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
	prop_coordinate_system("coordinate_system", true),
	prop_background_color("background_color", std::string("0,0,0")),
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

		CLOG(LDEBUG) << "r=" << r_ << " g=" << g_ << " b=" << b_;
		return true;
	} catch (...) {
		CLOG(LWARNING) << "parseColor failure - invalid color format. Accepted format: r,g,b";
		return false;
	}
}



CloudViewer::~CloudViewer() {
}

void CloudViewer::prepareInterface() {
	// Register basic cloud data streams.
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);

	// Register cloud "scene" aliases.
	registerStream("in_scene_cloud_xyz", &in_cloud_xyz);
	registerStream("in_scene_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_scene_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_scene_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);

	// Register cloud object/model streams.
	registerStream("in_om_ids", &in_om_ids);
	registerStream("in_om_clouds_xyzrgb", &in_om_clouds_xyzrgb);
	registerStream("in_om_clouds_xyzsift", &in_om_clouds_xyzsift);

	registerHandler("on_spin", boost::bind(&CloudViewer::refreshViewerState, this));
	addDependency("on_spin", NULL);
}



bool CloudViewer::onInit() {
	CLOG(LTRACE) << "onInit";
	viewer = new pcl::visualization::PCLVisualizer(prop_title);

	// Init camera parameters.
	viewer->initCameraParameters();

	// Init variables.
	coordinate_system_status_flag = !prop_coordinate_system;
	previous_om_names_number = 0;
	previous_om_clouds_xyzrgb_number= 0;
	previous_om_clouds_xyzsift_number = 0;

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

void CloudViewer::displayClouds_xyz() {
	CLOG(LTRACE) << "on_cloud_xyz";

	if (!prop_xyz_display) {
		viewer->removePointCloud ("xyz");
	} else {

		// If empty cloud - do nothing.
		if (in_cloud_xyz.empty())
			return;

		// Read cloud from port.
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();

		// UPDATE: Display XYZ cloud if required.
		if (!viewer->updatePointCloud<pcl::PointXYZ> (cloud,"xyz"))
			viewer->addPointCloud<pcl::PointXYZ> (cloud,"xyz");

		// TODO: objects?

	}//: else

}

void CloudViewer::displayClouds_xyzrgb() {
	CLOG(LTRACE) << "on_cloud_xyzrgb";

	if (!prop_xyzrgb_display) {
		viewer->removePointCloud ("xyzrgb");

		// Remove object clouds
		for(int i=0; i< previous_om_clouds_xyzrgb_number; i++) {
			// cloud name.
			std::ostringstream s;
			s << i;
			std::string cname = "xyzrgb" + s.str();
			viewer->removePointCloud (cname);
		}//: for

	} else {

		// Update scene cloud.
		if (!in_cloud_xyzrgb.empty()){
			// Read cloud from port.
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

			// Filter the NaN points.
			std::vector<int> indices;
			cloud->is_dense = false;
			pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

			// Colour field hanlder.
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution(cloud);

			// UPDATE: Display cloud only if required.
			if (!viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, color_distribution, "xyzrgb"))
				viewer->addPointCloud<pcl::PointXYZRGB> (cloud, color_distribution, "xyzrgb");
		}//: if cloud empty


		// Update object/models clouds.
		if (!in_om_clouds_xyzrgb.empty()){

			// Read clouds from port.
			std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > om_clouds = in_om_clouds_xyzrgb.read();

			// 3 cases: in_om_clouds_xyzrgb.size < || > || == displayed_om_number
			// Instead remove all previous and add new - not optimal solution - but those are small clouds!

			// Remove object clouds.
			for(int i=0; i< previous_om_clouds_xyzrgb_number; i++) {
				// Generate cloud name.
				std::ostringstream s;
				s << i;
				std::string cname = "xyzrgb" + s.str();
				// Remove object/model cloud.
				viewer->removePointCloud (cname);
			}//: for

			for(int i=0; i< om_clouds.size(); i++) {
				// Generate cloud name.
				std::ostringstream s;
				s << i;
				std::string cname = "xyzrgb" + s.str();

				// Colour field hanlder.
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution(om_clouds[i]);

				// Add object/model cloud.
				viewer->addPointCloud<pcl::PointXYZRGB> (om_clouds[i], color_distribution, cname);
			}//: for

			previous_om_clouds_xyzrgb_number = om_clouds.size();
		}//: if

	}//: else

}

void CloudViewer::displayClouds_xyzsift() {
	CLOG(LTRACE) << "on_cloud_xyzsift";

	if (!prop_xyzsift_display) {
		viewer->removePointCloud ("xyzsift");

		// Remove object clouds
		for(int i=0; i< previous_om_clouds_xyzsift_number; i++) {
			// cloud name.
			std::ostringstream s;
			s << i;
			std::string cname = "xyzsift" + s.str();
			viewer->removePointCloud (cname);
		}//: for

	} else {
		// Set SIFT colours.
		double r=255, g=0, b=0;
		parseColor(prop_xyzsift_color, r, g, b);

		// Update scene cloud.
		if (!in_cloud_xyzsift.empty()){
			// Read cloud from port.
			pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();

			// Filter the NaN points.
			std::vector<int> indices;
			cloud->is_dense = false;
			pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

			// Transform to XYZ cloud.
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*cloud, *cloud_xyz);

			// UPDATE: Display cloud only if required.
			if (!viewer->updatePointCloud<pcl::PointXYZ> (cloud_xyz, "xyzsift"))
				viewer->addPointCloud<pcl::PointXYZ> (cloud_xyz, "xyzsift");

			// Update SIFT cloud properties.
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, prop_xyzsift_size,"xyzsift");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "xyzsift");
		}//: if cloud empty


		// Update object/models clouds.
		if (!in_om_clouds_xyzsift.empty()){

			// Read clouds from port.
			std::vector< pcl::PointCloud<PointXYZSIFT>::Ptr > om_clouds = in_om_clouds_xyzsift.read();

			// 3 cases: in_om_clouds_xyzsift.size < || > || == displayed_om_number
			// Instead remove all previous and add new - not optimal solution - but those are small clouds!

			// Remove object clouds.
			for(int i=0; i< previous_om_clouds_xyzsift_number; i++) {
				// Generate cloud name.
				std::ostringstream s;
				s << i;
				std::string cname = "xyzsift" + s.str();
				viewer->removePointCloud (cname);
			}//: for

			for(int i=0; i< om_clouds.size(); i++) {
				// Generate cloud name.
				std::ostringstream s;
				s << i;
				std::string cname = "xyzsift" + s.str();

				// Transform to XYZ cloud.
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*(om_clouds[i]), *cloud_xyz);

				viewer->addPointCloud<pcl::PointXYZ> (cloud_xyz, cname);

				// Update SIFT cloud properties.
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, prop_xyzsift_size, cname);
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cname);
			}//: for

			previous_om_clouds_xyzsift_number = om_clouds.size();
		}//: if



	}//: else
}


void CloudViewer::displayClouds_xyzrgb_normals() {
	CLOG(LTRACE) << "on_cloud_xyzrgb_normals";

	if (!prop_xyznormals_display) {
/*		viewer->removePointCloud ("xyzrgb");
		viewer->removePointCloud ("xyzrgbnormals");*/
	} else {

		// If empty cloud - do nothing.
		if (in_cloud_xyzrgb_normals.empty())
			return;

		// Read cloud from port.
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
		viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloudRgb, normals, prop_xyznormals_level, prop_xyznormals_scale, "xyzrgbnormals");
	}//: else
}

void CloudViewer::refreshViewerState() {
	CLOG(LTRACE) << "refreshViewerState";

	if (!viewer)
		return;

	// Change background color.
	double r=0, g=0, b=0;
	parseColor(prop_background_color, r, g, b);
	viewer->setBackgroundColor(r, g, b);

	// Show/hide coordinate system.
	CLOG(LDEBUG) << "prop_coordinate_system="<<prop_coordinate_system;
	if (coordinate_system_status_flag != prop_coordinate_system) {
		if (prop_coordinate_system) {
			// TODO: Currently only 1.7.1/1.7.2 is available in the 012/031/032 laboratories. Fix for other versions of PCL.
			//#if PCL_VERSION_COMPARE(>=,1,7,1)
				viewer->addCoordinateSystem ();

			//#endif
		} else {
				viewer->removeCoordinateSystem ();
		}//: else
		coordinate_system_status_flag = prop_coordinate_system;
	}

	// TODO: check sizes of om names/clouds!!!


	// Displays xyz cloud.
	displayClouds_xyz();

	// Displays xyz cloud.
	displayClouds_xyzrgb();

	// Displays xyz cloud.
	displayClouds_xyzsift();

	// Displays xyzrgb with normals cloud.
	displayClouds_xyzrgb_normals();

	// Refresh viewer.
	viewer->spinOnce(100);
}

} //: namespace CloudViewer
} //: namespace Processors
