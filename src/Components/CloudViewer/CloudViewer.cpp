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
    prop_coordinate_system_scale("coordinate_system_scale", 1.0),
	prop_background_color("background_color", std::string("0,0,0")),
	prop_xyznormals_scale("xyznormals.scale", 0.1),
	prop_xyznormals_level("xyznormals.level", 1),
	prop_xyzsift_size("xyzsift.size", 1),
	prop_xyzsift_color("xyzsift.color", std::string("255,0,0")),
	prop_display_xyz("display.xyz", true),
	prop_display_xyzrgb("display.xyzrgb", true),
	prop_display_xyzsift("display.xyzsift", true),
	prop_display_xyznormals("xyznormals.display", true),
	prop_display_scene("display.scene", true),
	prop_display_objects("display.objects", true),
	prop_display_object_bounding_boxes("display.object_bounding_boxes",true)
{
	// General properties.
	registerProperty(prop_title);
	registerProperty(prop_coordinate_system);
    registerProperty(prop_coordinate_system_scale);
	registerProperty(prop_background_color);

	// Display properties.
	registerProperty(prop_display_scene);
	registerProperty(prop_display_objects);
	registerProperty(prop_display_xyz);
	registerProperty(prop_display_xyzrgb);
	registerProperty(prop_display_xyzsift);
	registerProperty(prop_display_xyznormals);
	registerProperty(prop_display_object_bounding_boxes);

	// XYZNormals properties.
	registerProperty(prop_xyznormals_scale);
	registerProperty(prop_xyznormals_level);

	// XYZSIFT properties.
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
	registerStream("in_om_corners_xyz", &in_om_corners_xyz);

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
	previous_om_names_size = 0;
	previous_om_clouds_xyzrgb_size= 0;
	previous_om_clouds_xyzsift_size = 0;
	previous_om_object_bounding_boxes_xyz_size = 0;

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
				viewer->addCoordinateSystem (prop_coordinate_system_scale);

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

	// Displays wireframes (bounding boxes) generated on the basis of xyz cloud containing corners.
	displayObjectBoundingBoxesFromCorners_xyz();

	// Refresh viewer.
	viewer->spinOnce(100);
}




void CloudViewer::displayClouds_xyz() {
	CLOG(LTRACE) << "displayClouds_xyz";

	if (!prop_display_xyz) {
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
	CLOG(LTRACE) << "displayClouds_xyzrgb";

	if (!prop_display_xyzrgb) {
		viewer->removePointCloud ("xyzrgb");

		// Remove object clouds
		for(int i=0; i< previous_om_clouds_xyzrgb_size; i++) {
			// cloud name.
			std::ostringstream s;
			s << i;
			std::string cname = "xyzrgb" + s.str();
			viewer->removePointCloud (cname);
		}//: for

	} else {

        // Update scene cloud.
		if(!prop_display_scene){
            viewer->removePointCloud ("xyzrgb");
        }
        else if (!in_cloud_xyzrgb.empty()){
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
			for(int i=0; i< previous_om_clouds_xyzrgb_size; i++) {
				// Generate cloud name.
				std::ostringstream s;
				s << i;
				std::string cname = "xyzrgb" + s.str();
				// Remove object/model cloud.
				viewer->removePointCloud (cname);
			}//: for
			if(prop_display_objects){
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

				previous_om_clouds_xyzrgb_size = om_clouds.size();
            }else{
				previous_om_clouds_xyzrgb_size = 0;
            }
		}//: if

	}//: else

}

void CloudViewer::displayClouds_xyzsift() {
	CLOG(LTRACE) << "displayClouds_xyzsift";

	if (!prop_display_xyzsift) {
		viewer->removePointCloud ("xyzsift");

		// Remove object clouds
		for(int i=0; i< previous_om_clouds_xyzsift_size; i++) {
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
		if(!prop_display_scene){
            viewer->removePointCloud ("xyzsift");
        }
        else if (!in_cloud_xyzsift.empty()){
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
			for(int i=0; i< previous_om_clouds_xyzsift_size; i++) {
				// Generate cloud name.
				std::ostringstream s;
				s << i;
				std::string cname = "xyzsift" + s.str();
				viewer->removePointCloud (cname);
			}//: for
			if(prop_display_objects){
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
				previous_om_clouds_xyzsift_size = om_clouds.size();
            }else{
				previous_om_clouds_xyzsift_size = 0;
            }

		}//: if



	}//: else
}


void CloudViewer::displayClouds_xyzrgb_normals() {
	CLOG(LTRACE) << "displayClouds_xyzrgb_normals";

	if (!prop_display_xyznormals) {
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



void CloudViewer::displayObjectBoundingBoxesFromCorners_xyz() {
	CLOG(LTRACE) << "displayObjectBoundingBoxesFromCorners_xyz";

	// Remove bounding boxes objects - performed always!
	for(int i=0; i< previous_om_object_bounding_boxes_xyz_size; i++) {
		// Generate given object BB name prefix.
		std::ostringstream s;
		s << i;
		std::string cname = "bounding_line" + s.str();
		// Remove lines one by one.
		viewer->removeShape(cname + std::string("01"));
		viewer->removeShape(cname + std::string("12"));
		viewer->removeShape(cname + std::string("23"));
		viewer->removeShape(cname + std::string("30"));
		viewer->removeShape(cname + std::string("04"));
		viewer->removeShape(cname + std::string("15"));
		viewer->removeShape(cname + std::string("26"));
		viewer->removeShape(cname + std::string("37"));
		viewer->removeShape(cname + std::string("45"));
		viewer->removeShape(cname + std::string("56"));
		viewer->removeShape(cname + std::string("67"));
		viewer->removeShape(cname + std::string("74"));
	}//: for

	if ((prop_display_object_bounding_boxes) && (!in_om_corners_xyz.empty())) {

		// Read cloud from port.
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> corners = in_om_corners_xyz.read();

		// Check bb size.
		// If too small.
		while (bounding_box_colours.size() < corners.size()) {
			// Add random colour.
			pcl::PointXYZ rgb ((double)(rand()&255)/255.0, (double)(rand()&255)/255.0, (double)(rand()&255)/255.0);
			bounding_box_colours.push_back(rgb);
		}//: while
		// If too big.
		while (bounding_box_colours.size() > corners.size()) {
			// Remove last colour from list.
			bounding_box_colours.pop_back();
		}//: while


		for(int i=0; i< corners.size(); i++) {
			// Generate given object BB name prefix.
			std::ostringstream s;
			s << i;
			std::string cname = "bounding_line" + s.str();
			// Get colour from list.
			double r = bounding_box_colours[i].x;
			double g = bounding_box_colours[i].y;
			double b = bounding_box_colours[i].z;

			// Remove lines one by one.
			viewer->addLine(corners[i]->at(0), corners[i]->at(1), r, g, b, cname + std::string("01"));
			viewer->addLine(corners[i]->at(1), corners[i]->at(2), r, g, b, cname + std::string("12"));
			viewer->addLine(corners[i]->at(2), corners[i]->at(3), r, g, b, cname + std::string("23"));
			viewer->addLine(corners[i]->at(3), corners[i]->at(0), r, g, b, cname + std::string("30"));
			viewer->addLine(corners[i]->at(0), corners[i]->at(4), r, g, b, cname + std::string("04"));
			viewer->addLine(corners[i]->at(1), corners[i]->at(5), r, g, b, cname + std::string("15"));
			viewer->addLine(corners[i]->at(2), corners[i]->at(6), r, g, b, cname + std::string("26"));
			viewer->addLine(corners[i]->at(3), corners[i]->at(7), r, g, b, cname + std::string("37"));
			viewer->addLine(corners[i]->at(4), corners[i]->at(5), r, g, b, cname + std::string("45"));
			viewer->addLine(corners[i]->at(5), corners[i]->at(6), r, g, b, cname + std::string("56"));
			viewer->addLine(corners[i]->at(6), corners[i]->at(7), r, g, b, cname + std::string("67"));
			viewer->addLine(corners[i]->at(7), corners[i]->at(4), r, g, b, cname + std::string("74"));
		}//: for

		previous_om_object_bounding_boxes_xyz_size = corners.size();
	} else {
		previous_om_object_bounding_boxes_xyz_size = 0;
	}//: else

}



} //: namespace CloudViewer
} //: namespace Processors
