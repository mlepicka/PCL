/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#include <memory>
#include <string>

#include "CloudViewer.hpp"
#include "Common/Logger.hpp"

#include <algorithm>
#include <boost/bind.hpp>
#include <pcl/filters/filter.h>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

namespace Processors {
namespace CloudViewer {


CloudViewer::CloudViewer(const std::string & name) :
	Base::Component(name),
	prop_title("title",std::string("Point Cloud Viewer")),
	prop_coordinate_system_scale("coordinate_system_scale", 0.3),
	prop_background_color("background_color", std::string("0,0,0")),
	prop_xyznormals_scale("xyznormals.scale", 0.1),
	prop_xyznormals_level("xyznormals.level", 1),
	prop_xyzsift_size("xyzsift.size", 1),
	prop_xyzsift_color("xyzsift.color", std::string("255,0,0")),
	prop_label_scale("labels.scale", 0.03),
	prop_scene_coordinate_system("scene.display_coordinate_system", true),
	prop_display_scene_xyz("scene.display_xyz", true),
	prop_display_scene_xyzrgb("scene.display_xyzrgb", true),
	prop_display_scene_xyzsift("scene.display_xyzsift", true),
	prop_display_scene_xyznormals("scene.display_xyznormals", true),
	prop_scene_translation_x("scene.translation.x", 0),
	prop_scene_translation_y("scene.translation.y", 0),
	prop_scene_translation_z("scene.translation.z", 0),
	prop_display_objects_xyz("objects.display_xyz", true),
	prop_display_objects_xyzrgb("objects.display_xyzrgb", true),
	prop_display_objects_xyzsift("objects.display_xyzsift", true),
	prop_display_objects_xyznormals("objects.display_xyznormals", true),
	prop_display_object_bounding_boxes("objects.display_bounding_boxes",true),
	prop_display_object_labels("objects.display_labels", true),
	prop_display_objects_scene_correspondences("objects.display_scene_correspondences",true),
	prop_display_object_coordinate_systems("objects.display_coordinate_systems",true)
{
	// General properties.
	registerProperty(prop_title);
    registerProperty(prop_coordinate_system_scale);
	registerProperty(prop_background_color);

	// Scene properties.
	registerProperty(prop_scene_coordinate_system);
	registerProperty(prop_display_scene_xyz);
	registerProperty(prop_display_scene_xyzrgb);
	registerProperty(prop_display_scene_xyznormals);
	registerProperty(prop_display_scene_xyzsift);
	// Scene translation properties.
	registerProperty(prop_scene_translation_x);
	registerProperty(prop_scene_translation_y);
	registerProperty(prop_scene_translation_z);

	// Objects properties.
	registerProperty(prop_display_objects_xyz);
	registerProperty(prop_display_objects_xyzrgb);
	registerProperty(prop_display_objects_xyznormals);
	registerProperty(prop_display_objects_xyzsift);
	registerProperty(prop_display_object_bounding_boxes);
	registerProperty(prop_display_object_labels);
	registerProperty(prop_display_objects_scene_correspondences);
	registerProperty(prop_display_object_coordinate_systems);

	// XYZNormals properties.
	registerProperty(prop_xyznormals_scale);
	registerProperty(prop_xyznormals_level);

	// XYZSIFT properties.
	registerProperty(prop_xyzsift_size);
	registerProperty(prop_xyzsift_color);

	// Label properties.
	registerProperty(prop_label_scale);


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
	//registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	//registerStream("in_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);

	// Register cloud "scene" aliases.
	//registerStream("in_scene_cloud_xyz", &in_cloud_xyz);
	registerStream("in_scene_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_scene_cloud_xyzsift", &in_cloud_xyzsift);
	//registerStream("in_scene_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);

	// Register cloud objects/clusters/models streams.
	registerStream("in_object_labels", &in_object_labels);
	registerStream("in_object_clouds_xyzrgb", &in_object_clouds_xyzrgb);
	registerStream("in_object_clouds_xyzsift", &in_object_clouds_xyzsift);
	registerStream("in_object_corners_xyz", &in_object_corners_xyz);
	registerStream("in_object_poses", &in_object_poses);

	// Register cloud objects/clusters/models-scene correspondences streams.
	registerStream("in_objects_scene_correspondences", &in_objects_scene_correspondences);

	registerHandler("on_spin", boost::bind(&CloudViewer::refreshViewerState, this));
	addDependency("on_spin", NULL);
}



bool CloudViewer::onInit() {
	CLOG(LTRACE) << "onInit";

	// Initialize camera and set its parameters.
	viewer = new pcl::visualization::PCLVisualizer(prop_title);
	viewer->initCameraParameters();

	// Initialize  coodrinate system flag.
	scene_coordinate_system_status_flag = !prop_scene_coordinate_system;

	// Initialize  previous sizes.
	previous_om_xyzrgb_size = 0;
	previous_om_xyzsift_size = 0;
	previous_om_bb_size = 0;
	previous_oms_correspondences_size = 0;
	previous_om_labels_size = 0;
	previous_om_coordinate_systems_size = 0;

	// Initialize
    scene_cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    scene_cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());

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
	CLOG(LDEBUG) << "prop_coordinate_system="<<prop_scene_coordinate_system;
	if (scene_coordinate_system_status_flag != prop_scene_coordinate_system) {
		if (prop_scene_coordinate_system) {
			// TODO: Currently only 1.7.1/1.7.2 is available in the 012/031/032 laboratories. Fix for other versions of PCL.
			//#if PCL_VERSION_COMPARE(>=,1,7,1)
				viewer->addCoordinateSystem (prop_coordinate_system_scale);

			//#endif
		} else {
				viewer->removeCoordinateSystem ();
		}//: else
		scene_coordinate_system_status_flag = prop_scene_coordinate_system;
	}

	// Check whether object names changed - if so, reload all objects, if not - leave unchanged... what about object poses?

    bool refresh_correspondences = false;

    // Define translation between clouds.
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0, 3) = prop_scene_translation_x;
    trans(1, 3) = prop_scene_translation_y;
    trans(2, 3) = prop_scene_translation_z;

	// Check scene xyzrgb cloud.
	if (!in_cloud_xyzrgb.empty()){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_xyzrgb_tmp = in_cloud_xyzrgb.read();
        pcl::transformPointCloud(*scene_cloud_xyzrgb_tmp, *scene_cloud_xyzrgb, trans);
        refreshSceneCloudXYZRGB(scene_cloud_xyzrgb);
	}//: if


	// Check scene xyzsift cloud.
	if (!in_cloud_xyzsift.empty()){
        pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift_tmp = in_cloud_xyzsift.read();
		//is_fresh_scene_xyzsift = true;
        pcl::transformPointCloud(*scene_cloud_xyzsift_tmp, *scene_cloud_xyzsift, trans);
        refreshSceneCloudXYZSIFT(scene_cloud_xyzsift);
        refresh_correspondences = true;
	}//: if


	// Read om XYZRGB clouds from port.
	if (!in_object_clouds_xyzrgb.empty()){
			object_clouds_xyzrgb = in_object_clouds_xyzrgb.read();
			refreshOMCloudsXYZRGB(object_clouds_xyzrgb);
	}//: if

	// Read om XYZSIFT clouds from port.
	if (!in_object_clouds_xyzsift.empty()){
			object_clouds_xyzsift = in_object_clouds_xyzsift.read();
			refreshOMCloudsXYZSIFT(object_clouds_xyzsift);
			//is_fresh_om_xyzsift = true;
            refresh_correspondences = true;
    }//: if

	// Read om bounding boxes from port.
	if (!in_object_corners_xyz.empty()){
			object_corners_xyz = in_object_corners_xyz.read();
			refreshOMBoundingBoxesFromCorners(object_corners_xyz);
	}//: if

	// Read models-scene correspondences from port.
    if (!in_objects_scene_correspondences.empty()){
        objects_scene_correspondences = in_objects_scene_correspondences.read();
        refresh_correspondences = true;
    }//: if

    if (refresh_correspondences){ // && is_fresh_scene_xyzsift && is_fresh_om_xyzsift){
			if (scene_cloud_xyzsift->empty()) {
				CLOG(LERROR) << "Cannot display correspondences as scene_cloud_xyzsift is empty";
			} else if (object_clouds_xyzsift.empty()) {
				CLOG(LERROR) << "Cannot display correspondences as om_clouds_xyzsift is empty";
            } else if (objects_scene_correspondences.empty()) {
                CLOG(LERROR) << "Cannot display correspondences as objects_scene_correspondences is empty";
            } else {
				refreshModelsSceneCorrespondences(objects_scene_correspondences, scene_cloud_xyzsift, object_clouds_xyzsift);
			}//: else
	}//: if

	// Read object/models names from port.
	if (!in_object_labels.empty()){
		if (object_corners_xyz.empty()) {
			CLOG(LERROR) << "Cannot display object ids as om_corners_xyz is empty";
		} else {
			refreshOMIds(in_object_labels.read(), object_corners_xyz);
		}//: else

	}//: if

	if (!in_object_poses.empty()){
		om_poses = in_object_poses.read();
		refreshOMCoordinateSystems(om_poses);
	}//: if

	// Refresh viewer.
	viewer->spinOnce(100);
}


void CloudViewer::refreshSceneCloudXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_xyzrgb_){
	CLOG(LTRACE) << "refreshSceneCloudXYZRGB";

	if (!prop_display_scene_xyzrgb) {
		viewer->removePointCloud ("scene_xyzrgb");
	} else {
		// Filter the NaN points.
		std::vector<int> indices;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);;
		pcl::removeNaNFromPointCloud(*scene_cloud_xyzrgb_, *tmp_cloud_xyzrgb, indices);
		tmp_cloud_xyzrgb->is_dense = false;

		// Colour field hanlder.
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution(tmp_cloud_xyzrgb);

		// UPDATE: Display cloud only if required.
		if (!viewer->updatePointCloud<pcl::PointXYZRGB> (tmp_cloud_xyzrgb, color_distribution, "scene_xyzrgb"))
			viewer->addPointCloud<pcl::PointXYZRGB> (tmp_cloud_xyzrgb, color_distribution, "scene_xyzrgb");
	}//: else
}


void CloudViewer::refreshSceneCloudXYZSIFT(pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift_){
	CLOG(LTRACE) << "refreshSceneCloudXYZSIFT";

	if (!prop_display_scene_xyzsift) {
		viewer->removePointCloud ("scene_xyzsift");
	} else {
		// Transform to XYZ cloud.
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*scene_cloud_xyzsift_, *cloud_xyz);

		// Filter the NaN points.
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud_xyz, *cloud_xyz, indices);
		cloud_xyz->is_dense = false;

		// UPDATE: Display cloud only if required.
		if (!viewer->updatePointCloud<pcl::PointXYZ> (cloud_xyz, "scene_xyzsift"))
			viewer->addPointCloud<pcl::PointXYZ> (cloud_xyz, "scene_xyzsift");

		// Set SIFT colours.
		double r=255, g=0, b=0;
		parseColor(prop_xyzsift_color, r, g, b);

		// Update SIFT cloud properties.
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, prop_xyzsift_size,"scene_xyzsift");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "scene_xyzsift");
	}//: else
}


void CloudViewer::refreshOMCloudsXYZRGB(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> om_clouds_xyzrgb_) {
	CLOG(LTRACE) << "refreshOMCloudsXYZRGB";

	if (!prop_display_objects_xyzrgb) {
		// Remove object clouds
		for(int i=0; i< previous_om_xyzrgb_size; i++) {
			// cloud name.
			std::ostringstream s;
			s << i;
			std::string cname = "xyzrgb_" + s.str();
			viewer->removePointCloud (cname);
		}//: for

		previous_om_xyzrgb_size = 0;
	} else {
		// Update object clouds that need to be updated.
		for(int i=0; i< om_clouds_xyzrgb_.size(); i++) {
			// Generate cloud name.
			std::ostringstream s;
			s << i;
			std::string cname = "xyzrgb_" + s.str();

			// Filter the NaN points.
			std::vector<int> indices;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);;
			pcl::removeNaNFromPointCloud(*(om_clouds_xyzrgb_[i]), *tmp_cloud_xyzrgb, indices);
			tmp_cloud_xyzrgb->is_dense = false;

			// Colour field hanlder.
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution(tmp_cloud_xyzrgb);

			// UPDATE: Display cloud only if required.
			if (!viewer->updatePointCloud<pcl::PointXYZRGB> (tmp_cloud_xyzrgb, color_distribution, cname))
				viewer->addPointCloud<pcl::PointXYZRGB> (tmp_cloud_xyzrgb, color_distribution, cname);
		}//: for

		// Remove unnecessary object clouds.
		for(int i=om_clouds_xyzrgb_.size(); i< previous_om_xyzrgb_size; i++) {
			// Generate cloud name.
			std::ostringstream s;
			s << i;
			std::string cname = "xyzrgb_" + s.str();
			// Remove object/model cloud.
			viewer->removePointCloud (cname);
		}//: for

		previous_om_xyzrgb_size = om_clouds_xyzrgb_.size();
	}//: else

}


void CloudViewer::refreshOMCloudsXYZSIFT(std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> om_clouds_xyzsift_) {
	CLOG(LTRACE) << "refreshOMCloudsXYZSIFT";

	if (!prop_display_objects_xyzsift) {
		// Remove object clouds
		for(int i=0; i< previous_om_xyzsift_size; i++) {
			// cloud name.
			std::ostringstream s;
			s << i;
			std::string cname = "xyzsift_" + s.str();
			viewer->removePointCloud (cname);
		}//: for

		previous_om_xyzsift_size = 0;
	} else {

		// Generate colour vector for MAX of om clouds (sifts, corners, correspondences,names).
		resizeColourVector(om_clouds_xyzsift_.size());

		// Update object clouds that need to be updated.
		for(int i=0; i< om_clouds_xyzsift_.size(); i++) {
			// Generate cloud name.
			std::ostringstream s;
			s << i;
			std::string cname = "xyzsift_" + s.str();

			// Transform to XYZ cloud.
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*(om_clouds_xyzsift_[i]), *tmp_cloud_xyz);

			// Filter the NaN points.
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*tmp_cloud_xyz, *tmp_cloud_xyz, indices);
			tmp_cloud_xyz->is_dense = false;

			// UPDATE: Display cloud only if required.
			if (!viewer->updatePointCloud<pcl::PointXYZ> (tmp_cloud_xyz, cname))
				viewer->addPointCloud<pcl::PointXYZ> (tmp_cloud_xyz, cname);

			// Set SIFT colours.
			double r= colours[i][0];
			double g= colours[i][1];
			double b= colours[i][2];

			// Update SIFT cloud properties.
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, prop_xyzsift_size, cname);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cname);

		}//: for

		// Remove unnecessary object clouds.
		for(int i=om_clouds_xyzsift_.size(); i< previous_om_xyzsift_size; i++) {
			// Generate cloud name.
			std::ostringstream s;
			s << i;
			std::string cname = "xyzsift_" + s.str();
			// Remove object/model cloud.
			viewer->removePointCloud (cname);
		}//: for

		previous_om_xyzsift_size = om_clouds_xyzsift_.size();
	}//: else

}


void CloudViewer::refreshOMBoundingBoxesFromCorners(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  om_corners_xyz_) {
	CLOG(LTRACE) << "refreshOMBoundingBoxesFromCorners";

	// Remove bounding boxes objects - performed always! TODO FIX!
	for(int i=0; i< previous_om_bb_size; i++) {
		// Generate given object BB name prefix.
		std::ostringstream s;
		s << i;
		std::string cname = "bounding_line_" + s.str()+"_";
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

	if (prop_display_object_bounding_boxes) {

		// Generate colour vector for MAX of om clouds (sifts, corners, correspondences,names).
		resizeColourVector(om_corners_xyz_.size());

		for(int i=0; i< om_corners_xyz_.size(); i++) {
			// Generate given object BB name prefix.
			std::ostringstream s;
			s << i;
			std::string cname = "bounding_line_" + s.str()+"_";
			// Get colour from list.
			double r = colours[i][0];
			double g = colours[i][1];
			double b = colours[i][2];

			// Remove lines one by one.
			viewer->addLine(om_corners_xyz_[i]->at(0), om_corners_xyz_[i]->at(1), r, g, b, cname + std::string("01"));
			viewer->addLine(om_corners_xyz_[i]->at(1), om_corners_xyz_[i]->at(2), r, g, b, cname + std::string("12"));
			viewer->addLine(om_corners_xyz_[i]->at(2), om_corners_xyz_[i]->at(3), r, g, b, cname + std::string("23"));
			viewer->addLine(om_corners_xyz_[i]->at(3), om_corners_xyz_[i]->at(0), r, g, b, cname + std::string("30"));
			viewer->addLine(om_corners_xyz_[i]->at(0), om_corners_xyz_[i]->at(4), r, g, b, cname + std::string("04"));
			viewer->addLine(om_corners_xyz_[i]->at(1), om_corners_xyz_[i]->at(5), r, g, b, cname + std::string("15"));
			viewer->addLine(om_corners_xyz_[i]->at(2), om_corners_xyz_[i]->at(6), r, g, b, cname + std::string("26"));
			viewer->addLine(om_corners_xyz_[i]->at(3), om_corners_xyz_[i]->at(7), r, g, b, cname + std::string("37"));
			viewer->addLine(om_corners_xyz_[i]->at(4), om_corners_xyz_[i]->at(5), r, g, b, cname + std::string("45"));
			viewer->addLine(om_corners_xyz_[i]->at(5), om_corners_xyz_[i]->at(6), r, g, b, cname + std::string("56"));
			viewer->addLine(om_corners_xyz_[i]->at(6), om_corners_xyz_[i]->at(7), r, g, b, cname + std::string("67"));
			viewer->addLine(om_corners_xyz_[i]->at(7), om_corners_xyz_[i]->at(4), r, g, b, cname + std::string("74"));
		}//: for

		previous_om_bb_size = om_corners_xyz_.size();
	} else {
		previous_om_bb_size = 0;
	}//: else
}



void CloudViewer::refreshModelsSceneCorrespondences(std::vector<pcl::CorrespondencesPtr> models_scene_correspondences_, pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift_, std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> om_clouds_xyzsift_) {
	CLOG(LTRACE) << "refreshModelsSceneCorrespondences";

	CLOG(LDEBUG) << "Size of correspondences=" << models_scene_correspondences_.size() << " Size of om clouds=" << om_clouds_xyzsift_.size();

	// Remove correspondences - TODO: FIX (but how?).
	for(int i = 0; i < previous_oms_correspondences_size; i++){
		ostringstream s;
		s << i;
		std::string cname = "correspondences_" + s.str();

		viewer->removeCorrespondences(cname) ;
	}//: else

	if (prop_display_objects_scene_correspondences) {
        if(om_clouds_xyzsift_.size() != models_scene_correspondences_.size()){
            CLOG(LERROR) << "Wrong correspondences size";
            return;
        }
		// Generate colour vector for MAX of om clouds (sifts, corners, correspondences,names).
		resizeColourVector(models_scene_correspondences_.size());

		// Display correspondences.
		for (size_t i = 0; i<models_scene_correspondences_.size(); ++i) {
			CLOG(LDEBUG) << "Adding " << i << "-th model-scene correspondences";

			// Get i-th model cloud.previous_om_object_bounding_boxes_xyz_size
			pcl::CorrespondencesPtr correspondences = models_scene_correspondences_[i];
			// Generate given correspondences set name prefix.
			std::ostringstream s;
			s << i;
			std::string cname = "correspondences_" + s.str();

			// Get colour from list.
			double r = colours[i][0];
			double g = colours[i][1];
			double b = colours[i][2];

			viewer->addCorrespondences<PointXYZSIFT>((om_clouds_xyzsift_[i]), scene_cloud_xyzsift_, *correspondences, cname) ;
			viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cname) ;
		}//: for

		previous_oms_correspondences_size = models_scene_correspondences_.size();
	} else {
		previous_oms_correspondences_size = 0;
	}//: else
}

void CloudViewer::refreshOMIds(std::vector<std::string>  om_ids_, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  om_corners_xyz_) {
	CLOG(LTRACE) << "refreshOMNames";

	// Remove previous names.
	for(int i = 0; i < previous_om_labels_size; i++){
		ostringstream s;
		s << i;
		std::string cname = "name_" + s.str();

		viewer->removeText3D(cname) ;
	}//: else

	if (prop_display_object_labels) {

		// Generate colour vector for MAX of om clouds (sifts, corners, correspondences,names).
		resizeColourVector(om_ids_.size());

		// Display correspondences.
		for (size_t i = 0; i<om_ids_.size(); ++i) {
			CLOG(LDEBUG) << "Adding " << i << "-th o/m name";

			// Get i-th model name.
			std::string om_id = om_ids_[i];
			// Generate name.
			std::ostringstream s;
			s << i;
			std::string cname = "name_" + s.str();

			// Get colour from list.
			double r = colours[i][0];
			double g = colours[i][1];
			double b = colours[i][2];

			// Create and accumulate points
			pcl::CentroidPoint<pcl::PointXYZ> centroid;
			for (size_t j = 0; j < 8; ++j) {
				centroid.add( om_corners_xyz_[i]->at(j) );
			}//: for
			// Fetch centroid using `get()`
			pcl::PointXYZ om_center;
			centroid.get (om_center);

			viewer->addText3D ( om_id, om_center, prop_label_scale, r, g, b, cname);
		}//: for

		previous_om_labels_size = om_ids_.size();
	} else {
		previous_om_labels_size = 0;
	}//: else

}


void CloudViewer::refreshOMCoordinateSystems(std::vector<Types::HomogMatrix> om_poses_) {
	CLOG(LTRACE) << "refreshOMCoordinateSystems";

	// Remove previous names.
	for(int i = 0; i < previous_om_coordinate_systems_size; i++){
		ostringstream s;
		s << i;
		std::string cname = "coordinate_system_" + s.str();

		viewer->removeCoordinateSystem (cname);

	}//: else

	if (prop_display_object_coordinate_systems) {

		// Display correspondences.
		for (size_t i = 0; i<om_poses_.size(); ++i) {
			CLOG(LDEBUG) << "Adding " << i << "-th o/m coordinate system";

			// Generate name.
			std::ostringstream s;
			s << i;
			std::string cname = "coordinate_system_" + s.str();

			Eigen::Affine3f pose = om_poses_[i];

			viewer->addCoordinateSystem (prop_coordinate_system_scale, pose, cname);
		}//: for

		previous_om_coordinate_systems_size = om_poses_.size();
	} else {
		previous_om_coordinate_systems_size = 0;
	}//: else

}





void CloudViewer::resizeColourVector(unsigned int size_) {
	CLOG(LTRACE) << "resizeColourVector";

	CLOG(LDEBUG) << "at start: max size_ =" << size_ << "colours.size() =" << colours.size();
	// If too small.
	while (colours.size() < size_) {
		// Add random colour.
		double* rgb = new double(3);
		for (size_t i = 0; i < 3; ++i) {
			rgb[i] = (double)(rand()&255)/255.0;
		}//: for
		CLOG(LDEBUG) << "rgb =" << rgb[0] << "," << rgb[1] << "," << rgb[2];
		colours.push_back(rgb);
	}//: while
	// If too big.
	/*while (colours.size() > size_) {
		// Remove last colour from list.
		colours.pop_back();
	}//: while*/
	CLOG(LDEBUG) << "at end: max size_ =" << size_ << "colours.size() =" << colours.size();
}



/*

void CloudViewer::displayClouds_xyzrgb_normals() {
	CLOG(LTRACE) << "displayClouds_xyzrgb_normals";

	if (!prop_display_xyznormals) {
		viewer->removePointCloud ("xyzrgb");
		viewer->removePointCloud ("xyzrgbnormals");
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
		//viewer->removeAllPointClouds();
		//viewer->removeAllShapes();

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudRgb);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloudRgb, rgb, "xyzrgb");
		viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloudRgb, normals, prop_xyznormals_level, prop_xyznormals_scale, "xyzrgbnormals");
	}//: else
}
*/


} //: namespace CloudViewer
} //: namespace Processors
