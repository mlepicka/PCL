/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef CLOUDTRANSFORMER_HPP_
#define CLOUDTRANSFORMER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>
#include <Types/PointXYZKAZE.hpp>
#include <Types/HomogMatrix.hpp>

namespace Processors {
namespace CloudTransformer {

/*!
 * \class CloudTransformer
 * \brief CloudTransformer processor class.
 *
 * 
 */
class CloudTransformer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CloudTransformer(const std::string & name = "CloudTransformer");

	/*!
	 * Destructor
	 */
	virtual ~CloudTransformer();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyz;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyzrgb;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyzsift;
	Base::DataStreamIn<pcl::PointCloud<PointXYZKAZE>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyzkaze;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyzshot;
	Base::DataStreamIn<Types::HomogMatrix, Base::DataStreamBuffer::Newest> in_hm;

    Base::DataStreamIn<vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, Base::DataStreamBuffer::Newest> in_clouds_xyz;
    Base::DataStreamIn<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, Base::DataStreamBuffer::Newest> in_clouds_xyzrgb;
    Base::DataStreamIn<vector<pcl::PointCloud<PointXYZSIFT>::Ptr>, Base::DataStreamBuffer::Newest> in_clouds_xyzsift;
    Base::DataStreamIn<vector<pcl::PointCloud<PointXYZKAZE>::Ptr>, Base::DataStreamBuffer::Newest> in_clouds_xyzkaze;
    Base::DataStreamIn<vector<pcl::PointCloud<PointXYZSHOT>::Ptr>, Base::DataStreamBuffer::Newest> in_clouds_xyzshot;
    Base::DataStreamIn<vector<Types::HomogMatrix>, Base::DataStreamBuffer::Newest> in_hms;

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSHOT>::Ptr> out_cloud_xyzshot;
	Base::DataStreamOut<pcl::PointCloud<PointXYZKAZE>::Ptr> out_cloud_xyzkaze;
	
    Base::DataStreamOut<vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > out_clouds_xyz;
    Base::DataStreamOut<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > out_clouds_xyzrgb;
    Base::DataStreamOut<vector<pcl::PointCloud<PointXYZSIFT>::Ptr> > out_clouds_xyzsift;
    Base::DataStreamOut<vector<pcl::PointCloud<PointXYZSHOT>::Ptr> > out_clouds_xyzshot;
	Base::DataStreamOut<vector<pcl::PointCloud<PointXYZKAZE>::Ptr> > out_clouds_xyzkaze;
	
	// Handlers
	void transform_clouds();
    void transform_vector_of_clouds();


    // Helper functions, specialized for every cloud type.
	void transform_xyz(Types::HomogMatrix hm_);
	void transform_xyzrgb(Types::HomogMatrix hm_);
	void transform_xyzsift(Types::HomogMatrix hm_);
	void transform_xyzkaze(Types::HomogMatrix hm_);
	void transform_xyzshot(Types::HomogMatrix hm_);

    void transform_vector_xyz(vector<Types::HomogMatrix> hms_);
    void transform_vector_xyzrgb(vector<Types::HomogMatrix> hms_);
    void transform_vector_xyzsift(vector<Types::HomogMatrix> hms_);
    void transform_vector_xyzkaze(vector<Types::HomogMatrix> hms_);
    void transform_vector_xyzshot(vector<Types::HomogMatrix> hms_);

    /// Property: if true, the component will not transform the input cloud.
    Base::Property<bool> pass_through;

    /// Property: inverse transformation(s).
    Base::Property<bool> inverse;

};

} //: namespace CloudTransformer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudTransformer", Processors::CloudTransformer::CloudTransformer)

#endif /* CLOUDTRANSFORMER_HPP_ */
