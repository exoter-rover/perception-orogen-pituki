/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <fstream>

using namespace pituki;

#define DEBUG_PRINTS 1

Task::Task(std::string const& name)
    : TaskBase(name)
{
    sensor_point_cloud.reset(new PCLPointCloud);
    merge_point_cloud.reset(new PCLPointCloud);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
    sensor_point_cloud.reset();
    merge_point_cloud.reset();
}

void Task::pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
    this->tf_pose_samples = pose_samples_sample.getTransform();
    return;
}

void Task::point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"[PITUKI] Received pituki point clouds\n";
    #endif

    /** The transformation from the transformer **/
    Eigen::Affine3d tf;

    /** Get the transformation (transformation) **/
    if (_pose_samples.connected())
    {
        tf = this->tf_pose_samples;
    }
    else if ((_sensor_frame.value().compare(_world_frame.value()) == 0) && (!_pose_samples.connected()))
    {
        tf.setIdentity();
    }
    else if(!_sensor2world.get( ts, tf ) && (_pose_samples.connected()))
    {
        tf = tf * this->tf_pose_samples;
    }
    else if(!_sensor2world.get( ts, tf ) && (!_pose_samples.connected()))
    {
        RTT::log(RTT::Warning)<<"[PITUKI FATAL ERROR] No transformation provided for the transformer."<<RTT::endlog();
        return;
    }

    if (this->idx >= this->samples_count)
    {
        /** Reset counter **/
        this->idx = 0;

        /** Convert to pcl point clouds **/
        this->toPCLPointCloud(point_cloud_samples_sample, *sensor_point_cloud.get());
        sensor_point_cloud->height = static_cast<int>(_sensor_point_cloud_height.value());
        sensor_point_cloud->width = static_cast<int>(_sensor_point_cloud_width.value());
        std::cout<<"sensor_point_cloud->width: "<< sensor_point_cloud->width<<"\n";
        std::cout<<"sensor_point_cloud->height: "<< sensor_point_cloud->height<<"\n";
        std::cout<<"sensor_point_cloud->size: "<< sensor_point_cloud->size()<<"\n";

        /** Temporal point cloud **/
        PCLPointCloudPtr tmp_point_cloud;

        /** Conditional Removal in sensor **/
        if (_sensor_conditional_removal_config.value().filter_on)
        {
            tmp_point_cloud.reset(new PCLPointCloud(*sensor_point_cloud));
            this->conditionalRemoval(tmp_point_cloud, _sensor_conditional_removal_config.value(), sensor_point_cloud);
        }

        /** Bilateral filter **/
        if (this->bfilter_config.filter_on)
        {
            tmp_point_cloud.reset(new PCLPointCloud(*sensor_point_cloud));
            this->bilateral_filter(tmp_point_cloud, this->bfilter_config, sensor_point_cloud);
        }

        /** Outlier removal **/
        if (this->outlierfilter_config.type != NONE)
        {
            this->outlierRemoval(sensor_point_cloud, this->outlierfilter_config, sensor_point_cloud);
        }

        tmp_point_cloud.reset();
        tmp_point_cloud = NULL;

        /** Remove NaN **/
        std::vector<int> indices;
        PCLPointCloudPtr unorganized_point_cloud(new PCLPointCloud);
        pcl::removeNaNFromPointCloud(*sensor_point_cloud, *unorganized_point_cloud, indices); 

        /** Transform the point cloud in world frame **/
        this->transformPointCloud(*unorganized_point_cloud, tf);

        /** Conditional Removal in complete point cloud **/
        if (_map_conditional_removal_config.value().filter_on)
        {
            this->conditionalRemoval(unorganized_point_cloud, _map_conditional_removal_config.value(), sensor_point_cloud);
        }
        else
        {
            *sensor_point_cloud = *unorganized_point_cloud;
        }

        /** Accumulate the cloud points **/
        PCLPointCloudPtr combine_point_cloud;
        combine_point_cloud.reset(new PCLPointCloud);
        *combine_point_cloud = *merge_point_cloud + *sensor_point_cloud;

        /** Downsample the point cloud **/
        this->downsample(combine_point_cloud, _downsample_size.value(), merge_point_cloud);
        #ifdef DEBUG_PRINTS
        std::cout<<"[PITUKI] Finished Downsample\n";
        std::cout<<"merge_point_cloud->width: "<< merge_point_cloud->width<<"\n";
        std::cout<<"merge_point_cloud->height: "<< merge_point_cloud->height<<"\n";
        std::cout<<"merge_point_cloud->size: "<< merge_point_cloud->size()<<"\n";
        #endif
        combine_point_cloud.reset();
        combine_point_cloud = NULL;

        /** Write the point cloud into the port **/
        ::envire::core::SpatioTemporal<pcl::PCLPointCloud2> point_cloud_out;
        pcl::toPCLPointCloud2(*merge_point_cloud.get(), point_cloud_out.data);
        point_cloud_out.time = point_cloud_samples_sample.time;
        _point_cloud_samples_out.write(point_cloud_out);
    }

    this->idx++;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    this->idx = 0;

    this->bfilter_config = _bfilter_config.get();
    this->outlierfilter_config = _outlierfilter_config.get();

    this->tf_pose_samples.setIdentity();

    if (_input_point_cloud_period.value() < _point_cloud_samples_period.value())
    {
        _input_point_cloud_period.value() = _point_cloud_samples_period.value();
    }

    this->samples_count = round(_input_point_cloud_period.value()/_point_cloud_samples_period.value());

    std::cout<<"this->samples_count: "<<this->samples_count<<"\n";

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();

    pcl::io::savePLYFileBinary (_output_ply.value(), *merge_point_cloud.get());
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

void Task::writePlyFile( const base::samples::Pointcloud& points, const std::string& file )
{
    std::ofstream data( file.c_str() );

    data << "ply" << "\n";
    data << "format ascii 1.0\n";

    data << "element vertex " << points.points.size() <<  "\n";
    data << "property float x\n";
    data << "property float y\n";
    data << "property float z\n";

    if( !points.colors.empty() )
    {
	data << "property uchar red\n";
	data << "property uchar green\n";
	data << "property uchar blue\n";
	data << "property uchar alpha\n";
    }
    data << "end_header\n";

    for( size_t i = 0; i < points.points.size(); i++ )
    {
	data 
	    << points.points[i].x() << " "
	    << points.points[i].y() << " "
	    << points.points[i].z() << " ";
	if( !points.colors.empty() )
	{
	    data 
		<< (int)(points.colors[i].x()*255) << " "
		<< (int)(points.colors[i].y()*255) << " "
		<< (int)(points.colors[i].z()*255) << " "
		<< (int)(points.colors[i].w()*255) << " ";
	}
	data << "\n";
    }
}


void Task::toPCLPointCloud(const ::base::samples::Pointcloud & pc, pcl::PointCloud< PointType >& pcl_pc, double density)
{
    pcl_pc.clear();
    std::vector<bool> mask;
    unsigned sample_count = (unsigned)(density * pc.points.size());

    if(density <= 0.0 || pc.points.size() == 0)
    {
        return;
    }
    else if(sample_count >= pc.points.size())
    {
        mask.resize(pc.points.size(), true);
    }
    else
    {
        mask.resize(pc.points.size(), false);
        unsigned samples_drawn = 0;

        while(samples_drawn < sample_count)
        {
            unsigned index = rand() % pc.points.size();
            if(mask[index] == false)
            {
                mask[index] = true;
                samples_drawn++;
            }
        }
    }

    for(size_t i = 0; i < pc.points.size(); ++i)
    {
        if(mask[i])
        {
            PointType pcl_point;
            pcl_point.x = pc.points[i].x();
            pcl_point.y = pc.points[i].y();
            pcl_point.z = pc.points[i].z();
            uint8_t r = pc.colors[i].x()*255.00, g = pc.colors[i].y()*255.00, b = pc.colors[i].z()*255.00;
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            pcl_point.rgb = *reinterpret_cast<float*>(&rgb);

            /** Point info **/
            pcl_pc.push_back(pcl_point);
        }
    }

    /** All data points are finite (no NaN or Infinite) **/
    pcl_pc.is_dense = false;
}

void Task::fromPCLPointCloud(::base::samples::Pointcloud & pc, const pcl::PointCloud< PointType >& pcl_pc, double density)
{
    std::vector<bool> mask;
    unsigned sample_count = (unsigned)(density * pcl_pc.size());

    if(density <= 0.0 || pcl_pc.size() == 0)
    {
        return;
    }
    else if(sample_count >= pcl_pc.size())
    {
        mask.resize(pcl_pc.size(), true);
    }
    else
    {
        mask.resize(pcl_pc.size(), false);
        unsigned samples_drawn = 0;

        while(samples_drawn < sample_count)
        {
            unsigned index = rand() % pcl_pc.size();
            if(mask[index] == false)
            {
                mask[index] = true;
                samples_drawn++;
            }
        }
    }

    for(size_t i = 0; i < pcl_pc.size(); ++i)
    {
        if(mask[i])
        {
            PointType const &pcl_point(pcl_pc.points[i]);

            /** Position **/
            pc.points.push_back(::base::Point(pcl_point.x, pcl_point.y, pcl_point.z));

            /** Color **/
            uint32_t rgb = *reinterpret_cast<const int*>(&pcl_point.rgb);
            uint8_t r = (rgb >> 16) & 0x0000ff;
            uint8_t g = (rgb >> 8)  & 0x0000ff;
            uint8_t b = (rgb)       & 0x0000ff;
            pc.colors.push_back(::base::Vector4d(r, g, b, 255.0)/255.00);
        }
    }
}

void Task::transformPointCloud(const ::base::samples::Pointcloud & pc, ::base::samples::Pointcloud & transformed_pc, const Eigen::Affine3d& transformation)
{
    transformed_pc.points.clear();
    for(std::vector< ::base::Point >::const_iterator it = pc.points.begin(); it != pc.points.end(); it++)
    {
        transformed_pc.points.push_back(transformation * (*it));
    }
}

void Task::transformPointCloud(::base::samples::Pointcloud & pc, const Eigen::Affine3d& transformation)
{
    for(std::vector< ::base::Point >::iterator it = pc.points.begin(); it != pc.points.end(); it++)
    {
        *it = (transformation * (*it));
    }
}

void Task::transformPointCloud(pcl::PointCloud<PointType> &pcl_pc, const Eigen::Affine3d& transformation)
{
    for(std::vector< PointType, Eigen::aligned_allocator<PointType> >::iterator it = pcl_pc.begin(); it != pcl_pc.end(); it++)
    {
        Eigen::Vector3d point (it->x, it->y, it->z);
        point = transformation * point;
        PointType pcl_point;
        pcl_point.x = point[0]; pcl_point.y = point[1]; pcl_point.z = point[2];
        pcl_point.rgb = it->rgb;
        *it = pcl_point;
    }
}

void Task::downsample (const PCLPointCloudPtr &points, const ::base::Vector3d &leaf_size, PCLPointCloudPtr &downsampled_out)
{

  pcl::VoxelGrid<PointType> vox_grid;
  vox_grid.setLeafSize (leaf_size[0], leaf_size[1], leaf_size[2]);
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);

  return;
}

void Task::compute_surface_normals (PCLPointCloudPtr &points, float normal_radius,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
  pcl::NormalEstimation<PointType, pcl::Normal> norm_est;

  // Use a FLANN-based KdTree to perform neighborhood searches
  //norm_est.setSearchMethod (pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>));
  norm_est.setSearchMethod (pcl::search::KdTree<PointType>::Ptr (new pcl::search::KdTree<PointType>));


  // Specify the size of the local neighborhood to use when computing the surface normals
  norm_est.setRadiusSearch (normal_radius);

  // Set the input points
  norm_est.setInputCloud (points);

  // Estimate the surface normals and store the result in "normals_out"
  norm_est.compute (*normals_out);
}

void Task::bilateral_filter(const PCLPointCloudPtr &points, const pituki::BilateralFilterConfiguration &config, PCLPointCloudPtr &filtered_out)
{
    pcl::FastBilateralFilter<PointType> b_filter;

    /** Configure Bilateral filter **/
    b_filter.setSigmaS(config.spatial_width);
    b_filter.setSigmaR(config.range_sigma);

    b_filter.setInputCloud(points);
    b_filter.filter(*filtered_out);
    #ifdef DEBUG_PRINTS
    std::cout<<"[PITUKI] Finished Bilateral Filter\n";
    #endif
}

void Task::outlierRemoval(const PCLPointCloudPtr &points, const pituki::OutlierRemovalFilterConfiguration &config, PCLPointCloudPtr &outliersampled_out)
{
    if (config.type == STATISTICAL)
    {
        pcl::StatisticalOutlierRemoval<PointType> sor;

        sor.setMeanK(config.parameter_one);
        sor.setStddevMulThresh(config.parameter_two);

        #ifdef DEBUG_PRINTS
        std::cout<<"STATISTICAL FILTER\n";
        #endif
        outliersampled_out->width = points->width;
        outliersampled_out->height = points->height;
        sor.setInputCloud(points);
        sor.filter (*outliersampled_out);
    }
    else if (config.type == RADIUS)
    {
        pcl::RadiusOutlierRemoval<PointType> ror;

        ror.setRadiusSearch(config.parameter_one);
        ror.setMinNeighborsInRadius(config.parameter_two);

        #ifdef DEBUG_PRINTS
        std::cout<<"RADIUS FILTER\n";
        #endif
        PCLPointCloud filtered_cloud;
        outliersampled_out->width = points->width;
        outliersampled_out->height = points->height;
        ror.setInputCloud(points);
        ror.filter (*outliersampled_out);
    }

    return;
}

void Task::conditionalRemoval(const PCLPointCloudPtr &points, const pituki::ConditionalRemovalConfiguration &config, PCLPointCloudPtr &outliersampled_out)
{
    /** Clean the out point cloud **/
    outliersampled_out->clear();

    /**  build the condition **/
    pcl::ConditionAnd<PointType>::Ptr range_cond (new
      pcl::ConditionAnd<PointType> ());

    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
      pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::GT, config.gt_boundary[0])));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
      pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::LT, config.lt_boundary[0])));

    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
      pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::GT, config.gt_boundary[1])));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
      pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::LT, config.lt_boundary[1])));

    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
      pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::GT, config.gt_boundary[2])));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
      pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::LT, config.lt_boundary[2])));

    /** Apply the condition filter **/
    pcl::ConditionalRemoval<PointType> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (points);
    condrem.setKeepOrganized(config.keep_organized);
    condrem.filter (*outliersampled_out);
}

void Task::compute_PFH_features (PCLPointCloud::Ptr &points,
                      pcl::PointCloud<pcl::Normal>::Ptr &normals,
                      float feature_radius,
                      pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
{
    // Create a PFHEstimation object
    pcl::PFHEstimation<PointType, pcl::Normal, pcl::PFHSignature125> pfh_est;

    // Set it to use a FLANN-based KdTree to perform its neighborhood searches
    pfh_est.setSearchMethod (pcl::search::KdTree<PointType>::Ptr (new pcl::search::KdTree<PointType>));

    // Specify the radius of the PFH feature
    pfh_est.setRadiusSearch (feature_radius);

    // Set the input points and surface normals
    pfh_est.setInputCloud (points);
    pfh_est.setInputNormals (normals);

    // Compute the features
    pfh_est.compute (*descriptors_out);

    return;
}

void Task::detect_keypoints (PCLPointCloud::Ptr &points,
          float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast,
          pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_out)
{
    pcl::SIFTKeypoint<PointType, pcl::PointWithScale> sift_detect;

    // Use a FLANN-based KdTree to perform neighborhood searches
    sift_detect.setSearchMethod (pcl::search::KdTree<PointType>::Ptr (new pcl::search::KdTree<PointType>));

    // Set the detection parameters
    sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
    sift_detect.setMinimumContrast (min_contrast);

    // Set the input
    sift_detect.setInputCloud (points);

    // Detect the keypoints and store them in "keypoints_out"
    sift_detect.compute (*keypoints_out);

    return;
}

void Task::compute_PFH_features_at_keypoints (PCLPointCloud::Ptr &points,
                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
                           pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
{
    // Create a PFHEstimation object
    pcl::PFHEstimation<PointType, pcl::Normal, pcl::PFHSignature125> pfh_est;

    // Set it to use a FLANN-based KdTree to perform its neighborhood searches
    pfh_est.setSearchMethod (pcl::search::KdTree<PointType>::Ptr (new pcl::search::KdTree<PointType>));

    // Specify the radius of the PFH feature
    pfh_est.setRadiusSearch (feature_radius);

    /* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
    * use them as an input to our PFH estimation, which expects clouds of PointXYZRGBA points.  To get around this,
    * we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to
    * "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGBA>).  Note that the original cloud doesn't have any RGB
    * values, so when we copy from PointWithScale to PointXYZRGBA, the new r,g,b fields will all be zero.
    */

    PCLPointCloudPtr keypoints_xyzrgb (new pcl::PointCloud<PointType>);
    pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);

    // Use all of the points for analyzing the local structure of the cloud
    pfh_est.setSearchSurface (points);
    pfh_est.setInputNormals (normals);

    // But only compute features at the keypoints
    pfh_est.setInputCloud (keypoints_xyzrgb);

    // Compute the features
    pfh_est.compute (*descriptors_out);

    return;
}

void Task::find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                      pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                      std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{

    // Resize the output vector
    correspondences_out.resize (source_descriptors->size ());
    correspondence_scores_out.resize (source_descriptors->size ());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);
    for (size_t i = 0; i < source_descriptors->size (); ++i)
    {
        descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }

    return;
}

