/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace pituki;

#define DEBUG_PRINTS 1

void writePlyFile( const base::samples::Pointcloud& points, const std::string& file )
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

Task::Task(std::string const& name)
    : TaskBase(name)
{
    sensor_point_cloud.reset(new PCLPointCloud);
    merge_point_cloud.reset(new PCLPointCloud);
    viewer.reset(new pcl::visualization::CloudViewer("Simple Cloud Viewer"));
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
    sensor_point_cloud.reset();
}

void Task::point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample)
{
    /** Get the transformation from the transformer **/
    Eigen::Affine3d tf;

    std::cout<<"Received pituki point clouds\n";

    /** Get the transformation (transformation) Tbody_laser which transforms laser pints two body points **/
    if(!_body2navigation.get( ts, tf ))
    {
        RTT::log(RTT::Warning)<<"[ EXOTER POINT CLOUD FATAL ERROR] No transformation provided for the transformer."<<RTT::endlog();
        return;
    }

    /** Transform the point cloud in navigation frame **/
    ::base::samples::Pointcloud point_cloud_in;
    point_cloud_in = point_cloud_samples_sample;
    register int k = 0;
    for (std::vector<base::Point>::const_iterator it = point_cloud_samples_sample.points.begin();
        it != point_cloud_samples_sample.points.end(); it++)
    {
        point_cloud_in.points[k] = tf * (*it);
        k++;
    }

    /** Convert to pcl point clouds **/
    this->toPCLPointCloud(point_cloud_in, *sensor_point_cloud.get());
    sensor_point_cloud->height = static_cast<int>(_sensor_point_cloud_height.value());
    sensor_point_cloud->width = static_cast<int>(_sensor_point_cloud_width.value());

    /** Integrate Fields **/
    *merge_point_cloud += *sensor_point_cloud;

    #ifdef DEBUG_PRINTS
    std::cerr << "PointCloud before filtering: " << merge_point_cloud->width * merge_point_cloud->height
       << " data points (" << pcl::getFieldsList (*merge_point_cloud) << ").\n";
    #endif

    if (idx++ > 200)
    {
        // Create the filtering object
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud (merge_point_cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.1f);
        sor.filter (*merge_point_cloud);

        #ifdef DEBUG_PRINTS
        std::cerr << "PointCloud after filtering: " << merge_point_cloud->width * merge_point_cloud->height
           << " data points (" << pcl::getFieldsList (*merge_point_cloud) << ").\n";
        #endif

        /** Outlier filter in case of not NULL **/
        if (outlierfilter_config.type == STATISTICAL)
        {

            #ifdef DEBUG_PRINTS
            std::cout<<"STATISTICAL FILTER\n";
            #endif
            PCLPointCloud filtered_cloud;
            filtered_cloud.width = merge_point_cloud->width;
            filtered_cloud.height = merge_point_cloud->height;
            sor.setInputCloud(merge_point_cloud);
            sor.filter (filtered_cloud);
            merge_point_cloud = boost::make_shared<PCLPointCloud>(filtered_cloud);
        }
        else if (outlierfilter_config.type == RADIUS)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"RADIUS FILTER\n";
            #endif
            PCLPointCloud filtered_cloud;
            filtered_cloud.width = merge_point_cloud->width;
            filtered_cloud.height = merge_point_cloud->height;
            //ror.setInputCloud(merge_point_cloud);
            //ror.filter (filtered_cloud);
            //merge_point_cloud = boost::make_shared<PCLPointCloud>(filtered_cloud);
        }

        // Detect Features

        // Detect Keypoints

        idx = 0;
    }

    /** Write the point cloud into the port **/
    ::base::samples::Pointcloud point_cloud_out;
    this->fromPCLPointCloud(point_cloud_out, *merge_point_cloud.get());
    std::cerr<< "Base PointcCloud size: "<<point_cloud_out.points.size()<<"\n";
    point_cloud_out.time = point_cloud_samples_sample.time;
    _point_cloud_samples_out.write(point_cloud_out);

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    //viewer->showCloud (merge_point_cloud);

    this->idx=0;

    /** Configure Outlier filter **/
    if (outlierfilter_config.type == pituki::STATISTICAL)
    {
        sor.setMeanK(outlierfilter_config.parameter_one);
        sor.setStddevMulThresh(outlierfilter_config.parameter_two);
    }
    else if (outlierfilter_config.type == pituki::RADIUS)
    {
        ror.setRadiusSearch(outlierfilter_config.parameter_one);
        ror.setMinNeighborsInRadius(outlierfilter_config.parameter_two);
    }

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

    ::base::samples::Pointcloud point_cloud_out;
    this->fromPCLPointCloud(point_cloud_out, *merge_point_cloud.get());

	writePlyFile(point_cloud_out, _output_ply.value() );
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
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
            if (base::isnotnan<base::Point>(pc.points[i]))
            {
                PointType pcl_point;
                pcl_point.x = pc.points[i].x();
                pcl_point.y = pc.points[i].y();
                pcl_point.z = pc.points[i].z();
                uint8_t r = pc.colors[i].x()*255.00, g = pc.colors[i].y()*255.00, b = pc.colors[i].z()*255.00;
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                pcl_point.rgb = *reinterpret_cast<float*>(&rgb);

                if (rgb > 0.00)
                {
                    /** Point info **/
                    pcl_pc.push_back(pcl_point);
                }
            }
        }
    }

    /** All data points are finite (no NaN or Infinite) **/
    pcl_pc.is_dense = true;
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
        *it = pcl_point;
    }
}



