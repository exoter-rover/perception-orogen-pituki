/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace pituki;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    sensor_pointcloud.reset(new PCLPointCloud);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
    sensor_pointcloud.reset();
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


    /** Convert to pcl point clouds **/
    this->toPCLPointCloud(point_cloud_samples_sample, *sensor_pointcloud.get());
    sensor_pointcloud->height = static_cast<int>(_sensor_point_cloud_height.value());
    sensor_pointcloud->width = static_cast<int>(_sensor_point_cloud_width.value());

    /**Integrate **/
    this->tsdf->integrateCloud (*sensor_pointcloud, pcl::PointCloud<pcl::Normal>(),  Eigen::Affine3d::Identity());

    std::vector<Eigen::Vector3i> indices;
    this->tsdf->getOccupiedVoxelIndices(indices);

    std::cout<<"indices.size(): "<<indices.size()<<"\n";

    /** Transform the point cloud in body frame **/
//    pointcloud.time = point_cloud_samples_sample.time;
//    pointcloud.points.resize(point_cloud_samples_sample.points.size());
//    pointcloud.colors = point_cloud_samples_sample.colors;
//    register int k = 0;
//    for (std::vector<base::Point>::const_iterator it = point_cloud_samples_sample.points.begin();
//        it != point_cloud_samples_sample.points.end(); it++)
//    {
//        pointcloud.points[k] = tf * (*it);
//        k++;
//    }

    /** Write the point cloud into the port **/
    ::base::samples::Pointcloud output_pc;
    //this->fromPCLPointCloud(output_pc, *sensor_pointcloud.get());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raytraced = this->tsdf->renderColoredView (); // Optionally can render it
    this->fromPCLPointCloud(output_pc, *raytraced.get());
    output_pc.time = point_cloud_samples_sample.time;
    _point_cloud_samples_out.write(output_pc);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Initialize TSDF Volume **/
    this->tsdf.reset(new cpu_tsdf::TSDFVolumeOctree());
    this->tsdf->setGridSize (10., 10., 10.); // 10m x 10m x 10m
    this->tsdf->setResolution (2048, 2048, 2048); // Smallest cell size = 10m / 2048 = about half a centimeter
    this->tsdf->setIntegrateColor (true); // Set to true if you want the TSDF to store color
    this->tsdf->reset();// initialize volume

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

    /** Save the mesh into a file **/
    cpu_tsdf::MarchingCubesTSDFOctree mc;
    mc.setInputTSDF (this->tsdf);
    //mc.setMinWeight (10);
    mc.setColorByRGB (true);
    pcl::PolygonMesh mesh;
    mc.reconstruct (mesh);
    pcl::io::savePLYFileBinary ("./mesh.ply", mesh);

    this->tsdf->save("./tsdf_volume.vol");

    std::cout<<"[STOP] SAVED MESH INTO FILE\n";
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

void Task::toPCLPointCloud(const ::base::samples::Pointcloud & pc, pcl::PointCloud< pcl::PointXYZRGBA >& pcl_pc, double density)
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
                pcl::PointXYZRGBA pcl_point;
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

void Task::fromPCLPointCloud(::base::samples::Pointcloud & pc, const pcl::PointCloud< pcl::PointXYZRGBNormal >& pcl_pc, double density)
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
            pcl::PointXYZRGBNormal const &pcl_point(pcl_pc.points[i]);

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

void Task::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGBA> &pcl_pc, const Eigen::Affine3d& transformation)
{
    for(std::vector< pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA> >::iterator it = pcl_pc.begin(); it != pcl_pc.end(); it++)
    {
        Eigen::Vector3d point (it->x, it->y, it->z);
        point = transformation * point;
        pcl::PointXYZRGBA pcl_point;
        pcl_point.x = point[0]; pcl_point.y = point[1]; pcl_point.z = point[2];
        *it = pcl_point;
    }
}

