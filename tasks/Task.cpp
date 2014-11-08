/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace pituki;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample)
{
    /** Get the transformation from the transformer **/
    Eigen::Affine3d tf;

    /** Get the transformation (transformation) Tbody_laser which transforms laser pints two body points **/
    if(!_body2world.get( ts, tf ))
    {
        RTT::log(RTT::Warning)<<"[ EXOTER POINT CLOUD FATAL ERROR] No transformation provided for the transformer."<<RTT::endlog();
        return;
    }


    base::samples::Pointcloud pointcloud;

    /** Transform the point cloud in body frame **/
    pointcloud.time = point_cloud_samples_sample.time;
    pointcloud.points.resize(point_cloud_samples_sample.points.size());
    pointcloud.colors = point_cloud_samples_sample.colors;
    register int k = 0;
    for (std::vector<base::Point>::const_iterator it = point_cloud_samples_sample.points.begin();
        it != point_cloud_samples_sample.points.end(); it++)
    {
        pointcloud.points[k] = tf * (*it);
        k++;
    }

    /** Write the point cloud into the port **/
    _point_cloud_samples_out.write(pointcloud);

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
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
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
