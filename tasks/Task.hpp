/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef PITUKI_TASK_TASK_HPP
#define PITUKI_TASK_TASK_HPP

/** PCL **/
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/pfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/io/ply_io.h>

/* Envire **/
#include <envire_core/items/Item.hpp>

/** SLAM MAPS **/
#include <maps/grid/TSDFVolumetricMap.hpp>
#include <maps/grid/OccupancyGridMap.hpp>
#include <maps/tools/TSDFPolygonMeshReconstruction.hpp>

#include <memory>

#include "pituki/TaskBase.hpp"

namespace pituki {

    typedef pcl::PointXYZ PointType;

    typedef pcl::PointCloud<PointType> PCLPointCloud;
    typedef typename PCLPointCloud::Ptr PCLPointCloudPtr;

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

    The corresponding C++ class can be edited in tasks/Task.hpp and
    tasks/Task.cpp, and will be put in the pituki namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','pituki::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        /**************************/
        /*** Property Variables ***/
        /**************************/
        pituki::BilateralFilterConfiguration bfilter_config; /** Bilateral filter Configuration **/
        pituki::OutlierRemovalFilterConfiguration outlierfilter_config; /** Outlier Filter Removal Configuration **/

        /***************************/
        /** Input port variables **/
        /***************************/
        int idx;
        PCLPointCloudPtr sensor_point_cloud;

        /*******************************/
        /** General Purpose variables **/
        /*******************************/
        PCLPointCloudPtr merge_point_cloud;
        //::maps::grid::TSDFVolumetricMap::Ptr tsdf_map;
        std::shared_ptr<::maps::grid::OccupancyGridMap> tsdf_map;

    protected:
        virtual void point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample);

    public:

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "pituki::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        void writePlyFile( const base::samples::Pointcloud& points, const std::string& file );

        void toPCLPointCloud(const ::base::samples::Pointcloud & pc, pcl::PointCloud< PointType >& pcl_pc, double density = 1.0);

        void fromPCLPointCloud(::base::samples::Pointcloud & pc, const pcl::PointCloud< PointType >& pcl_pc, double density = 1.0);

        void transformPointCloud(const ::base::samples::Pointcloud & pc, ::base::samples::Pointcloud & transformed_pc, const Eigen::Affine3d& transformation);

        void transformPointCloud(::base::samples::Pointcloud & pc, const Eigen::Affine3d& transformation);

        void transformPointCloud(PCLPointCloud &pcl_pc, const Eigen::Affine3d& transformation);

        void downsample(PCLPointCloudPtr &points, const ::base::Vector3d &leaf_size, PCLPointCloudPtr &downsampled_out);

        void compute_surface_normals (PCLPointCloudPtr &points, float normal_radius,  pcl::PointCloud<pcl::Normal>::Ptr &normals_out);

        void bilateral_filter(PCLPointCloudPtr &points, const pituki::BilateralFilterConfiguration &config, PCLPointCloudPtr &filtered_out);

        void outlierRemoval(PCLPointCloudPtr &points, const pituki::OutlierRemovalFilterConfiguration &config, PCLPointCloudPtr &outliersampled_out);

//        void compute_PFH_features(PCLPointCloud::Ptr &points,
//                      pcl::PointCloud<pcl::Normal>::Ptr &normals,
//                      float feature_radius,
//                      pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out);
//
//        void detect_keypoints(PCLPointCloud::Ptr &points,
//                  float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast,
//                  pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_out);
//
//        void compute_PFH_features_at_keypoints(PCLPointCloud::Ptr &points,
//                                   pcl::PointCloud<pcl::Normal>::Ptr &normals,
//                                   pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
//                                   pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out);
//
//        void find_feature_correspondences(pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
//                              pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
//                              std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out);

    };
}

#endif

