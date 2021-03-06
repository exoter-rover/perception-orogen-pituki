#ifndef pituki_TYPES_HPP
#define pituki_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Eigen.hpp>

namespace pituki {

    enum OutlierFilterType
    {
        NONE,
        STATISTICAL,
        RADIUS
    };

    struct BilateralFilterConfiguration
    {
        bool filter_on;
        float spatial_width; //size of the window bilateral filter
        float range_sigma; // the standard deviation of the Gaussian for the intensity difference
    };

    struct OutlierRemovalFilterConfiguration
    {
        OutlierFilterType type;

        //STATISTICAL: the number of nearest neighbors to use for mean distance estimation (nr_k)
        //RADIUS: Get the radius of the sphere that will determine which points are neighbors (radiu).
        float parameter_one;

        //STATISTICAL: the standard deviation multiplier for the distance threshold calculation.(stddev_null)
        //RADIUS: number of neighbors that need to be present in order to be classified as an inlier(min_pts)
        float parameter_two;
    };

    struct TSDFConfiguration
    {
        base::Vector3d size;
        base::Vector3d resolution;
        bool integrate_color;

        int mesh_min_weight;
    };

    struct ConditionalRemovalConfiguration
    {
        bool filter_on;
        bool keep_organized;
        base::Vector3d gt_boundary;
        base::Vector3d lt_boundary;
    };
}

#endif

