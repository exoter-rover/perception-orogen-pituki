#ifndef pituki_TYPES_HPP
#define pituki_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace pituki {

    enum OutlierFilterType
    {
        NONE,
        STATISTICAL,
        RADIUS
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
}

#endif

