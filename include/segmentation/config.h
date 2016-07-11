#ifndef CONFIG
#define CONFIG

#endif // CONFIG

namespace APC {


class Config{

public:

    Config():voxel_resolution(0.003f),
        seed_resolution(0.05f),
        color_importance (1.0f),
        spatial_importance (0.4f),
        normal_importance  (1.0f),
        use_single_cam_transform (false),
        use_supervoxel_refinement (false),

        // Default parameters for model fitting
        use_random_sampling (false),
        outlier_cost(0.02f),
        smooth_cost (outlier_cost*0.01),
        min_inliers_per_plane (10),
        label_cost (min_inliers_per_plane*0.5*outlier_cost),
        max_num_iterations (25),
        max_curvature (0.001f),
        gc_scale (1e4){}

    public:

    float voxel_resolution;
    float seed_resolution;
    float color_importance;
    float spatial_importance;
    float normal_importance;
    bool use_single_cam_transform;
    bool use_supervoxel_refinement;

    // Default parameters for model fitting
    bool use_random_sampling;
    float outlier_cost;
    float smooth_cost;
    int min_inliers_per_plane;
    float label_cost;
    int max_num_iterations;
    float max_curvature;
    int gc_scale;


    Config& operator=(const Config& config){

        voxel_resolution=config.voxel_resolution;
        seed_resolution=config.seed_resolution;
        color_importance=config.color_importance;
        spatial_importance=config.spatial_importance;
        normal_importance=config.normal_importance;
        use_single_cam_transform=config.use_single_cam_transform;
        use_supervoxel_refinement=config.use_supervoxel_refinement;

        use_random_sampling=config.use_random_sampling;
        outlier_cost=config.outlier_cost;
        smooth_cost=config.smooth_cost;
        min_inliers_per_plane=config.min_inliers_per_plane;
        label_cost=config.label_cost;
        max_num_iterations=config.max_num_iterations;
        max_curvature=config.max_curvature;
        gc_scale=config.gc_scale;

    }

};

}

