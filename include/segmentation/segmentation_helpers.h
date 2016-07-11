/* Helper functions */

// Eigen
#include <Eigen/Core>

int isColorSimilar(Eigen::Vector3i c1, Eigen::Vector3i c2)
{
	Eigen::Vector3i c = c1 - c2;
	int d = c.norm();
	return d;
}

bool isConvex(Eigen::Vector3f p1, Eigen::Vector3f n1, Eigen::Vector3f p2, Eigen::Vector3f n2)
{
	float concavity_tolerance_threshold = 20;
	const Eigen::Vector3f& source_centroid = p1;
	const Eigen::Vector3f& target_centroid = p2;

	const Eigen::Vector3f& source_normal = n1;
	const Eigen::Vector3f& target_normal = n2;

    //NOTE For angles below 0 nothing will be merged
	if (concavity_tolerance_threshold < 0)
	{
		return (false);
	}

	bool is_convex = true;
	bool is_smooth = true;

	float normal_angle = pcl::getAngle3D(source_normal, target_normal, true);
    //  Geometric comparisons
	Eigen::Vector3f vec_t_to_s, vec_s_to_t;

	vec_t_to_s = source_centroid - target_centroid;
	vec_s_to_t = -vec_t_to_s;

	Eigen::Vector3f ncross;
	ncross = source_normal.cross (target_normal);

  	// Smoothness Check: Check if there is a step between adjacent patches
	bool use_smoothness_check = true;
	float seed_resolution = 0.1f; 
	float smoothness_threshold = 0.1;
	float voxel_resolution = 0.01;
	if (use_smoothness_check)
	{
		float expected_distance = ncross.norm () * seed_resolution;
		float dot_p_1 = vec_t_to_s.dot (source_normal);
		float dot_p_2 = vec_s_to_t.dot (target_normal);
		float point_dist = (std::fabs (dot_p_1) < std::fabs (dot_p_2)) ? std::fabs (dot_p_1) : std::fabs (dot_p_2);
    const float dist_smoothing = smoothness_threshold * voxel_resolution;  // This is a slacking variable especially important for patches with very similar normals

    if (point_dist > (expected_distance + dist_smoothing))
    {
    	is_smooth &= false;
    }
}

// Sanity Criterion: Check if definition convexity/concavity makes sense for connection of given patches
bool use_sanity_check = true;
float intersection_angle =  pcl::getAngle3D (ncross, vec_t_to_s, true);
float min_intersect_angle = (intersection_angle < 90.) ? intersection_angle : 180. - intersection_angle;

float intersect_thresh = 57. * 1. / (1. + exp (-0.3 * (normal_angle - 33.)));
if (min_intersect_angle < intersect_thresh && use_sanity_check)
{
    // std::cout << "Concave/Convex not defined for given case!" << std::endl;
	is_convex &= false;
}


  // vec_t_to_s is the reference direction for angle measurements
  // Convexity Criterion: Check if connection of patches is convex. If this is the case the two SuperVoxels should be merged.
if ((pcl::getAngle3D(vec_t_to_s, source_normal) - pcl::getAngle3D(vec_t_to_s, target_normal)) <= 0)
{
    is_convex &= true;  // connection convex
}
else
{
    is_convex &= (normal_angle < concavity_tolerance_threshold);  // concave connections will be accepted  if difference of normals is small
}
return (is_convex && is_smooth);

}
