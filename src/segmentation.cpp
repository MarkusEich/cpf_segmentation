#include <segmentation/segmentation.hpp>

using namespace APC;

Segmentation::Segmentation(){


}


void Segmentation::doSegmentation(){


      // Start the clock
    clock_t tStart = clock();

  /// Preparation of Input: Supervoxel Oversegmentation
    pcl::SupervoxelClustering<PointT> super (config_.voxel_resolution, config_.seed_resolution);
    super.setUseSingleCameraTransform (config_.use_single_cam_transform);
    super.setInputCloud (input_cloud_ptr_);
    super.setColorImportance (config_.color_importance);
    super.setSpatialImportance (config_.spatial_importance);
    super.setNormalImportance (config_.normal_importance);
      std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

      PCL_INFO ("Extracting supervoxels\n");
      super.extract (supervoxel_clusters);

    if (config_.use_supervoxel_refinement)
      {
          PCL_INFO ("Refining supervoxels\n");
          super.refineSupervoxels (2, supervoxel_clusters);
      }
      std::cout << "Number of supervoxels: " << supervoxel_clusters.size () << "\n";

      PCL_INFO ("Getting supervoxel adjacency\n");
      std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
      super.getSupervoxelAdjacency (supervoxel_adjacency);
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (supervoxel_clusters);
      std::cout << "Time taken at super-voxel segmentation: " << (double)(clock() - tStart)/CLOCKS_PER_SEC << "\n";


      /////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Constrained plane extraction
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* Generate plane hypotheses using super-voxels */
      std::vector<Eigen::Vector4f> planes_coeffs;
      std::vector<Eigen::Vector3f> planes_hough;
      double min_theta = 360; double max_theta = -360;
      double min_phi = 360; double max_phi = -360;
      double min_rho = 100; double max_rho = -100;

      std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator cluster_itr_c = supervoxel_clusters.begin();
      for (; cluster_itr_c != supervoxel_clusters.end(); cluster_itr_c++){
          pcl::Supervoxel<PointT>::Ptr sv = cluster_itr_c->second;
          pcl::PointCloud<PointT>::Ptr cloud = sv->voxels_;
          float curvature;
          Eigen::Vector4f plane_par;
          Eigen::Vector3f hough_par;
          pcl::computePointNormal(*cloud, plane_par, curvature);
        if (curvature < config_.max_curvature){

         // Convert to Hough transform
              double theta = std::atan(plane_par(1)/plane_par(0))*180/M_PI;
              double phi = std::acos(plane_par(2))*180/M_PI;
              double rho = plane_par(3);
              if (isnan(theta) | isnan(phi) | isnan(rho)) continue;
              hough_par(0) = theta;
              hough_par(1) = phi;
              hough_par(2) = rho;
              if (theta < min_theta) min_theta = theta;
              if (theta > max_theta) max_theta = theta;
              if (phi < min_phi) min_phi = phi;
              if (phi > max_phi) max_phi = phi;
              if (rho < min_rho) min_rho = rho;
              if (rho > max_rho) max_rho = rho;
              planes_hough.push_back(hough_par);
              planes_coeffs.push_back(plane_par);
          }
      }

    // Plane hypothesis generation using random sampling
    if (config_.use_random_sampling){
      int max_random_hyps = 100;
      int count = 0;
      int num_supervoxels = supervoxel_clusters.size ();
      srand(time(NULL));
      pcl::SampleConsensusModelPlane<pcl::PointNormal>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointNormal> (sv_centroid_normal_cloud));
      while (count < max_random_hyps){
          // random sample 4 points
          std::vector<int> samples;
          for (int s=0; s<3;s++) {
              samples.push_back((int)(rand()%num_supervoxels+1));
          }

          Eigen::VectorXf plane_par;
          model_p->computeModelCoefficients(samples, plane_par);
          if (plane_par.size() == 0) continue;

          std::set<int> test_points;
          test_points.insert((int)(rand()%num_supervoxels+1));
            bool good_model = model_p->doSamplesVerifyModel(test_points, plane_par, config_.outlier_cost);
          if (good_model == false) continue;

          Eigen::Vector3f hough_par;
          double theta = std::atan(plane_par(1)/plane_par(0))*180/M_PI;
          double phi = std::acos(plane_par(2))*180/M_PI;
          double rho = plane_par(3);

          if (isnan(theta) | isnan(phi) | isnan(rho)) continue;

          planes_coeffs.push_back(plane_par);
          hough_par(0) = theta;
          hough_par(1) = phi;
          hough_par(2) = rho;

          if (theta < min_theta) min_theta = theta;
          if (theta > max_theta) max_theta = theta;
          if (phi < min_phi) min_phi = phi;
          if (phi > max_phi) max_phi = phi;
          if (rho < min_rho) min_rho = rho;
          if (rho > max_rho) max_rho = rho;
          planes_hough.push_back(hough_par);
          count++;
      }
    }
    std::cout << "Num of plane hypothesis generated = " << planes_coeffs.size() << "\n";
    // Assign points to planes.
    uint32_t node_ID = 0;
    std::map<uint32_t, uint32_t> label2index;
    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator cluster_itr = supervoxel_clusters.begin();
    for (; cluster_itr != supervoxel_clusters.end(); cluster_itr++){
      label2index[cluster_itr->first] = node_ID;
      node_ID++;
    }
    uint32_t num_super_voxels = sv_centroid_normal_cloud->size();
    std::vector<int> sv_labels(num_super_voxels,0);
    int good_planes_count = 0;
    uint32_t outlier_label = 0;
    if (planes_coeffs.size() > 0){

      // Remove duplicated planes
      std::vector<Eigen::Vector4f> plane_candidates;

      double step_theta = 1;
      double step_phi = 1;
      double step_rho = 0.01;
      int theta_bins = round((max_theta - min_theta)/step_theta) + 1;
      int phi_bins = round((max_phi - min_phi)/step_phi) + 1;
      int rho_bins = round((max_rho - min_rho)/step_rho) + 1;

      unsigned char*** accumulator;
      accumulator = new unsigned char**[theta_bins];
      if (accumulator != NULL){
          for (int i=0; i<theta_bins; i++){
              accumulator[i] = new unsigned char*[phi_bins];
              if (accumulator[i] != NULL)
                  for (int j=0; j<phi_bins; j++)
                      accumulator[i][j] = new unsigned char[rho_bins];
              }
      }

      for (int i=0; i<planes_coeffs.size(); i++){
              Eigen::Vector3f hough_par = planes_hough.at(i);
              int b_theta = floor((hough_par(0) - min_theta + 0.001)/step_theta);
              int b_phi = floor((hough_par(1) - min_phi + 0.0001)/step_phi);
              int b_rho = floor((hough_par(2) - min_rho + 0.0001)/step_rho);
              if (accumulator[b_theta][b_phi][b_rho] != 1){
                  accumulator[b_theta][b_phi][b_rho] = 1;
                  plane_candidates.push_back(planes_coeffs.at(i));
              }
      }
      // Free accumulator memory
      for (int i = 0; i < theta_bins; i++){
          for (int j = 0; j < phi_bins; j++){
              delete [] accumulator[i][j];
          }
          delete [] accumulator[i];
      }
      delete [] accumulator;

      std::cout << "Number of planes remained after hough-based filtering = " << plane_candidates.size() << "\n";

          // Compute plane unary costs
          std::vector<Eigen::Vector4f> good_planes;
          std::vector<Eigen::VectorXf> planes_inliers_idx;
          std::vector<float> unaries;
          uint32_t num_planes = plane_candidates.size();
          Eigen::MatrixXf inliers_mat(num_planes, num_super_voxels);
          Eigen::MatrixXf normals_mat(num_planes, num_super_voxels);
          Eigen::MatrixXf point2plane_mat(num_planes, num_super_voxels);

          for (size_t j = 0; j < num_planes; ++j){
              Eigen::Vector4f p_coeffs = plane_candidates.at(j);
              Eigen::VectorXf inliers_idx(num_super_voxels);
              Eigen::VectorXf point2plane(num_super_voxels);
              inliers_idx = Eigen::VectorXf::Zero(num_super_voxels);
              int inliers_count = 0;
              for (size_t i = 0; i < num_super_voxels; ++i){
                  pcl::PointXYZ p;
                  p.x = sv_centroid_normal_cloud->at(i).x;
                  p.y = sv_centroid_normal_cloud->at(i).y;
                  p.z = sv_centroid_normal_cloud->at(i).z;
                  float data_cost = pcl::pointToPlaneDistance(p,p_coeffs);
                  point2plane(i) = data_cost;
          if (data_cost <= config_.outlier_cost) { inliers_idx(i) = 1; inliers_count++;}
              }

        if (inliers_count >= config_.min_inliers_per_plane){
                inliers_mat.row(good_planes_count) = inliers_idx;
                  normals_mat.row(good_planes_count) << p_coeffs(0), p_coeffs(1), p_coeffs(2);
                  point2plane_mat.row(good_planes_count) = point2plane;
                  good_planes.push_back(p_coeffs);
                  planes_inliers_idx.push_back(inliers_idx);
          double u_cost = std::exp(-(double)(inliers_count - config_.min_inliers_per_plane)/(4*config_.min_inliers_per_plane)) - 1;

                  if (isnan(u_cost)){
                      PCL_WARN("NaN Unary \n");
                  }
                  else{
                      unaries.push_back(u_cost);
            good_planes_count++;
                  }

              }
          }

          uint32_t size_n = unaries.size();
      if (size_n > 1){

          inliers_mat.conservativeResize(good_planes_count, num_super_voxels);
          normals_mat.conservativeResize(good_planes_count, num_super_voxels);
          point2plane_mat.conservativeResize(good_planes_count, num_super_voxels);

          Eigen::VectorXf inliers_count = inliers_mat.rowwise().sum();
          Eigen::MatrixXf temp1 = inliers_mat.transpose();
          Eigen::MatrixXf ov_mat = inliers_mat*temp1;
          Eigen::MatrixXf temp2 = normals_mat.transpose();
          Eigen::MatrixXf dot_mat = normals_mat*temp2;

          Eigen::VectorXf plane_unaries = Eigen::Map<Eigen::MatrixXf> (unaries.data(), size_n, 1);
          Eigen::MatrixXf plane_pairwises = Eigen::MatrixXf::Zero(size_n, size_n);

          for (int i=0; i<size_n-1; i++){
              for (int j=i+1; j<size_n; j++){
                  double ov_cost = ov_mat(i,j)/std::min(inliers_count(i), inliers_count(j));
                  if (ov_cost > 0.1) ov_cost = 1.001;
                  double angle = acos(dot_mat(i,j))*180/3.14159265359;
                  if (isnan(angle))
                      angle = 45;
                  else
                      angle = std::min(angle, 180 - angle);
                  double angle_cost = 0;
              if (angle < 90 - angle){ // Parallel
                  angle_cost = 1 - std::exp(-angle/20);
              }else {
                  angle_cost = 1 - std::exp(-(90-angle)/20);
              }

              //if (ov_cost == 0) angle_cost = 0;
              double p_cost = 0.25*angle_cost + 0.5*ov_cost;
              if (isnan(p_cost)){
                  PCL_WARN("NaN Pairwise \n");
              }
              else{
                  plane_pairwises(i,j) = p_cost;
                  plane_pairwises(j,i) = p_cost;
              }

           }
        }

        Eigen::VectorXf initLabeling(size_n);

        initLabeling = Eigen::VectorXf::Ones(size_n);
        Eigen::VectorXf finalLabeling(size_n);
        double finalEnergy = 0;
        LSA_TR(&finalEnergy, &finalLabeling, size_n, plane_unaries, plane_pairwises, initLabeling);

        printf("Time taken after Constrained Plane Extraction: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
        if (finalEnergy == 0){
          PCL_WARN("Optimization got stuck \n");
          finalLabeling = Eigen::VectorXf::Ones(size_n);
        }

        int num_selected_planes = finalLabeling.sum();
        std::vector<Eigen::Vector4f> selected_planes;
        Eigen::MatrixXf unary_matrix(num_selected_planes + 1, num_super_voxels);
        int sidx = 0;
        for (int i=0; i<size_n; i++){
          if (finalLabeling(i) == 1) {
              selected_planes.push_back(good_planes.at(i));
                unary_matrix.row(sidx) = point2plane_mat.row(i)*config_.gc_scale;
              sidx++;
          }
        }

        // Outlier data cost
        int num_labels = num_selected_planes + 1;
        outlier_label = num_labels - 1;
        unary_matrix.row(outlier_label) = Eigen::VectorXf::Ones(num_super_voxels)*((config_.outlier_cost*config_.gc_scale));
        Eigen::MatrixXi unary_matrix_int = unary_matrix.cast<int>();
        int *data_cost = new int[num_super_voxels*num_labels];
        Eigen::Map<Eigen::MatrixXi>(data_cost, unary_matrix_int.rows(), unary_matrix_int.cols() ) = unary_matrix_int;
        GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(num_super_voxels, num_labels);
        gc->setDataCost(data_cost);
        std::cerr << "Creating graph with " << gc->numSites() << " nodes and " << gc->numLabels() << " labels \n";

        int *smooth = new int[num_labels*num_labels];
        std::memset(smooth,10, num_labels*num_labels*sizeof(int));
        std::fill_n(smooth, num_labels*num_labels, (int)(config_.smooth_cost*config_.gc_scale));
        for (int l = 0; l < num_labels; l++) smooth[l+l*num_labels] = 0;
          gc->setSmoothCost(smooth);

        std::multimap<uint32_t,uint32_t>::iterator adjacency_itr = supervoxel_adjacency.begin();
        for ( ; adjacency_itr != supervoxel_adjacency.end(); ++adjacency_itr)
        {
          uint32_t node1 = label2index[adjacency_itr->first];
          uint32_t node2 = label2index[adjacency_itr->second];
          gc->setNeighbors(node1,node2,1);
        }

        gc->expansion(config_.max_num_iterations);
        for (int i=0; i<num_super_voxels;i++){
            sv_labels.at(i) = gc->whatLabel(i);
        }
        // Free some memory
        delete gc;
        delete smooth;
        delete data_cost;
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // End of constrained plane extraction
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
      }
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Segment the point cloud into objects
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace boost;
std::vector<uint32_t> supervoxel_labels;
{
  typedef adjacency_list <vecS, vecS, undirectedS> Graph;

  Graph G;
  for (uint32_t i = 0; i < supervoxel_clusters.size(); ++i){
      add_vertex(G);
  }

  std::multimap<uint32_t,uint32_t>::iterator adjacency_itr = supervoxel_adjacency.begin();
  for ( ; adjacency_itr != supervoxel_adjacency.end(); ++adjacency_itr)
  {
      uint32_t from = label2index[adjacency_itr->first];
      uint32_t to = label2index[adjacency_itr->second];

    uint32_t label_from = sv_labels.at(from);
    uint32_t label_to = sv_labels.at(to);

      Eigen::Vector3f p1;
      p1[0] = sv_centroid_normal_cloud->at(from).x;
      p1[1] = sv_centroid_normal_cloud->at(from).y;
      p1[2] = sv_centroid_normal_cloud->at(from).z;

      Eigen::Vector3f n1;
      n1[0] = sv_centroid_normal_cloud->at(from).normal_x;
      n1[1] = sv_centroid_normal_cloud->at(from).normal_y;
      n1[2] = sv_centroid_normal_cloud->at(from).normal_z;

      Eigen::Vector3f p2;
      p2[0] = sv_centroid_normal_cloud->at(to).x;
      p2[1] = sv_centroid_normal_cloud->at(to).y;
      p2[2] = sv_centroid_normal_cloud->at(to).z;

      Eigen::Vector3f n2;
      n2[0] = sv_centroid_normal_cloud->at(to).normal_x;
      n2[1] = sv_centroid_normal_cloud->at(to).normal_y;
      n2[2] = sv_centroid_normal_cloud->at(to).normal_z;
      if (label_from != label_to) continue;
      if (label_from == label_to && label_from != outlier_label){
          add_edge(from,to,G);
          continue;
      }

      bool convex = isConvex(p1, n1, p2, n2);
      if (convex == false) continue;
      add_edge(from,to,G);
  }

  std::vector<uint32_t> component(num_vertices(G));
  uint32_t num = connected_components(G, &component[0]);
  std::cout << "Number of connected components: " << num <<"\n";

  int min_voxels_per_cluster = 2;
  int outlier_label = 0;
  std::map<uint32_t,uint32_t> label_list_map;
  int new_label = 1;
  for (uint32_t i = 0; i != component.size(); ++i){
      int count = std::count(component.begin(), component.end(), component[i]);
      int label = component[i];
      if (label_list_map.find(label) == label_list_map.end() && count >= min_voxels_per_cluster){ // label not found
          label_list_map[label] = new_label;
          new_label++;
      }
      if (count < min_voxels_per_cluster){ // minimum number of supervoxels in each component
          supervoxel_labels.push_back(outlier_label);
      }
      else
          supervoxel_labels.push_back(label_list_map.find(label)->second);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// End of scene segmentation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Re-label the point cloud
segmented_cloud_ptr_ = super.getLabeledCloud ();
std::cerr << "Re-label supervoxel\n";
std::map<uint32_t,uint32_t> label_to_seg_map;
std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator cluster_itr_ = supervoxel_clusters.begin();
uint32_t idx = 0;
for (; cluster_itr_ != supervoxel_clusters.end(); cluster_itr_++){
  label_to_seg_map[cluster_itr_->first] = supervoxel_labels.at(idx);
  idx++;
}
typename pcl::PointCloud<pcl::PointXYZL>::iterator point_itr = (*segmented_cloud_ptr_).begin();
uint32_t zero_label = 0;
for (; point_itr != (*segmented_cloud_ptr_).end(); ++point_itr)
{
  if (point_itr->label == 0){
      zero_label++;
  }else{
      point_itr->label = label_to_seg_map[point_itr->label];
  }
}

printf("All Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);


}
/// END main

/// -------------------------| Definitions of helper functions|-------------------------
