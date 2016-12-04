/*
* Copyright (C) 2016, Australian Centre for Robotic Vision, ACRV
* All rights reserved.
*
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Osnabrück University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*  Created on: 11.07.2016
*
*      Authors:
*         Trung T. Pham <trung.pham@adelaide.edu.au>
*         Markus Eich <markus.eich@qut.edu.au>
*
*
*/

#ifndef __segmentation__hpp
#define __segmentation__hpp

// Stdlib
#include <stdlib.h>
#include <cmath>
#include <limits.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>


// PCL input/output
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

// PCL other
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// Segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// VTK
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

// Graph cuts
#include <gco-v3.0/GCoptimization.h>

// LSA TR Optimisation
#include <segmentation/lsa_tr.h>

#include <segmentation/segmentation_helpers.h>

// Boost
#include <boost/format.hpp>
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

// Eigen
#include <Eigen/Core>


#include <segmentation/config.h>

# define M_PI 3.14159265358979323846  /* pi */

/// *****  Type Definitions ***** ///
typedef pcl::PointXYZRGBA PointT;  // The point type used for input


namespace APC {


  class Segmentation{

  public:

    /**
    * @brief Segmentation
    */
    Segmentation();

    /**
    * @brief setConfig
    * @param config
    */
    void setConfig(const Config& config){config_=config;}

    /**
    * @brief getConfig
    * @return
    */
    Config getConfig(){return config_;}

    /**
    * @brief setPointCloud
    * @param input_cloud_ptr
    */
    void setPointCloud(pcl::PointCloud<PointT>::Ptr input_cloud_ptr){input_cloud_ptr_=input_cloud_ptr;}

    /**
    * @brief doSegmentation Main function for segmentation
    */
    void doSegmentation();

    /**
    * @brief segmentedPointCloud
    * @return the segmented point cloud
    */
    pcl::PointCloud<pcl::PointXYZL>::Ptr getSegmentedPointCloud(){return segmented_cloud_ptr_;}


  private:

    Config config_;

    pcl::PointCloud<PointT>::Ptr input_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud_ptr_;


  };

}

#endif
