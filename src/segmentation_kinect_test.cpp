/*
 * Copyright (C) 2016, Australian Centre for Robotic Vision, ACRV
 * All rights reserved.
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
 *  Last update: 20 Oct 2016
 */


#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <segmentation/segmentation.hpp>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/bilateral.h>

using namespace std;
using namespace APC;

//typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Mutex: //
boost::mutex cloud_mutex;

enum { COLS = 640, ROWS = 480 };

void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud,
    bool* new_cloud_available_flag)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}

int main (int argc, char** argv)
{

  Segmentation seg;
  Config config;
  config.voxel_resolution = 0.001; // 1 cm
  config.seed_resolution = 0.05; // 5 cm
  config.use_single_cam_transform = true;
  config.min_inliers_per_plane = 25;
  config.noise_threshold = 0.01;
  seg.setConfig(config);


  // Read Kinect live stream:
  PointCloudT::Ptr cloud (new PointCloudT);
  bool new_cloud_available_flag = false;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f =
      boost::bind (&cloud_cb_, _1, cloud, &new_cloud_available_flag);
  interface->registerCallback (f);
  interface->start ();

  // Wait for the first frame:
  while(!new_cloud_available_flag)
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  new_cloud_available_flag = false;

  // Initialize new viewer:
  pcl::visualization::PCLVisualizer seg_viewer("PCL Segmentation Viewer");          // viewer initialization
  seg_viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  pcl::visualization::PCLVisualizer viewer("PCL RGBD Viewer");          // viewer initialization
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Main loop:
  while (!seg_viewer.wasStopped())
  {
    if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
    {
      new_cloud_available_flag = false;
      viewer.removeAllPointClouds();
      viewer.removeAllShapes();

      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);

      pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
      pcl::FastBilateralFilter<PointT> bFilter;
      bFilter.setInputCloud(cloud);
      bFilter.setSigmaS(5);
      bFilter.setSigmaR(0.2);
      bFilter.applyFilter(*cloud_filtered);

      viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
      viewer.spinOnce();

      seg.setPointCloud(cloud_filtered);
      seg.doSegmentation();
      pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud_ptr;
      segmented_cloud_ptr=seg.getSegmentedPointCloud();

      pcl::PointCloud<pcl::PointXYZL> segmented_cloud2;
      pcl::copyPointCloud(*segmented_cloud_ptr, segmented_cloud2);
      segmented_cloud2.clear();

      BOOST_FOREACH (pcl::PointXYZL point, *segmented_cloud_ptr) {
      	if (point.label == 0) continue;
        segmented_cloud2.push_back(point);
      }
      pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud2_ptr = segmented_cloud2.makeShared();
      seg_viewer.removeAllPointClouds();
      seg_viewer.removeAllShapes();
      seg_viewer.addPointCloud (segmented_cloud2_ptr, "segmented point cloud");
      seg_viewer.spinOnce();

      cloud_mutex.unlock ();
    }
  }

  return 0;
}
