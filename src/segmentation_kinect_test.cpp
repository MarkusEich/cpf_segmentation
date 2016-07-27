#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <segmentation/segmentation.hpp>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/bilateral.h>

using namespace std;
using namespace APC;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// PCL viewer //
//pcl::visualization::PCLVisualizer seg_viewer("PCL Segmentation Viewer");
//pcl::visualization::PCLVisualizer viewer("PCL RBGD Viewer");

// Mutex: //
boost::mutex cloud_mutex;

enum { COLS = 640, ROWS = 480 };

int print_help()
{
  cout << "*******************************************************" << std::endl;
  cout << "Ground based people detection app options:" << std::endl;
  cout << "   --help    <show_this_help>" << std::endl;
  cout << "   --svm     <path_to_svm_file>" << std::endl;
  cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>" << std::endl;
  cout << "   --min_h   <minimum_person_height (default = 1.3)>" << std::endl;
  cout << "   --max_h   <maximum_person_height (default = 2.3)>" << std::endl;
  cout << "*******************************************************" << std::endl;
  return 0;
}

void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud,
    bool* new_cloud_available_flag)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};
  
void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main (int argc, char** argv)
{
  if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
        return print_help();


  Segmentation seg;
  Config config;
  seg.setConfig(config);


  // Read Kinect live stream:
  PointCloudT::Ptr cloud (new PointCloudT);
  bool new_cloud_available_flag = false;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
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

      // Draw cloud and people bounding boxes in the viewer:
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
