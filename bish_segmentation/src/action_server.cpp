#include "action_server.h"

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

// custom
#include <bish_segmentation_msgs/BishSegmentAction.h>
#include <bish_segmentation/ViewGenerator.h>
#include <bish_segmentation/PointcloudLabeler.h>
#include <bish_segmentation/batchProcessor.h>

// STL
#include <memory>
#include <vector>
#include <stdint.h>
#include <cstring>

// boost
#include <boost/thread.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

class BishSegmentationActionServer
{
public:
  typedef actionlib::SimpleActionServer<bish_segmentation_msgs::BishSegmentAction> ActionServer;
  typedef std::shared_ptr<ActionServer> ActionServerPtr;
  typedef pcl::PointCloud<pcl::PointSurfel> PointSurfelCloud;
  typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > Affine3dVector;
  typedef ViewGenerator::GLUintVectorVector GLUintVectorVector;
  typedef ViewGenerator::GLFloatVectorVector GLFloatVectorVector;
  typedef uint32_t uint32;
  typedef uint64_t uint64;
  typedef std::vector<uint32> Uint32Vector;
  typedef std::shared_ptr<ViewGenerator> ViewGeneratorPtr;
  typedef pcl::PointCloud<pcl::PointXYZRGBA> PointXYZRGBACloud;

  BishSegmentationActionServer(ros::NodeHandle & nh): m_nh(nh)
  {
    std::string param_string;
    int param_int;

    m_nh.param<std::string>(PARAM_NAME_ACTION_NAME, param_string, PARAM_DEFAULT_ACTION_NAME);
    m_action_server.reset(new ActionServer(m_nh, param_string,
                          boost::bind(&BishSegmentationActionServer::onAction, this, _1), false));



    m_nh.param<int>("test_segment_number", param_int, 50); // number of bands for image to be segmented
    m_test_segment_number = param_int;

    m_nh.param<std::string>("results_path", m_results_path, "~/data/results/");
    m_nh.param<std::string>("test_images_path", m_tpath, "~/data/test_images/");
    m_nh.param<std::string>("labeled_images_path", m_lpath, "~/data/labeled_images/");
    m_nh.param<std::string>("prototypes_path", m_protopath, "~/data/prototypes/");
    m_nh.param<std::string>("results_pointclouds_path", m_rppath, "~/data/results_pointclouds/");
    m_nh.param<std::string>("test_pointclouds_path", m_test_cloud_path, "~/data/test_pointclouds/");
  }

  bool Process(const bish_segmentation_msgs::BishSegmentGoalConstPtr goal,
               const bish_segmentation_msgs::BishSegmentResultPtr result_ptr)
  {
    PointSurfelCloud cloud;
    pcl::fromROSMsg(goal->cloud, cloud);

    for (pcl::PointSurfel & pt : cloud)
      pt.radius = 0.005;

    Affine3dVector view_poses;
    for (const geometry_msgs::Pose & pose : goal->view_poses)
    {
      Eigen::Affine3d p;
      tf::poseMsgToEigen(pose, p);
      view_poses.push_back(p);
    }

    bish_segmentation_msgs::BishSegmentResult & result = *result_ptr;

    std::vector<std::string> classdirs = getDirectoryNames(m_lpath.c_str());
    if(classdirs.size() == 0)
    {
      ROS_ERROR("Comparison dataset folder '%s' is empty", m_lpath.c_str());
      return false;
    }

    static uint64 id = 0;
    const std::string action_unique_name = "action" + std::to_string(id);
    id++;

    //recursiveDelete(m_results_path);

    std::string test_path = m_tpath + action_unique_name + "/";
    recursiveDelete(test_path);

    std::string test_cloud_name = m_test_cloud_path + action_unique_name + ".pcd";
    mkdir(m_test_cloud_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
    pcl::io::savePCDFileBinary(test_cloud_name, cloud);

    // scope only
    {
      ViewGeneratorPtr vg(new ViewGenerator(m_nh));
      vg->runMem(cloud, view_poses,
                 false, true, false, goal->back_face_culling,
                 m_tpath, action_unique_name);
    }

    std::shared_ptr<PointcloudLabeler> labeler(new PointcloudLabeler(m_nh));
    std::shared_ptr<batchProcessor> processor(new batchProcessor);

    processor->imageSimplify(test_path, m_test_segment_number);

    std::vector<std::tuple<std::string, std::string, unsigned int> > matching;
    std::string match_class;
    unsigned long int min_dist = ULONG_MAX;
    bool prototype_used = false;	//used to not redo the retrieval if the best was found directly in the dataset
    for(std::string class_name : classdirs)
    {
      if(!fileExists(m_results_path))
        mkdir(m_results_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
      std::string tmp_res_path = m_results_path + class_name;
      if(!fileExists(tmp_res_path))
        mkdir(tmp_res_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
      tmp_res_path += action_unique_name + "/";

      bool proto = true;
      std::string tmp_lpath = std::string(m_protopath).append(class_name);
      if(!fileExists(tmp_lpath) || getDirectoryNames(tmp_lpath.c_str()).size()==0)
      {
        ROS_WARN("Missing prototype for %s, searching in the dataset", class_name.c_str());
        tmp_lpath = std::string(m_lpath).append(class_name);
        proto = false;

        if(getDirectoryNames(tmp_lpath.c_str()).size() == 0)
        {
          ROS_WARN("No data neither prototype for class %s", class_name.c_str());
          continue;
        }
      }

      processor->batchRetrieval(test_path, tmp_lpath, tmp_res_path);

      unsigned long int total_dist;
      std::vector<std::tuple<std::string, std::string, unsigned int> > match =
        processor->readMatchedFile(tmp_res_path, &total_dist);

      if(total_dist >= min_dist)
        continue;

      min_dist = total_dist;
      matching = match;
      match_class = std::string(class_name);
      prototype_used = proto;
    }

    if(match_class.empty())
    {
      ROS_ERROR("No match found, check dataset and prototype");
      return false;
    }

    const std::string strip_match_class = match_class.back() == '/' ?
      match_class.substr(0, match_class.size() - 1) : match_class;
    ROS_INFO("bish_segmentation: matched as class \"%s\"", strip_match_class.c_str());

    if(prototype_used)
    {
      std::string tmp_lpath = std::string(m_lpath).append(match_class);
      std::string tmp_res_path = std::string(m_results_path).append(match_class);
      if(fileExists(tmp_res_path))
        recursiveDelete(tmp_res_path);
      mkdir(tmp_res_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
      tmp_res_path += action_unique_name + "/";

      processor->batchRetrieval(test_path, tmp_lpath, tmp_res_path);

      unsigned long int total_dist;
      matching = processor->readMatchedFile(tmp_res_path, &total_dist);
    }

    std::string labeled_path = std::string(m_lpath).append(match_class);
    std::string res_path = std::string(m_results_path).append(match_class);
    res_path += action_unique_name + "/";

    for (int i = 0; i < matching.size(); i++)
      processor->matchTransfer(labeled_path, test_path, res_path, std::get<0>(matching[i]), std::get<1>(matching[i]));

    const bool labeler_success = labeler->run(res_path, m_test_cloud_path, m_rppath, action_unique_name + ".pcd",
                                              test_path, labeled_path);
    if (!labeler_success)
      return false;

    PointXYZRGBACloud cloud_labeled;
    if (pcl::io::loadPCDFile(m_rppath + action_unique_name + "-2_LABELED.pcd", cloud_labeled))
      return false;

    // FILL RESULT
    for (uint64 i = 0; i < cloud.size(); i++)
      cloud[i].rgba = cloud_labeled[i].rgba;
    pcl::toROSMsg(cloud, result.segmented_cloud);
    result.segmented_class = strip_match_class;

    return true;
  }

  void Run()
  {
    boost::mutex::scoped_lock lock(m_mutex);
    m_action_server->start();
    ROS_INFO("bish_segmentation: action_server: processing thread started.");

    while (true)
    {
      while (!ros::isShuttingDown() && !m_current_goal)
        m_cond.wait_for(lock, boost::chrono::milliseconds(500));

      if (ros::isShuttingDown())
        return;

      bish_segmentation_msgs::BishSegmentGoalConstPtr goal = m_current_goal;
      m_current_goal.reset();
      ROS_INFO("bish_segmentation: action_server: processing goal.");

      lock.unlock();
      const bish_segmentation_msgs::BishSegmentResultPtr result(new bish_segmentation_msgs::BishSegmentResult);
      const bool success = Process(goal, result);
      lock.lock();
      m_current_success = success;
      m_current_result = result;

      ROS_INFO("bish_segmentation: action_server: processing goal complete.");
      m_cond.notify_all();
    }
  }

  void onAction(const bish_segmentation_msgs::BishSegmentGoalConstPtr goal)
  {
    ROS_INFO("bish_segmentation: action_server: action started.");
    boost::mutex::scoped_lock lock(m_mutex);

    m_current_result.reset();
    m_current_goal = goal;
    m_cond.notify_all();
    while (!ros::isShuttingDown() && !m_current_result)
      m_cond.wait_for(lock, boost::chrono::milliseconds(500));

    if (ros::isShuttingDown())
      return;

    if (!m_current_success)
      m_action_server->setAborted(*m_current_result);
    else
      m_action_server->setSucceeded(*m_current_result);
    m_current_result.reset();
    ROS_INFO("bish_segmentation: action_server: action complete (%s).", m_current_success ? "SUCCESS" : "ABORTED");
  }

private:
  ros::NodeHandle & m_nh;

  ActionServerPtr m_action_server;

  bish_segmentation_msgs::BishSegmentGoalConstPtr m_current_goal;
  bish_segmentation_msgs::BishSegmentResultConstPtr m_current_result;
  bool m_current_success;

  std::string m_tpath;
  std::string m_results_path;
  std::string m_lpath;
  std::string m_protopath;
  std::string m_rppath;
  std::string m_test_cloud_path;

  uint64 m_test_segment_number;

  boost::mutex m_mutex;
  boost::condition_variable m_cond;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bish_segmentation_action_server");

  ros::NodeHandle nh("~");
  BishSegmentationActionServer cpas(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  cpas.Run();

  return 0;
}
