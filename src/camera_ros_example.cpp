#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"
#include "gl_depth_sim/interfaces/pcl_interface.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <chrono>

#include "gl_depth_sim/interfaces/opencv_interface.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include  <random_numbers/random_numbers.h>


//using namespace random_numbers;

static Eigen::Affine3d lookat(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up)
{
  Eigen::Vector3d z = (eye - origin).normalized();
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Affine3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}


/* 
//generating random number in Range [-0.5, 0.5] in meter
static double randNumPos()
{
  /* generate random Position and orientation of the mesh
  int minVal = -5; // gesucht ist -0.5 Meter
  int maxVal = 5; // gesucht ist +0.5
  int range = maxVal - minVal + 1;
  float validPosNum = rand() % range + (minVal);
  float validPosNum2 = validPosNum/10;
  float validPosNum = minVal + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxVal-minVal)));
  return validPosNum2;  
  /
  random_numbers::RandomNumberGenerator randObjPos = random_numbers::RandomNumberGenerator();
  double No = randObjPos.uniformReal(-5, 5);
  return No/10;
}
*/

/*
static double* randOri()
{
  random_numbers::RandomNumberGenerator randObjOri = random_numbers::RandomNumberGenerator();
  double oriMat[4];
  oriMat[0] = (randObjOri.uniformReal(-1, 1))/10;
  oriMat[1] = (randObjOri.uniformReal(-1, 1))/10;
  oriMat[2] = (randObjOri.uniformReal(-1, 1))/10;
  oriMat[3] = (randObjOri.uniformReal(-1, 1))/10;
  randObjOri.quaternion(oriMat);
  return oriMat;
}
*/

/* generate random Position and orientation of the mesh 
Position of 
/**/
static Eigen::Affine3d genRanPose()
{
  Eigen::Affine3d objPose(Eigen::Affine3d::Identity());
  random_numbers::RandomNumberGenerator randObjPose = random_numbers::RandomNumberGenerator();
  int low_bound, up_bound;
  low_bound = -5;
  up_bound = 5;
  double x,y,z;
  x = randObjPose.uniformReal(low_bound,up_bound)/10;
  y = randObjPose.uniformReal(low_bound,up_bound)/10;
  z = randObjPose.uniformReal(low_bound,up_bound)/10;
  objPose.translate(Eigen::Vector3d(x, y, z));
  //objPose.translate(Eigen::Vector3d(0, 0, 0));
    
 double quat[3];
 // quat = randOri();
 // std::cout<<  "quat: " << quat << "\n";

    //objPose.Quaterniond(const Scalar& quat[0], const Scalar& quat[1], const Scalar& quat[2], const Scalar& quat[3]);

/*  
  Eigen::Quaternion<double> q;
  q.x() = quat[0];
  q.y() = quat[1];
  q.z() = quat[2];
  q.w() = quat[3];
  q.normalize();
 // Eigen::Matrix3d R = q.normalized().toRotationMatrix();
 */
  //objPose.rotate(Eigen::Quaterniond( quat[0], 0, 0, 0));
  //objPose.rotate(Eigen::Quaterniond( 0, quat[0], 0, 0));
  //objPose.rotate(Eigen::Quaterniond( 0, 0, quat[0], 0));
  //objPose.rotate(Eigen::Quaterniond( 0, 0, 0, quat[0]));
  //objPose.rotate(Eigen::Quaterniond( quat[3], quat[0], quat[1], quat[2]));
  //Eigen::Quaternion<double> q(quat[3], quat[0], quat[1], quat[2]);
  
  objPose.rotate(Eigen::Quaterniond(0, 0, 0, 0));
  //objPose.rotate(Eigen::Quaterniond(q));
  //objPose.rotate(q.toRotationMatrix());
  //objPose.rotate(Eigen::Quaternion<double>(quat));
  return objPose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_depth_sim_orbit");
  ros::NodeHandle nh, pnh ("~");

  // Setup ROS interfaces
  ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1, true);

  tf::TransformBroadcaster broadcaster;

  // Load ROS parameters
  std::string mesh_path;
  if (!pnh.getParam("mesh", mesh_path))
  {
    ROS_ERROR_STREAM("User must set the 'mesh' private parameter");
    return 1;
  }

  std::string base_frame = pnh.param<std::string>("base_frame", "world");
  std::string camera_frame = pnh.param<std::string>("camera_frame", "camera");

  double radius = pnh.param<double>("radius", 1.0);
  double z = pnh.param<double>("z", 1.0);

  double focal_length = pnh.param<double>("focal_length", 550.0);
  int width = pnh.param<int>("width", 640);
  int height = pnh.param<int>("height", 480);

  auto mesh_ptr = gl_depth_sim::loadMesh(mesh_path);

  if (!mesh_ptr)
  {
    ROS_ERROR_STREAM("Unable to load mesh from path: " << mesh_path);
    return 1;
  }

  gl_depth_sim::CameraProperties props;
  props.width = width;
  props.height = height;
  props.fx = focal_length;
  props.fy = focal_length;
  props.cx = props.width / 2;
  props.cy = props.height / 2;
  props.z_near = 0.25;
  props.z_far = 10.0f;

  // Create the simulation
  gl_depth_sim::SimDepthCamera sim (props);
  Eigen::Affine3d meshPos = genRanPose();
  sim.add(*mesh_ptr, meshPos);


  // State for FPS monitoring
  long frame_counter = 0;
  // In the main (rendering) thread, begin orbiting...
  const auto start = std::chrono::steady_clock::now();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = camera_frame;

  ros::init(argc, argv, "depth_Img_Pub");
  ros::NodeHandle nhd;
  image_transport::ImageTransport it(nhd);
  image_transport::Publisher depth_pub = it.advertise("vCam/depth/image", 1);

  while (ros::ok())
  {
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    //Eigen::Vector3d camera_pos (radius * cos(dt),
    //                            radius * sin(dt),
    //                            z);

    Eigen::Vector3d camera_pos (radius,
                                radius,
                                z);  

    Eigen::Vector3d look_at (0, 0, 0);

    const auto pose = lookat(camera_pos, look_at, Eigen::Vector3d(0,0,1));

    const auto depth_img = sim.render(pose);

    //double *quat2;
    //quat2 = randOri();

    frame_counter++;


    if (frame_counter % 100 == 0)
    {
      std::cout << "FPS: " << frame_counter / dt << "\n";
      //std::cout<<  "q0: " << quat2[0] << "\n";
      //std::cout<<  "q1: " << quat2[1] << "\n";
      //std::cout<<  "q2: " << quat2[2] << "\n";
      //std::cout<<  "q3: " << quat2[3] << "\n";
    }

    // Step 1: Publish the cloud
    gl_depth_sim::toPointCloudXYZ(props, depth_img, cloud);
    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
    cloud_pub.publish(cloud);

    // Step 2: Publish the TF so we can see it in RViz
    tf::Transform transform;
    tf::transformEigenToTF(pose, transform);
    tf::StampedTransform stamped_transform (transform, ros::Time::now(), base_frame, camera_frame);
    broadcaster.sendTransform(stamped_transform);

    // Step 3: Publish the depth Image
    cv::Mat image(width, height, CV_16UC1);
    gl_depth_sim::toCvImage16u(depth_img, image);
    //cv::Mat image = cv::Mat(width, height, CV_64FC1, depth_img);
    cv::waitKey(30);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", image).toImageMsg();
    depth_pub.publish(msg);
    ros::Rate loop_rate(5);

    ros::spinOnce();
  }

  return 0;
}
