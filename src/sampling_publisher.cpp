#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "sampling_publisher");
  ros::NodeHandle nh;

  //Create posearray publisher
  ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseArray>("Pose_topic", 100, true);

  //Define PoseArray message
  geometry_msgs::PoseArray msg;
  msg.header.frame_id = "map";

  ROS_INFO("Starting sampling publisher");

  //Define Pose message
  geometry_msgs::Pose p;
  p.orientation.w = 1;

  //Allocate memory by appending p to pose array
  //msg.poses.push_back(p);

  while(ros::ok()){

// Input params here
    uint16_t nrx = 1;   double orientation_x_lower = 0.;
    uint16_t nry = 30;   double orientation_y_lower = 0.;
    uint16_t nrz = 30;   double orientation_z_lower = 0.;
    uint16_t ntx = 1;   double position_x_lower    = 0.;
    uint16_t nty = 1;   double position_y_lower    = 0.;
    uint16_t ntz = 1;   double position_z_lower    = 0.;

    double orient_increment = 0.1;
    double pos_increment    = 0.0025;

    double rx, ry, rz , tx, ty, tz;

    // Input sampling algorithm here
    for (size_t ii = 0; ii < nrx; ++ii)
    {
      rx = orientation_x_lower + orient_increment * ii;
      for (size_t jj = 0; jj < nry; ++jj)
      {
        ry = orientation_y_lower + orient_increment * jj;
        for (size_t kk = 0; kk < nrz; ++kk)
        {
          rz = orientation_z_lower + orient_increment * kk;
          for (size_t ll = 0; ll < ntx; ++ll)
          {
            tx = position_x_lower + pos_increment * ll;
            for (size_t mm = 0; mm < nty; ++mm)
            {
              ty = position_y_lower + pos_increment * mm;
              for (size_t nn = 0; nn < ntz; ++nn)
              {
                tz = position_z_lower + pos_increment * nn;

                /*              sampled_frame = Eigen::Translation3d(tx,ty,tz) *
                                  Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());*/
               // sampled_frame =
                 //   descartes_core::utils::toFrame(tx, ty, tz, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ);
                //rtn.push_back(sampled_frame);
                p.position.x = tx;
                p.position.y = ty;
                p.position.z = tz;

                tf::Quaternion q = tf::createQuaternionFromRPY(rx, ry, rz);
                p.orientation.x = q[0];
                p.orientation.y = q[1];
                p.orientation.z = q[2];
                p.orientation.w = q[3];

                msg.poses.push_back(p);
              }
            }
          }
        }
      }
    }
   // ROS_DEBUG_STREAM("Uniform sampling of frame, utilizing orientation increment: "
     //                << orient_increment << ", and position increment: " << pos_increment << " resulted in " << rtn.size()
       //              << " samples");



  //msg.poses[0].position.x += 0.5;
  pose_publisher.publish(msg);
  ROS_INFO("Poses Published");
  ros::Duration(1).sleep();
  //ros::spinOnce();
  ros::spin();
  }



}
