//
// Created by glawless on 10.05.17.
//

#include <model_distance_from_height/models/Model3D.h>

namespace model_distance_from_height{

  Model3D::Model3D(double hmean, double hvar) : height_model_(hmean, hvar) {
    ROS_ASSERT_MSG(hmean != 0, "The height model mean can't be 0");
  }

  double Model3D::compute_distance(const geometry_msgs::Point& p, double ymin, double delta_y) {
    //z == 1/2*(p3*real_person_height*ymin - p2*real_person_height)/(ycenter - ymin)
    //z = H*(p3*ymin - p2)/(height_pixels)
    //where height_pixels = ymax-ymin

    ROS_ASSERT(delta_y != 0.0);
    return height_model_.mean * (p.z * ymin - p.y) / delta_y;
  }

  double
  Model3D::compute_dist_var(const geometry_msgs::PoseWithCovariance &pose, const double delta_y, const double var_delta_y,
                            const double ymin, const double var_ymin, const double dist) {

    //TODO update this simplified model
    ROS_ASSERT(delta_y != 0.0);
    return dist*dist*(height_model_.var/height_model_.mean + var_delta_y/delta_y);
  }


}