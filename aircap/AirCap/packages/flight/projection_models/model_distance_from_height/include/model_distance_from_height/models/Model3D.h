//
// Created by glawless on 10.05.17.
//

#ifndef MODEL_DISTANCE_FROM_HEIGHT_MODEL3D_H
#define MODEL_DISTANCE_FROM_HEIGHT_MODEL3D_H

#include "ModelBase.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <ros/assert.h>
#include <ros/static_assert.h>

namespace model_distance_from_height {

class Model3D : public ModelBase {
  /// Implements the 3d model for computing the distance to an object, given its height and center

public:
  struct height_model_uncertainty height_model_;

  Model3D(){};
  Model3D(double hmean, double hvar);

  double compute_distance(const geometry_msgs::Point& p, double ymin, double delta_y);
  double compute_dist_var(const geometry_msgs::PoseWithCovariance& pose, const double delta_y, const double var_delta_y, const double ymin, const double var_ymin, const double dist);
};
}


#endif //MODEL_DISTANCE_FROM_HEIGHT_MODEL3D_H
