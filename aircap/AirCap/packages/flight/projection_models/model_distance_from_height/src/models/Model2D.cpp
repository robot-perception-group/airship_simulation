//
// Created by glawless on 10.05.17.
//

#include <ros/ros.h>
#include <model_distance_from_height/models/Model2D.h>

namespace model_distance_from_height{

  Model2D::Model2D(int fy, int cy, double hmean) : fy_(fy), cy_(cy), height_model_(hmean){};

  double Model2D::compute_distance(double pitch, double height_pixels, double center_pixels){
    return fy_ * height_model_.mean * cos(pitch) / height_pixels * sqrt(1 + (center_pixels-cy_)/fy_ );
  }

}