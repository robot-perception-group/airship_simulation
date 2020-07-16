//
// Created by glawless on 10.05.17.
//

#ifndef MODEL_DISTANCE_FROM_HEIGHT_MODELBASE_H
#define MODEL_DISTANCE_FROM_HEIGHT_MODELBASE_H

namespace model_distance_from_height {

struct height_model_simple{
  double mean;

  height_model_simple(double _mean) : mean(_mean){}
  height_model_simple() {};
};

struct height_model_uncertainty{
  double mean, var;

  height_model_uncertainty(double _mean, double _var) : mean(_mean), var(_var){};
  height_model_uncertainty() {};
};

class ModelBase {
  /// Every model will provide the distance to an object on the image given certain information about it
  /// Example of information is: height model, camera angle, detected height in pixels, object center in pixels
  /// Camera calibration should not be handled inside a Model object, but outside
protected:

public:
  //virtual double compute_distance() = 0;
  //virtual double compute_uncertainty() = 0;
};
}

#endif //MODEL_DISTANCE_FROM_HEIGHT_MODELBASE_H