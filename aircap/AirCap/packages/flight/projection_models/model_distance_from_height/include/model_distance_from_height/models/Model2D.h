//
// Created by glawless on 10.05.17.
//

#ifndef MODEL_DISTANCE_FROM_HEIGHT_MODEL2D_H
#define MODEL_DISTANCE_FROM_HEIGHT_MODEL2D_H

#include "ModelBase.h"

namespace model_distance_from_height {

class Model2D : public ModelBase {
  /// Implements a basic 2d model for computing the distance to an object, given its height
  /// Works fully on the Y-Z camera plane

  //TODO: Not finished!

protected:

  struct height_model_simple height_model_;

  double fy_;   // focal length in pixels
  double cy_;   // camera central point

public:
  Model2D(){};
  Model2D(int fy, int cy, double hmean);

  double compute_distance(double pitch, double height_pixels, double center_pixels);
};
}


#endif //MODEL_DISTANCE_FROM_HEIGHT_MODEL2D_H
