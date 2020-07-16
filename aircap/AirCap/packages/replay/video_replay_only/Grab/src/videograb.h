/*
 * TODO: Add apropriate License and copyright header
 */

#ifndef VIDEOGRAB_H
#define VIDEOGRAB_H

#include "ros/ros.h"
#include "boost/thread/mutex.hpp"
#include <sensor_msgs/Image.h>

namespace videograb {
class videograb_priv;

class videograb {
public:
    videograb(int argc, char * *argv);
    ~videograb();
    int run(void);
    void rosinfoPrint(const char *bla);
    void videoCallback(const sensor_msgs::ImageConstPtr & msgp);

private:
    videograb_priv *instance;
};

}

#endif // ifndef VIDEOGRAB_H
