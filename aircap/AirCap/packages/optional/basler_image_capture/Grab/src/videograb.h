/*
 * TODO: Add apropriate License and copyright header
 */

#ifndef VIDEOGRAB_H
#define VIDEOGRAB_H

#include "ros/ros.h"
#include "boost/thread/mutex.hpp"

namespace videograb {
class videograb_priv;

class videograb {
public:
    videograb(int argc, char * *argv);
    ~videograb();
    int run(void);
    boost::posix_time::ptime *getStart(void);
    boost::posix_time::ptime *getCurrent(void);
    void rosinfoPrint(const char *bla);

private:
    videograb_priv *instance;
};

}

#endif // ifndef VIDEOGRAB_H
