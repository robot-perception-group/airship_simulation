/*
 * TODO: Add apropriate License and copyright header
 */

#ifndef REPLAY_H
#define REPLAY_H

#include "ros/ros.h"
#include "boost/thread/mutex.hpp"

namespace replay {
class replay_priv;

class replay {
public:
    replay(int argc, char * *argv);
    ~replay();
    int run(void);
    boost::posix_time::ptime *getStart(void);
    boost::posix_time::ptime *getCurrent(void);
    void rosinfoPrint(const char *bla);

private:
    replay_priv *instance;
    unsigned long long corrected_diff(boost::posix_time::time_duration diff);
};

}

#endif // ifndef REPLAY_H
