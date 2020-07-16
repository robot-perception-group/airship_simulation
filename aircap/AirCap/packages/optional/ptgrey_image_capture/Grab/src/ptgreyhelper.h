/*
 * TODO: Add apropriate License and copyright header
 */

#ifndef PTGREYHELPER_H
#define PTGREYHELPER_H

#include "boost/thread/mutex.hpp"
#include "frameptr.h"

namespace videograb {
class ptgreyhelper_priv;

class ptgreyhelper {
public:
    ptgreyhelper(int framerate);
    ~ptgreyhelper();
    frameptr getFrame (uint8_t* buffer, size_t maxsize,int64_t minID);
    int getHeight(void);
    int getWidth(void);

private:
    ptgreyhelper_priv *instance;
};

}

#endif // ifndef PTGREYHELPER_H
