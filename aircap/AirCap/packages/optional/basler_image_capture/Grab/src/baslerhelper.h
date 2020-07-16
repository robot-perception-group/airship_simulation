/*
 * TODO: Add apropriate License and copyright header
 */

#ifndef BASLERHELPER_H
#define BASLERHELPER_H

#include "boost/thread/mutex.hpp"
#include "frameptr.h"

namespace videograb {
class baslerhelper_priv;

class baslerhelper {
public:
    baslerhelper(int framerate);
    ~baslerhelper();
    frameptr getFrame (void);
    int getHeight(void);
    int getWidth(void);

private:
    baslerhelper_priv *instance;
};

}

#endif // ifndef BASLERHELPER_H
