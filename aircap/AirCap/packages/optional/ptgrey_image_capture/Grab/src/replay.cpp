/*
 * TODO: Add apropriate License and copyright header
 */

#include "replay.h"
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include "frameptr.h"
#include <boost/filesystem/fstream.hpp>
#include <boost/thread.hpp>
#include <unistd.h>
extern "C" {
#include "libavhelper.h"
}

namespace replay {
replay *globalReplay;

/*
        void sigHandler(const boost::system::error_code& error, int signal_number) {
                // shutdown read and write threads too
                exit(-1);
                ros::shutdown();
        }
 */

class replay_priv {
public:
    int argc;
    char * *argv;
    ros::NodeHandle *nodehandle;
    boost::posix_time::ptime epoch;
    boost::posix_time::ptime start;
    boost::posix_time::ptime current;
    boost::mutex serial_Mutex;
    boost::mutex ROSinfo_Mutex;
    int width;
    int height;
    int nowait;
    double replayspeed;
    std::string videofilename;
    std::string timesfilename;
    std::string nameSpace;
    int threads;
    enum AVPixelFormat fmt;
    ros::Publisher frame_pub;
    boost::filesystem::ifstream timesfile;
    
    // ...
};

replay::replay(int argc, char * *argv)
{
    globalReplay = this;
    instance = new replay_priv;
    if (argc<6) {
        std::cerr << "Usage: " << std::string(argv[0]) << " <namespace> <videofile> <timesfile> <threads> <replayspeed> [-nowait] [ROS arguments*]\n";
	exit(1);
    }
    instance->threads=std::stoi(std::string(argv[4]));
    instance->replayspeed=std::stof(std::string(argv[5]));
    instance->epoch = boost::posix_time::ptime(boost::gregorian::date(1970,1,1));
    instance->start = boost::posix_time::microsec_clock::universal_time();
    instance->argc  = argc-5;
    instance->argv  = new char *[instance->argc];
    instance->argv[0] = argv[0];
    instance->nowait=0;
    if (argc>6) {
	if(std::string(argv[6])==std::string("-nowait")) instance->nowait=1;
    }
    int t;
    for (t=6;t<argc;t++) {
       instance->argv[t-5]=argv[t];
    }
    boost::posix_time::time_duration diff = instance->start - instance->epoch;
    instance->videofilename = argv[2];
    instance->timesfilename = argv[3];
    instance->nameSpace = argv[1];
    libavhelper_init_read(instance->videofilename.c_str(),&instance->width,&instance->height,&instance->fmt,instance->threads);
    ros::init(instance->argc,instance->argv,"replay");
    instance->nodehandle = new ros::NodeHandle;
    instance->frame_pub = instance->nodehandle->advertise<sensor_msgs::Image>(instance->nameSpace + "/video",10);
    instance->timesfile.open(instance->timesfilename);
}

replay::~replay()
{
    if (instance->nodehandle) {
        delete instance->nodehandle;
    }
    delete instance;
    libavhelper_close_read();
}

unsigned long long replay::corrected_diff(boost::posix_time::time_duration diff)
{
	return (unsigned long long)((double)diff.total_microseconds()*instance->replayspeed);
}

int replay::run(void)
{
    ros::init(instance->argc, instance->argv, "replay");

    instance->nodehandle = new ros::NodeHandle();
    // boost::asio::signal_set signals(instance->io_service, SIGINT, SIGTERM);


    // do everything here

    ros::AsyncSpinner spinner(4);
    spinner.start();
  
    int64_t frames=0;

    uint8_t *framebuffer =  new uint8_t[instance->width*instance->height*3];
    unsigned long long pts, time, starttime;
    instance->timesfile >> pts >> starttime;
    time = starttime;

    boost::posix_time::time_duration diff;

    int ret; 
    while (ros::ok()) {
	// read frame
	ret = libavhelper_read_frame(framebuffer,&pts);
        if (!ret) return 0; // end of video

	// sleep until frame is due 
	diff = boost::posix_time::microsec_clock::universal_time() - instance->start;
	unsigned long long offset = time-starttime;
	while (corrected_diff(diff)<offset) {
		if (offset-corrected_diff(diff)>2000000 && !instance->nowait) {
			rosinfoPrint( static_cast<std::ostringstream*>( &(std::ostringstream() <<  "Long gap: Sleeping for " << ((offset-corrected_diff(diff))/1000000.) << " seconds for frame " << pts << std::endl ))->str().c_str());
			diff = boost::posix_time::microsec_clock::universal_time() - instance->start;
		}
		unsigned long long sleep = offset-corrected_diff(diff);
		if (sleep>2000000) {
			sleep=2000000;
			if (instance->nowait) {
				sleep = 0;
				instance->start -= boost::posix_time::microseconds(2000000);
			}
		}
		boost::this_thread::sleep(boost::posix_time::microseconds(sleep));
		diff = boost::posix_time::microsec_clock::universal_time() - instance->start;
        }

	// send image to ros
	sensor_msgs::Image msg;
	msg.header.seq=pts;
	msg.header.stamp.sec=time / 1000000;
	msg.header.stamp.nsec=1000 * (time % 1000000);
	msg.header.frame_id= instance->nameSpace.substr(1,instance->nameSpace.length()-1) + "_camera_rgb_optical_link";
	msg.height=instance->height;
	msg.width=instance->width;
	msg.encoding="bgr8";
	msg.step=instance->width*3;
	msg.data.resize(msg.height*msg.step);
	memcpy((char*)(&msg.data[0]),&framebuffer[0],msg.height*msg.step);
	instance->frame_pub.publish(msg);

        //if (frames%instance->framerate==0) {
		rosinfoPrint( static_cast<std::ostringstream*>( &(std::ostringstream() <<  "Frame: " << frames << "/" << pts << ", Average rate: " << (double)(1000000.*pts)/diff.total_microseconds() << ", (" << (double)(1000000.*frames)/diff.total_microseconds() << "), Skipped frames: " << (pts-frames) << std::endl ))->str().c_str());
	//} 
	
	frames++;
        instance->timesfile >> pts >> time;
    }
    //ros::waitForShutdown();

    // join the other threads
    return 0;
}

boost::posix_time::ptime *replay::getStart(void)
{
    return &instance->start;
}

boost::posix_time::ptime *replay::getCurrent(void)
{
    return &instance->current;
}

void replay::rosinfoPrint(const char *bla)
{
    instance->ROSinfo_Mutex.lock();
    ROS_INFO("%s", bla);
    instance->ROSinfo_Mutex.unlock();
}
}
