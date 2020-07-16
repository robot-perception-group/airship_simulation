/*
 * TODO: Add apropriate License and copyright header
 */

#include "videograb.h"
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <boost/filesystem/fstream.hpp>
#include <boost/thread.hpp>
#include <unistd.h>
extern "C" {
#include "libavhelper.h"
}

namespace videograb {
videograb *globalVideoGrab;

/*
        void sigHandler(const boost::system::error_code& error, int signal_number) {
                // shutdown read and write threads too
                exit(-1);
                ros::shutdown();
        }
 */


ssize_t readall(int fd, void *buf, size_t count){
	ssize_t rest;
	while (count>0) {
		rest=read(fd,buf,count);
		if (rest<0) {
			std::cerr << "FUCK!" << rest << std::endl;
			perror("grrr");
			return rest;
		}
		count=count-rest;
		buf=(void*)(((ssize_t)buf)+rest);
	}
	return count;
}

class transfer_frame_priv {
public:
    uint64_t number;
    size_t size;
};

class videograb_priv {
public:
    int argc;
    char * *argv;
    ros::NodeHandle *nodehandle;
    ros::Time start;
    boost::mutex serial_Mutex;
    boost::mutex ROSinfo_Mutex;
    int width;
    int height;
    int framerate;
    std::string videofilename;
    std::string nameSpace;
    int threads;
    ros::Subscriber frame_sub;
    boost::filesystem::ofstream timesfile;
    boost::thread *thread;
    int pipe[2];
    volatile int recording = 0;
    int64_t frames, current,skipped;
    
    // ...
    void run()
    {
	uint8_t* buffer;
        transfer_frame_priv transfer;
        while (thread) {
            read(pipe[0],&transfer,sizeof(transfer));
		if (!buffer) {
			buffer = new uint8_t[3*width*height];
			if (!buffer) {
				std::cerr << "Out of memory!!!" <<std::endl;
				exit(-1);
			}
		}
	    if (transfer.size!=(size_t)(3*width*height)) {
                std::cerr << " buffer size mismatch " << (3*width*height) << "(" << 3 << " " << width<< " " << height <<") != " << transfer.size << std::endl;
		exit(-1);
	    } else {
                readall(pipe[0],&buffer[0],transfer.size);
	    	libavhelper_save_frame(&buffer[0],transfer.number);
	    }
        }
    }
    
    void recordCallback(const std_msgs::UInt8::ConstPtr & msg)
    {
	recording=msg->data;
    }
    
};

videograb::videograb(int argc, char * *argv)
{
    globalVideoGrab = this;
    instance = new videograb_priv;
    if (argc<5) {
        std::cerr << "Usage: " << std::string(argv[0]) << " <namespace> <video folder> <framerate in fps> <threads used for encoding> [ROS arguments*]\n";
	exit(1);
    }
    instance->framerate=std::stoi(std::string(argv[3]));
    instance->threads=std::stoi(std::string(argv[4]));
    instance->argc  = argc-4;
    instance->argv  = new char *[instance->argc];
    instance->argv[0] = argv[0];
    int t;
    for (t=5;t<argc;t++) {
       instance->argv[t-4]=argv[t];
    }
    instance->videofilename = static_cast<std::ostringstream*>( &(std::ostringstream() << argv[2] << "/capture.avi"))->str();
    instance->nameSpace = argv[1];
    ros::init(instance->argc,instance->argv,"videograb");
    instance->nodehandle = new ros::NodeHandle;
    instance->frame_sub = instance->nodehandle->subscribe(instance->nameSpace + "/video",10, &videograb::videoCallback, this);
    instance->timesfile.open(instance->videofilename + ".times");
    pipe(instance->pipe);
    instance->frames=0;
    instance->current=-1;
    instance->skipped=0;
    instance->thread = new boost::thread(boost::bind(&videograb_priv::run, instance));
}


videograb::~videograb()
{
    if (instance->thread) {
        delete instance->thread;
    }
    if (instance->nodehandle) {
        delete instance->nodehandle;
    }
    delete instance;
    libavhelper_close_write();
}

int videograb::run(void)
{
    ros::init(instance->argc, instance->argv, "videograb");

    instance->nodehandle = new ros::NodeHandle();
    // boost::asio::signal_set signals(instance->io_service, SIGINT, SIGTERM);


    // do everything here
    ros::AsyncSpinner spinner(4);
    spinner.start();
  

    ros::Subscriber subscriber1 = instance->nodehandle->subscribe("/global/record",10, &videograb_priv::recordCallback, instance);

    ros::waitForShutdown();
    while (ros::ok()) {
    }
    //ros::waitForShutdown();

    // join the other threads
    return 0;
}

void videograb::videoCallback(const sensor_msgs::ImageConstPtr & msgp)
{
	// first frame
	bool first=false;
        if (instance->start.sec==0) {
		instance->start=msgp->header.stamp;
		instance->width=msgp->width;
		instance->height=msgp->height;
		libavhelper_init_write(instance->videofilename.c_str(),"avi","ffvhuff",instance->width,instance->height,instance->framerate,AV_PIX_FMT_RGB24,instance->threads);
		first=true;
	}
        transfer_frame_priv transfer;
	int64_t number = 1+(msgp->header.stamp-instance->start).toSec()/(1.0/instance->framerate);
	// send image to ros
	if (instance->recording || first) {
            if (number>instance->current) {
		instance->timesfile << number << '\t' << (int64_t)((msgp->header.stamp).toNSec()/1000) << std::endl;

		// encode video
	        transfer.number = number;
		transfer.size = (size_t)(3*msgp->width*msgp->height);
		write(instance->pipe[1],&transfer,sizeof(transfer));
		write(instance->pipe[1],&msgp->data[0],transfer.size);
		//ssize_t ptr=0;
		//while (ptr<msgp->step*msgp->height) {
		//	write(instance->pipe[1],&(msgp->data[ptr]),msgp->width);
		//	ptr+=msgp->step;
		//}
		instance->current=number;
	    } else {
		rosinfoPrint( static_cast<std::ostringstream*>( &(std::ostringstream() <<  "Dropping Frame! " << number << " <= " << instance->current << std::endl ))->str().c_str());
		instance->skipped++;
	    }
	}
	
        if (instance->frames%instance->framerate==0) {
		rosinfoPrint( static_cast<std::ostringstream*>( &(std::ostringstream() <<  "Frame: " << instance->frames << "/" << number << ", Average rate: " << (double)(1000000.*instance->frames)/(uint64_t)((msgp->header.stamp-instance->start).toNSec()/1000) << ", Skipped frames: " << (number-instance->frames) << ", Dropped frames: " << instance->skipped << std::endl ))->str().c_str());
	}
	
	instance->frames++;
}

void videograb::rosinfoPrint(const char *bla)
{
    instance->ROSinfo_Mutex.lock();
    ROS_INFO("%s", bla);
    instance->ROSinfo_Mutex.unlock();
}
}
