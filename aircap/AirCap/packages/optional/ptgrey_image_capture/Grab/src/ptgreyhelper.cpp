/*
 * TODO: Add apropriate License and copyright header
 */

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <string>
#include <iostream>
#include "ptgreyhelper.h"
#include "frameptr.h"

namespace videograb {

// Namespace for using Spinnaker objects.
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

class ExposureEventHandler : public DeviceEvent
{
public:
    ExposureEventHandler() {};
    ~ExposureEventHandler() {};
    CameraPtr camera;
    boost::posix_time::ptime grabtime;
    //boost::posix_time::ptime p1t;
    //boost::posix_time::ptime p2t;
    //boost::posix_time::ptime p3t;
    int64_t volatile latestFrame = -1;
    //int64_t volatile p1 = -1;
    //int64_t volatile p2 = -1;
    //int64_t volatile p3 = -1;
    int64_t frameoffset = 0;
    void OnDeviceEvent( gcstring eventName)
    {
	    if (eventName=="EventExposureEnd") {
		    grabtime = boost::posix_time::microsec_clock::universal_time();
		    if (latestFrame==-1) {
			    frameoffset=camera->EventExposureEndFrameID.GetValue();
		    }
		    latestFrame=camera->EventExposureEndFrameID.GetValue()-frameoffset;
		    //p3t=p2t;
		    //p3=p2;
		    //p2t=p1t;
		    //p2=p1;
		    //p1t=grabtime;
		    //p1=latestFrame;
	    }
    }
};



class ptgreyhelper_priv {
public:
    int width;
    int height;
    int framerate;
    CameraPtr camera;
    SystemPtr system;
    //CPylonImage *image;
    //CImageFormatConverter *fc;
    ExposureEventHandler *exposureEventHandler;
};

ptgreyhelper::ptgreyhelper(int framerate)
{
    instance = new ptgreyhelper_priv();
    instance->framerate=framerate;

	instance->system = System::GetInstance();
	CameraList camList = instance->system->GetCameras();
	unsigned int numCameras = camList.GetSize();
	if (numCameras == 0)
	{
		std::cerr << "Camera not found!" << std::endl;
		instance->system->ReleaseInstance();
		instance->system=NULL;
		exit(1);
	}

    instance->camera = camList.GetByIndex(0);
    camList.Clear();
    instance->camera->Init();
    try{
    instance->camera->EndAcquisition();
    } catch (std::exception e) {}

    INodeMap& nodemap = instance->camera->GetNodeMap();
    CStringPtr ptrStringSerial = nodemap.GetNode("DeviceSerialNumber");
    std::cout << ptrStringSerial->GetValue();

    CIntegerPtr offsetX( nodemap.GetNode( "OffsetX"));
    CIntegerPtr offsetY( nodemap.GetNode( "OffsetY"));
    CIntegerPtr width( nodemap.GetNode( "Width"));
    CIntegerPtr height( nodemap.GetNode( "Height"));
    CBooleanPtr ISP( nodemap.GetNode( "IspEnable"));
    CEnumerationPtr ptrPixelFormat( nodemap.GetNode("PixelFormat"));
    try{
	    ISP->SetValue(false);
    } catch (std::exception e) {
            std::cerr << "Camera not ready! Replug camera." << std::endl;
	    instance->camera->DeviceReset.Execute(true);
	    instance->camera=NULL;
	    instance->system->ReleaseInstance();
	    instance->system=NULL;
	    exit(-1);
    }
    offsetX->SetValue(offsetX->GetMin());
    offsetY->SetValue(offsetY->GetMin());
    width->SetValue(width->GetMax());
    height->SetValue(height->GetMax());
    ptrPixelFormat->SetIntValue(ptrPixelFormat->GetEntryByName("BayerRG8")->GetValue());
    instance->width=width->GetValue();
    instance->height=height->GetValue();
std::cout << " setting width x height to " << instance->width << "x" << instance->height << std::endl;
    CBooleanPtr AcquisitionFrameRateEnable( nodemap.GetNode( "AcquisitionFrameRateEnable"));
    CFloatPtr AcquisitionFrameRate( nodemap.GetNode( "AcquisitionFrameRate"));
    AcquisitionFrameRateEnable->SetValue(1);
    try{
        AcquisitionFrameRate->SetValue(framerate);
    } catch (std::exception e) {
            std::cerr << "USB bus too slow, replug camera!" << std::endl;
	    instance->camera->DeviceReset.Execute(true);
	    instance->camera=NULL;
	    instance->system->ReleaseInstance();
	    instance->system=NULL;
	    exit(-1);
    }
    CEnumerationPtr ptrAcquisitionMode = nodemap.GetNode("AcquisitionMode");
    ptrAcquisitionMode->SetIntValue(ptrAcquisitionMode->GetEntryByName("Continuous")->GetValue());

    //instance->image = new CPylonImage();
    //instance->fc = new CImageFormatConverter();
    //instance->fc->OutputPixelFormat = PixelType_BGR8packed;


    CEnumerationPtr ptrEventSelector = nodemap.GetNode("EventSelector");
    NodeList_t entries;
    ptrEventSelector->GetEntries(entries);
	for (int i = 0; i < entries.size(); i++)
	{
		// Select entry on selector node
		CEnumEntryPtr ptrEnumEntry = entries.at(i);
		if (!IsAvailable(ptrEnumEntry) || !IsReadable(ptrEnumEntry))
		{
			// Skip if node fails
			continue;
		}
		
		ptrEventSelector->SetIntValue(ptrEnumEntry->GetValue());

		// Retrieve event notification node (an enumeration node)
		CEnumerationPtr ptrEventNotification = nodemap.GetNode("EventNotification");
		if (!IsAvailable(ptrEventNotification) || !IsWritable(ptrEventNotification))
		{
			// Skip if node fails
			continue;
		}
		
		// Retrieve entry node to enable device event
		CEnumEntryPtr ptrEventNotificationOn = ptrEventNotification->GetEntryByName("On");
		if (!IsAvailable(ptrEventNotification) || !IsReadable(ptrEventNotification))
		{
			// Skip if node fails
			continue;
		}
		
		ptrEventNotification->SetIntValue(ptrEventNotificationOn->GetValue());

		std::cout << "\t" << ptrEnumEntry->GetDisplayName() << ": enabled..." << std::endl;
	}
    instance->exposureEventHandler =  new ExposureEventHandler();
    instance->exposureEventHandler->camera = instance->camera;
    instance->camera->RegisterEvent( *instance->exposureEventHandler );

    // set exposure
    CEnumerationPtr ptrExposureAuto = nodemap.GetNode("ExposureAuto");
    if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
    {
	    std::cerr << "Unable to disable automatic exposure (node retrieval). Aborting..." << std::endl;
    } else {

	    CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
	    if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
	    {
		    std::cerr << "Unable to disable automatic exposure (enum entry retrieval). Aborting..." << std::endl;
	    } else {
		    ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
	    }

	    CFloatPtr ptrExposureTime = nodemap.GetNode("ExposureTime");
	    if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
	    {
		    std::cerr << "Unable to set exposure time. Aborting..." << std::endl;
	    } else {
		    std::cerr << "Forcing exposure time to 250 us" << std::endl;
		    ptrExposureTime->SetValue(250);
	    }

    }

    try{
        instance->camera->BeginAcquisition();
    } catch (std::exception e) {
            std::cerr << "Insufficient USB memory, increase /sys/module/usbcore/parameters/usbfs_memory_mb to at least 1000" << std::endl;
	    instance->camera=NULL;
	    instance->system->ReleaseInstance();
	    instance->system=NULL;
	    exit(-1);
    }
    
}

ptgreyhelper::~ptgreyhelper()
{
    
    try{
    instance->camera->EndAcquisition();
    } catch (std::exception e) {}
    instance->camera=NULL;
    instance->system->ReleaseInstance();
    instance->system=NULL;
    delete instance;
}

int ptgreyhelper::getWidth()
{
    return instance->width;
}

int ptgreyhelper::getHeight()
{
    return instance->height;
}

frameptr ptgreyhelper::getFrame(uint8_t* buffer, size_t maxsize,int64_t minID)
{
	frameptr result;
	result.buffer = NULL;
	if (!instance->camera->IsStreaming()) {
		return result;
	}
        
	result.timestamp = instance->exposureEventHandler->grabtime;
	result.number=instance->exposureEventHandler->latestFrame;
	ImagePtr pResultImage = instance->camera->GetNextImage();
	int64_t frameID = pResultImage->GetFrameID();
	if (frameID>result.number) {
		result.timestamp = instance->exposureEventHandler->grabtime;
		result.number=instance->exposureEventHandler->latestFrame;
	}
	while (frameID<result.number or frameID<=minID) {
		//std::cerr << "Frame outdated!!! " << result.number << " vs " <<  pResultImage->GetFrameID() << std::endl;
		pResultImage->Release();
		pResultImage = instance->camera->GetNextImage();	
		frameID = pResultImage->GetFrameID();
		if (frameID>result.number) {
			result.timestamp = instance->exposureEventHandler->grabtime;
			result.number=instance->exposureEventHandler->latestFrame;
		}
	}
	if (frameID>result.number) {
		// race condition - technically we should never have the data before the event handler got triggered
		// but the event handler timing sometimes isn't the greatest with spinnaker
		result.timestamp = boost::posix_time::microsec_clock::universal_time();
		result.number=frameID;
		std::cerr << "Event Handler Lagging!" << std::endl;
	}
	if (pResultImage->IsIncomplete()) {
		pResultImage->Release();
		return result;
	}
	/*
	result.timestamp = instance->exposureEventHandler->grabtime;
	result.number=instance->exposureEventHandler->latestFrame;
	if (result.number>frameID) {
		result.timestamp = instance->exposureEventHandler->p1t;
		result.number=instance->exposureEventHandler->p1;
		if (result.number>frameID) {
			result.timestamp = instance->exposureEventHandler->p2t;
			result.number=instance->exposureEventHandler->p2;
			if (result.number>frameID) {
				result.timestamp = instance->exposureEventHandler->p2t;
				result.number=instance->exposureEventHandler->p2;
				if (result.number>frameID) {
					std::cerr << "Frame outdated!!! " << instance->exposureEventHandler->latestFrame << " vs " <<  frameID << std::endl;
					pResultImage->Release();
					return result;
				}
			}
		}
	}
	*/
	ImagePtr convertedImage = pResultImage->Convert(PixelFormat_BGR8,NEAREST_NEIGHBOR);
      	uint8_t * origbuffer=(uint8_t*)convertedImage->GetData();
	size_t xwidth= convertedImage->GetWidth();
	size_t xstride=convertedImage->GetStride();
	size_t dstep = 3*instance->width;
	size_t count = convertedImage->GetHeight();
	if (count!=instance->height || count*dstep!=maxsize) {
		std::cerr << "Frame size invalid!" << std::endl;
		pResultImage->Release();
		return result;
	}
	//if (xstride!=dstep) {
		//std::cerr << "need to copy" << std::endl;
		for (int t=0;t<instance->height;t++) {
			std::memcpy( &buffer[t*dstep],&origbuffer[t*xstride],dstep);
		}
		result.buffer=buffer;
	//} else {
	//	result.buffer=origbuffer;
	//}
	//convertedImage->Release();
	pResultImage->Release();
	return result;
}



}
