/*
 * TODO: Add apropriate License and copyright header
 */

#include <pylon/PylonIncludes.h>

#include <string>
#include <iostream>
#include "baslerhelper.h"
#include "frameptr.h"

namespace videograb {

// Namespace for using GenApi objects.
using namespace GenApi;

// Namespace for using pylon objects.
using namespace Pylon;




class ExposureEventHandler : public CImageEventHandler
{
public:
    boost::posix_time::ptime grabtime;
    uint64_t volatile latestFrame;
    virtual void OnImageGrabbed( CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
    {
            grabtime = boost::posix_time::microsec_clock::universal_time();
            latestFrame = ptrGrabResult->GetBlockID();
    }
    
};



class baslerhelper_priv {
public:
    int width;
    int height;
    int framerate;
    CInstantCamera *camera;
    CPylonImage *image;
    CImageFormatConverter *fc;
    ExposureEventHandler *exposureEventHandler;
};

baslerhelper::baslerhelper(int framerate)
{
    instance = new baslerhelper_priv();
    instance->framerate=framerate;


    PylonInitialize();

    instance->camera = new CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
    std::cout << "Using device " << instance->camera->GetDeviceInfo().GetModelName() << std::endl;
    instance->camera->Open();

    INodeMap& nodemap = instance->camera->GetNodeMap();

    CIntegerPtr offsetX( nodemap.GetNode( "OffsetX"));
    CIntegerPtr offsetY( nodemap.GetNode( "OffsetY"));
    CIntegerPtr width( nodemap.GetNode( "Width"));
    CIntegerPtr height( nodemap.GetNode( "Height"));
    
    offsetX->SetValue(offsetX->GetMin());
    offsetY->SetValue(offsetY->GetMin());
    width->SetValue(width->GetMax());
    height->SetValue(height->GetMax());
    instance->width=width->GetValue();
    instance->height=height->GetValue();
std::cout << " setting woidth x height to " << instance->width << "x" << instance->height << std::endl;
    CBooleanPtr AcquisitionFrameRateEnable( nodemap.GetNode( "AcquisitionFrameRateEnable"));
    CFloatPtr AcquisitionFrameRate( nodemap.GetNode( "AcquisitionFrameRate"));
    AcquisitionFrameRateEnable->SetValue(1);
    AcquisitionFrameRate->SetValue(framerate);
    instance->image = new CPylonImage();
    instance->fc = new CImageFormatConverter();
    instance->fc->OutputPixelFormat = PixelType_BGR8packed;


    instance->exposureEventHandler =  new ExposureEventHandler();
    instance->camera->RegisterImageEventHandler( instance->exposureEventHandler, RegistrationMode_Append, Cleanup_Delete);

    instance->camera->StartGrabbing( GrabStrategy_LatestImageOnly );
    
}

baslerhelper::~baslerhelper()
{
    delete instance;
}

int baslerhelper::getWidth()
{
    return instance->width;
}

int baslerhelper::getHeight()
{
    return instance->height;
}

frameptr baslerhelper::getFrame()
{
	frameptr result;
	result.buffer = NULL;
	if (!instance->camera->IsGrabbing()) {
		return result;
	}
        CGrabResultPtr ptrGrabResult;
        instance->camera->RetrieveResult( 100, ptrGrabResult, TimeoutHandling_ThrowException);
	if (!ptrGrabResult->GrabSucceeded()) {
		return result;
	}
	result.timestamp = instance->exposureEventHandler->grabtime;
	result.number=instance->exposureEventHandler->latestFrame;
	if (result.number!=ptrGrabResult->GetBlockID()) {
		std::cerr << "Frame outdated!!!" << std::endl;
		return result;
	}   
	instance->fc->Convert(*instance->image, ptrGrabResult);
	result.buffer=(uint8_t*)instance->image->GetBuffer();
	return result;
}



}
