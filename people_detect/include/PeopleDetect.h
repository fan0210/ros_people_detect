#ifndef PEOPLE_DETECT_H_
#define PEOPLE_DETECT_H_

#include <opencv2/opencv.hpp>
#include <detectNet.h>
#include <cudaMappedMemory.h>
#include <cudaNormalize.h>
#include <stdio.h>
#include <unistd.h>

class PeopleDetect
{
public:
    PeopleDetect():net(detectNet::Create(0,nullptr)),maxBoxes(net->GetMaxBoundingBoxes()),classes(net->GetNumClasses()){}
    PeopleDetect(const PeopleDetect&) = delete;
    PeopleDetect& operator=(const PeopleDetect&) = delete;

    ~PeopleDetect(){ CUDA(cudaFreeHost(imgCPU));delete net;}

    PeopleDetect& detectInit(const cv::Mat &);
    const std::vector<cv::Rect>& detect(const cv::Mat &);
private:
    std::vector<cv::Rect>founds;

    detectNet *net = nullptr;

    const uint32_t maxBoxes = 0;
    const uint32_t classes = 0;
    float *bbCPU    = nullptr;
    float *bbCUDA   = nullptr;
    float *confCPU  = nullptr;
    float *confCUDA = nullptr;
    float *imgCPU   = nullptr;
    float *imgCUDA  = nullptr;

    int imgWidth = 0;
    int imgHeight= 0;

    void loadImage(const cv::Mat &,float4 **);
};

#endif
