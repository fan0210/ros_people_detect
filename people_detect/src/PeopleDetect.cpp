#include <PeopleDetect.h>

PeopleDetect& PeopleDetect::detectInit(const cv::Mat &image)
{
    cudaAllocMapped((void**)&bbCPU, (void**)&bbCUDA, maxBoxes * sizeof(float4));
    cudaAllocMapped((void**)&confCPU, (void**)&confCUDA, maxBoxes * classes * sizeof(float));

    imgHeight = image.rows;
    imgWidth = image.cols;

    const size_t imgSize = imgWidth * imgHeight * sizeof(float) * 4;
    cudaAllocMapped((void**)&imgCPU, (void**)&imgCUDA, imgSize);
    return *this;
}

void PeopleDetect::loadImage(const cv::Mat &image,float4 **img_cpu)
{
    float4 *cpuPtr = *img_cpu;
    for( uint32_t i=0; i < imgHeight; i++ )
    {
        for( uint32_t j=0; j < imgWidth; j++ )
        {
            // opencv: bgr
            cpuPtr[i*imgWidth+j] = make_float4(float(image.at<cv::Vec3b>(i,j)[2]),
                    float(image.at<cv::Vec3b>(i,j)[1]),
                    float(image.at<cv::Vec3b>(i,j)[0]),
                    0.0);
        }
    }
}

const std::vector<cv::Rect>& PeopleDetect::detect(const cv::Mat &image)
{
    loadImage(image,(float4**)&imgCPU);

    founds.clear();
    int box_num = maxBoxes;
    net->Detect(imgCUDA, imgWidth, imgHeight, bbCPU, &box_num, confCPU);

    for( int n=0; n < box_num; n++ )
    {
        float* bb = bbCPU + (n * 4);
        cv::Rect r(cv::Point(bb[0],bb[1]),cv::Point(bb[2],bb[3]));
        founds.push_back(r);
    }

    CUDA(cudaThreadSynchronize());
    return founds;
}
