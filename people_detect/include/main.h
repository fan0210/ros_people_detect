#ifndef MAIN_H_
#define MAIN_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <msgs/Target.h>
#include <msgs/BoundingBox.h>
#include <msgs/DetectedTargets.h>
#include <std_srvs/Empty.h>

#include <PeopleDetect.h>

class Main
{
public:
    Main():state(DETECT_INIT)
    {
        pub1 = n.advertise<msgs::Target>("target", 1000, true);
        pub2 = n.advertise<msgs::DetectedTargets>("detected_targets", 3, true);

        sub1 = n.subscribe("image",1,&Main::imageReceivedCB,this);
        server1 = n.advertiseService("cmd_stop_or_start", &Main::cmdReceivedCB, this);
        server2 = n.advertiseService("cmd_start_tracking", &Main::trackingCmdReceivedCB,this);
    }
    ~Main() = default;

    void process();

private:
    enum
    {
        DETECT_INIT,
        DETECT,
        STOPPED
    } state;

    PeopleDetect det;

    ros::NodeHandle n;
    ros::Publisher pub1;
    ros::Publisher pub2;
    ros::Subscriber sub1;
    ros::ServiceServer server1;
    ros::ServiceServer server2;

    bool trackingReady = false;

    bool imageReceived = false;
    bool detectStopped = false;

    cv::Mat img;
    cv_bridge::CvImagePtr img_buffer_ptr;

    void imageReceivedCB(const sensor_msgs::ImageConstPtr &);

    bool cmdReceivedCB(std_srvs::Empty::Request &,std_srvs::Empty::Response &);
    bool trackingCmdReceivedCB(std_srvs::Empty::Request &,std_srvs::Empty::Response &);

    void stopDetecting();
};

#endif
