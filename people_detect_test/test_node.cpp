#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include <msgs/DetectedTargets.h>
#include <msgs/BoundingBox.h>
class Main
{
public:
    Main()
    {
        img_sub = n.subscribe("image", 1, &Main::imageRecieveCB, this);
        tags_sub = n.subscribe("targets", 1, &Main::targetsRecieveCB, this);
        tracker_sub = n.subscribe("tracker_target", 1, &Main::trackRecieveCB, this);
    }
    ~Main(){}

    ros::NodeHandle n;

    ros::Subscriber img_sub;
    ros::Subscriber tags_sub;
    ros::Subscriber tracker_sub;

    cv::Mat image;
    std::vector<cv::Rect>targets;
    cv::Rect track_target;

    bool track_targetRecieved = false;
    void display()
    {
        ros::spinOnce();
        if(!image.empty())
        {
            for(auto it = targets.cbegin();it!=targets.cend();++it)
                rectangle(image,*it,cv::Scalar(0,255,0),2);
            if(track_targetRecieved)
                rectangle(image,track_target,cv::Scalar(0,0,255),2);
            cv::imshow("people_detect_test",image);
            cv::waitKey(1);
        }        
    }

private:
    void imageRecieveCB(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr img_buffer_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_buffer_ptr->image.copyTo(image);
    }

    void targetsRecieveCB(const msgs::DetectedTargetsConstPtr &msg)
    {
        targets.clear();
        for(auto it = msg->BoundingBoxs.cbegin();it!=msg->BoundingBoxs.cend();++it)
        {
            cv::Rect r(cv::Point(it->x,it->y),cv::Point(it->x+it->width,it->y+it->height));
            targets.push_back(r);
        }
    }
    void trackRecieveCB(const msgs::BoundingBox &msg)
    {
        ROS_ASSERT(msg.width>0);
        ROS_ASSERT(msg.height>0);

        track_target.x = msg.x;
        track_target.y = msg.y;
        track_target.width = msg.width;
        track_target.height = msg.height;
        track_targetRecieved = true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "people_detect_test");

    Main* main_node = new Main();

    while(ros::ok())
        main_node->display();

    delete main_node;

    return 0;
}
