#include <main.h>

namespace enc = sensor_msgs::image_encodings;

void Main::process()
{
    while(ros::ok())
    {
        ros::spinOnce();
        switch(state)
        {
        case DETECT_INIT:
            if(imageReceived)
            {
                det.detectInit(img);
                state = DETECT;
            }
            else
            {
                ros::Duration(1.0).sleep();
                ROS_INFO("Waiting for an image");
            }
            break;
        case DETECT:
        {
            const std::vector<cv::Rect>& founds = det.detect(img);

            if(trackingReady)
            {
                if(founds.size()>0)
                {
                    int dis = 1000000000;
                    std::vector<int> diss;

                    for(auto iter = founds.cbegin();iter!=founds.cend();++iter)
                    {
                        int d = (iter->x+iter->width/2-img.cols/2)*(iter->x+iter->width/2-img.cols/2)+(iter->y+iter->height/2-img.rows/2)*(iter->y+iter->height/2-img.rows/2);
                        dis = dis<d?dis:d;
                        diss.push_back(d);
                    }
                    for(size_t i = 0;i<diss.size();++i)
                    {
                        if(dis == diss[i])
                        {
                            msgs::Target msg;
                            msg.bb.x = founds[i].x;
                            msg.bb.y = founds[i].y;
                            msg.bb.width = founds[i].width;
                            msg.bb.height = founds[i].height;

                            cv_bridge::CvImage initImage;
                            img.copyTo(initImage.image);
                            initImage.header.frame_id = "firstImage";
                            initImage.header.stamp = ros::Time::now();
                            initImage.encoding = sensor_msgs::image_encodings::BGR8;
                            initImage.toImageMsg(msg.img);

                            pub1.publish(msg);

                            trackingReady = false;
                            break;
                        }
                    }
                }
            }

            msgs::DetectedTargets targets;
            for(auto iter = founds.cbegin();iter!=founds.cend();++iter)
            {
                msgs::BoundingBox box;
                box.x = iter->x;
                box.y = iter->y;
                box.width = iter->width;
                box.height = iter->height;
                targets.BoundingBoxs.push_back(box);
                //rectangle(img,*iter,cv::Scalar(0,255,0),2,8);
            }

            pub2.publish(targets);
        }
        break;
        case STOPPED:
            if(detectStopped)
            {
                ROS_INFO("Detect stopped...");
                detectStopped = false;
            }
            break;
        }
    }
}

void Main::imageReceivedCB(const sensor_msgs::ImageConstPtr &msg)
{
    img_buffer_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    img_buffer_ptr->image.copyTo(img);
    if(img.channels()==1)
        cv::cvtColor(img,img,CV_GRAY2BGR);

    imageReceived = true;
}

bool Main::cmdReceivedCB(std_srvs::Empty::Request &,std_srvs::Empty::Response &)
{
    stopDetecting();

    return true;
}

bool Main::trackingCmdReceivedCB(std_srvs::Empty::Request &,std_srvs::Empty::Response &)
{
    trackingReady = true;

    return true;
}

void Main::stopDetecting()
{
    if(state == DETECT_INIT)
        return;
    if(state == STOPPED)
      state = DETECT;
    else
    {
        detectStopped = true;
        state = STOPPED;
    }
}
