#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <iostream>

class ImagePublisher
{
public:
    ImagePublisher()
    {
        ros::param::get("~video",pub_video);
        ros::param::get("~video_location",video_loc);

        ros::param::get("~fps",fps);
        ros::param::get("~imageHeight",imageHeight);
        ros::param::get("~imageWidth",imageWidth);
        ros::param::get("~showImage",showImage);

        if(pub_video)
            capture = cv::VideoCapture(video_loc);
        else
            capture = cv::VideoCapture(0);

        pub = n.advertise<sensor_msgs::Image>("image",1);
    }
    ~ImagePublisher(){}

    bool init(){return capture.isOpened();}

    void sendImage()
    {
        ros::Rate loop_rate(fps);

        while(ros::ok())
        {
            capture>>image;
            if(!image.empty())
            {
                resize(image,image,cv::Size(imageWidth,imageHeight));
                image.copyTo(img.image);
                img.header.frame_id = "camera_link";
                img.header.stamp = ros::Time::now();
                if(image.channels() == 3)
                    img.encoding = sensor_msgs::image_encodings::BGR8;
                else
                    img.encoding = sensor_msgs::image_encodings::MONO8;
                pub.publish(img.toImageMsg());
                if(showImage)
                {
                    cv::imshow("image",image);
                    cv::waitKey(1);
                }
            }
            loop_rate.sleep();
        }
    }
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    cv::VideoCapture capture;

    cv::Mat image;
    cv_bridge::CvImage img;

    int fps;
    int imageHeight,imageWidth;
    bool showImage;
    bool pub_video;
    std::string video_loc;
};

int main(int argc, char **argv)
{
     ros::init(argc,argv,"image_pub_node");

     ImagePublisher imgPub;

     if(imgPub.init())
         imgPub.sendImage();
     else
         std::cout<<"camera or video open error, please check camera link or video path."<<std::endl;

     return 0;
}
