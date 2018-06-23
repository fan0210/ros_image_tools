#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

class Main
{
public:
    Main()
    {
        ros::param::get("~saveImg",saveImg);
        ros::param::get("~saveVideo",saveVideo);

        ros::param::get("~fps",fps);
        ros::param::get("~rgb_location",rgb_loc);
        ros::param::get("~depth_location",depth_loc);

        ros::param::get("~video_location",video_loc);

        ros::param::get("~showImage",showImage);

        rgb_sub = nh.subscribe("rgb_image_raw", 1, &Main::imageRgbReceiveCB, this);
        depth_sub = nh.subscribe("depth_image_raw", 1, &Main::imageDepthReceiveCB, this);
        server = nh.advertiseService("stop_cmd",&Main::stopCmdReceiveCB,this);
    }

    ~Main() = default;

    void save();

private:
    ros::NodeHandle nh;

    cv::Mat img_rgb;
    cv::Mat img_depth;

    cv::VideoWriter rgb_writer;
    cv::VideoWriter depth_writer;

    bool saveImg = false,saveVideo = false;

    int fps = 10;
    std::string rgb_loc = "/home/nvidia/catkin_ws/src/save_video/image/rgb";
    std::string depth_loc = "/home/nvidia/catkin_ws/src/save_video/image/depth";
    std::string video_loc ="/home/nvidia/catkin_ws/src/save_video/video";

    ros::Subscriber rgb_sub;
    ros::Subscriber depth_sub;
    ros::ServiceServer server;

    bool stop_running = false;

    bool rgb_recieved = false,depth_recieve = false;
    bool rgb_videoInit = false,depth_videoInit = false;

    bool showImage = false;

    void show()
    {
        if(showImage)
        {
            if(!img_rgb.empty())
                imshow("rgb_image",img_rgb);
            if(!img_depth.empty())
                imshow("depth_image",img_depth);
            cv::waitKey(1);
        }
    }

    void videoInitAndSave()
    {
        if(saveVideo)
        {
            if(rgb_videoInit)
                rgb_writer<<img_rgb;
            if(depth_videoInit)
            {
                cv::Mat convertImg(img_depth.size(),CV_8UC1);
                for(auto i = 0;i<img_depth.rows;++i)
                {
                    short *data_depth = img_depth.ptr<short>(i);
                    unsigned char *data_convert = convertImg.ptr<unsigned char>(i);
                    for(auto j = 0;j<img_depth.cols;++j)
                        data_convert[j] = data_depth[j]/256;
                }
                cv::cvtColor(convertImg,convertImg,CV_GRAY2BGR);

                depth_writer<<convertImg;
            }

            if(!rgb_videoInit&&rgb_recieved&&!img_rgb.empty())
            {
                std::string fileName = video_loc+"/rgb.avi";
                rgb_writer = cv::VideoWriter(fileName,CV_FOURCC('F','L','V','1'),fps,cv::Size(img_rgb.cols,img_rgb.rows));
                rgb_videoInit = true;
            }
            if(!depth_videoInit&&depth_recieve&&!img_depth.empty())
            {
                std::string fileName = video_loc+"/depth.avi";
                depth_writer = cv::VideoWriter(fileName,CV_FOURCC('F','L','V','1'),fps,cv::Size(img_depth.cols,img_depth.rows));
                depth_videoInit = true;
            }
        }
    }

    void imgInitAndSave(size_t& rgb_count,size_t& depth_count)
    {
        if(saveImg)
        {
            if(!img_rgb.empty())
            {
                std::string fileName = rgb_loc+"/rgb"+std::to_string(rgb_count)+".png";
                imwrite(fileName,img_rgb);
                ++rgb_count;
            }
            if(!img_depth.empty())
            {
                std::string fileName = depth_loc+"/depth"+std::to_string(depth_count)+".png";
                imwrite(fileName,img_depth);
                ++depth_count;
            }
        }

    }

    void imageRgbReceiveCB(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr img_buffer_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_buffer_ptr->image.copyTo(img_rgb);
        rgb_recieved = true;
    }

    void imageDepthReceiveCB(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr img_buffer_ptr = cv_bridge::toCvCopy(msg);
        img_buffer_ptr->image.copyTo(img_depth);
        depth_recieve = true;
    }

    bool stopCmdReceiveCB(std_srvs::Empty::Request&,std_srvs::Empty::Response&)
    {
        stop_running = true;
        return true;
    }
};

void Main::save()
{
    ros::Rate loop_rate(fps);

    size_t rgb_count = 0,depth_count = 0;
    while(ros::ok()&&!stop_running)
    {
        ros::spinOnce();

        show();

        videoInitAndSave();
        imgInitAndSave(rgb_count,depth_count);

        loop_rate.sleep();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"video_save_node");

    Main *obj = new Main();

    obj->save();
    ROS_INFO("Stop Successfully!");

    delete obj;

    return 0;
}
