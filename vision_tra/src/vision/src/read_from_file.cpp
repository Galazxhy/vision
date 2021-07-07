#include "vision/detector.h"

#define debug

int main(int argc,char **argv){
    ros::init(argc,argv,"read_from_file");
    ros::NodeHandle nh;
    
    cv::Mat pic = cv::imread("/home/galazxhy/save/41.jpg");

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image = it.advertise("read_from_file", 1);

    cv_bridge::CvImagePtr frame = boost::make_shared<cv_bridge::CvImage>();
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    while(ros::ok()){

        if(pic.empty()){
            ROS_ERROR_STREAM("failed to capture frame!");
        }

        frame->image = pic;
        frame->header.stamp = ros::Time::now();
        pub_image.publish(frame->toImageMsg());
        #ifdef debug
            cv::imshow("file", frame->image);    
        #endif

        cv::waitKey(1);
        ros::spinOnce();
    }
}