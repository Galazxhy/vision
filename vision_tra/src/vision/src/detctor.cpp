#include "vision/detector.h"

#define debug
#define bi_threshold 190

class sub_pub
{
    private:
        ros::NodeHandle nh;
        image_transport::Subscriber  sub;
        ros::Publisher pub;
    public:
        sub_pub(){
            // sub = nh.subscribe("camera", 1, &sub_pub::detector, this);
            image_transport::ImageTransport it(nh);
            sub = it.subscribe("camera", 1, &sub_pub::detector, this);
            pub = nh.advertise<std_msgs::Bool>("if_go",1);
        }
        void detector(const sensor_msgs::ImageConstPtr& src);
};

void sub_pub::detector(const sensor_msgs::ImageConstPtr& src){
    std::vector<cv::Mat> BGR;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(src,sensor_msgs::image_encodings::TYPE_8UC3);
    cv::Mat srcImg = cv_ptr->image;
    cv::Mat show_light = srcImg;

    // cv::namedWindow("src",cv::WINDOW_AUTOSIZE);
    // cv::imshow("src",srcImg);

    if(!srcImg.empty()){
    //白平衡+通道相减
    cv::Mat channel_dst;
    cv::Mat Balanced;
    cv::split(srcImg, BGR);
    
    double R, G, B;
    B = cv::mean(BGR[0])[0];
    G = cv::mean(BGR[1])[0];
    R = cv::mean(BGR[2])[0];

    double KR, KG, KB;
    KB = (R + G + B) / (3 * B);
    KG = (R + G + B) / (3 * G);
    KR = (R + G + B) / (3 * R);

    BGR[0] = BGR[0] * KB;
    BGR[1] = BGR[1] * KG;
    BGR[2] = BGR[2] * KR;

    channel_dst = 2 * BGR[1] - BGR[0] - BGR[2];

    merge(BGR, Balanced);
    #ifdef debug
    cv::imshow("balanced", Balanced);
    cv::imshow("channel_dst", channel_dst);
    #endif

    //二值化
    cv::Mat bi_dst;
    cv::threshold(channel_dst, bi_dst, bi_threshold, 255, CV_THRESH_BINARY);
    //膨胀
    cv::dilate(bi_dst, bi_dst, cv::Mat());
    #ifdef debug
    cv::imshow("bi_dst", bi_dst);
    #endif

    //获取轮廓
    std::vector<cv::RotatedRect> suspect_light;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bi_dst, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for(int i = 0; i < contours.size(); ++i){
        cv::RotatedRect find = cv::minAreaRect(contours[i]);
        cv::Rect bounding_rect  = find.boundingRect();
        if(bounding_rect.area() <= 220 || abs(find.angle) > 45)
            break; 
        suspect_light.emplace_back(find);
    }
    for(auto light : suspect_light){
        cv::Point2f vertices[4];
        light.points(vertices);
        for(int i = 0; i < 4; i++){
            cv::line(show_light, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0,255,0),1);
        }
    }
    #ifdef debug
    cv::imshow("show_light", show_light);
    #endif
    cv::waitKey(1);

    if(!suspect_light.empty()){
    std_msgs::Bool res;
    res.data = 1;
    pub.publish(res);
    }
    else{
        std_msgs::Bool res;
    res.data = 0;
    pub.publish(res);
    }
    }
    else {
        ROS_ERROR("Image Empty!");
        std_msgs::Bool res;
        res.data = 0;
        pub.publish(res);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sub_pub");
    
    sub_pub sub_puber;
    
    ros::spin();

    return 0;
}