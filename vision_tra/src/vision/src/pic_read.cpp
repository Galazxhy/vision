#include "vision/detector.h"

#define debug

std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method){
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=(int)" +
    std::to_string(capture_width) + ", height=(int)"+
    std::to_string(capture_height)+",format=(string)NV12, framerate=(fraction)"+
    std::to_string(framerate)+"/1 ! nvvidconv flip-method="+
    std::to_string(flip_method)+" ! video/x-raw, width=(int)"+
    std::to_string(display_width)+", height=(int)"+
    std::to_string(display_height)+", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc,char **argv){
    ros::init(argc, argv, "pic_read");
    ros::NodeHandle nh;
    int capture_width = 640;
    int capture_height = 480;
    int display_width = 640;
    int display_height = 480;
    int framerate = 60;
    int flip_method = 0;
    std::string pipeline = gstreamer_pipeline(capture_width,capture_height,display_width,display_width,framerate,flip_method);

    // cv::VideoCapture capture(pipeline, cv::CAP_GSTREAMER);
    cv::VideoCapture capture("/home/galazxhy/1.mp4");
    if(! capture.isOpened()){
        ROS_ERROR_STREAM(
            "Failed to open camera !"
        );
    }
    cv::Mat frame_parme;
    capture>>frame_parme;

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0,0) = 402.3383;
    cameraMatrix.at<double>(0,1) = 0;
    cameraMatrix.at<double>(0,2) = 327.4766;
    cameraMatrix.at<double>(1,1) = 715.2141;
    cameraMatrix.at<double>(1,2) = 399.4971;

    cv::Mat distCoeffs = cv::Mat::zeros(5,1,CV_64F);
    distCoeffs.at<double>(0,0) = -0.3383;
    distCoeffs.at<double>(1,0) = 0.0993;
    distCoeffs.at<double>(2,0) = 0;
    distCoeffs.at<double>(3,0) = 0;
    distCoeffs.at<double>(4,0) = 0;

    cv::Mat view, rview, map1, map2;
    cv::Size imagesize = frame_parme.size();
    cv::initUndistortRectifyMap(cameraMatrix,distCoeffs,cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imagesize, 1, imagesize, 0),imagesize, CV_16SC2, map1, map2 );

    #ifdef debug
    cv::namedWindow("CSI camera",cv::WINDOW_AUTOSIZE);
    #endif
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image = it.advertise("camera", 1);

    cv_bridge::CvImagePtr frame = boost::make_shared<cv_bridge::CvImage>();
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    while(ros::ok()){
        capture >> frame_parme;
        if(frame_parme.empty()){
            ROS_ERROR_STREAM("failed to capture frame!");
        }
        cv::remap(frame_parme, frame->image, map1, map2, cv::INTER_LINEAR);

        frame->header.stamp = ros::Time::now();
        pub_image.publish(frame->toImageMsg());
        #ifdef debug
            cv::imshow("CSI camera", frame->image);    
        #endif
        cv::waitKey(1);
        ros::spinOnce();
    }

    capture.release();
    return EXIT_SUCCESS;
}
