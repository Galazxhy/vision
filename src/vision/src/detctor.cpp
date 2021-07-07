#include "vision/detector.h"

#define debug

class sub_pub:
{
    private:
        ros::NodeHandle nh;
        ros::Subscriber  sub;
        ros::Publisher pub;
    public:
        sub_pub()
        {
            sub = nh.subscribe("camera", 1, &sub_pub::detector, this);

            pub = nh.advertise<bool>("if_go",1);
        }
        void detector();
}

void detector();

int main(int argc, char **argv){
    ros::init(argc, argv, "sub_pub");
    
    sub_pub sub_puber;
    
    ros::spin();

    return 0;
}