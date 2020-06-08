#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <string>

#include "my_service/location.h"

#define DATA_SIZE 11
#define KINECT_HEIGHT 700
#define KINECT_BACK 500
#define WORKSPACE_RADIUS 500
#define RADIUS 230
#define DETECTION_AREA_RADIUS 500

#define BALL 0
#define BEAR 1
#define CAN 2

#define CALIB 1.0

class perceptObject
{
public:
    perceptObject()
    {
        nh.setParam("object", BALL);
        objectSet.push_back({28,44,105,219,90,243});
        objectSet.push_back({13,46,126,245,18,199});
        objectSet.push_back({83,129,0,24,192,248});
        objectSet.push_back({0,15,142,255,135,226});
        

        message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "rgb/image_raw", 1);
        message_filters::Subscriber<sensor_msgs::Image> dph_sub(nh, "depth_to_rgb/image_raw", 1);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
        //typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dph_sub);

        sync.registerCallback(boost::bind(&perceptObject::capture, this, _1, _2));

        ros::spin();
    }

    ~perceptObject() {}

    void capture(const sensor_msgs::ImageConstPtr &rgbmsg, const sensor_msgs::ImageConstPtr &dphmsg)
    {
        nh.getParam("object",objectID);
        if(objectID >= objectSet.size()){
            ROS_ERROR("INVALID_PARAMETER_ERROR");
            return;
        }
        cv_bridge::CvImageConstPtr cv_ptr1;
        cv_bridge::CvImageConstPtr cv_ptr2;
        cv_ptr1 = cv_bridge::toCvShare(rgbmsg, sensor_msgs::image_encodings::BGR8);
        cv_ptr2 = cv_bridge::toCvShare(dphmsg);
        //std::cout<<"rgb / sec : "<<rgbmsg->header.stamp.sec<<" / nsec : "<<rgbmsg->header.stamp.nsec<<"\n";
        //std::cout<<"dph / sec : "<<dphmsg->header.stamp.sec<<" / nsec : "<<dphmsg->header.stamp.nsec<<"\n";
        n1 = rgbmsg->header.stamp.nsec;
        n2 = dphmsg->header.stamp.nsec;

        im_rgb = cv_ptr1->image;
        im_dph = cv_ptr2->image;
        //imshow("rgb", im_rgb);
        //imshow("dph", im_dph);
        //waitKey(1);
        if (findXY())
        {
            findDepth();
            if (depth != -1 && depth != 0 && depth < KINECT_BACK + DETECTION_AREA_RADIUS)
            {
                find3D();
            }
            else
            {
                //ROS_ERROR("DEPTH_ERROR_%f",depth);
            }

            if (Coordinates.size() == DATA_SIZE)
            {
                curveFitting();
                ros::Duration f(3.0);
                ros::Rate loop_rate(10);
                std::cout << "<==========Fitting finish==========>\n";
                t2_s = ros::Time::now().sec;
                t2_ns = ros::Time::now().nsec;
                std::cout<<"t1 : "<<t1_s<<"sec / "<<t1_ns<<"ns\n";
                std::cout<<"t2 : "<<t2_s<<"sec / "<<t2_ns<<"ns\n";
                std::cout<<"t2 - t1 = "<<t2_s - t1_s<<"sec / "<<t2_ns - t1_ns<<"ns\n";
                ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("point_data", 10);
                geometry_msgs::PointStamped pnt;
                pnt.header.frame_id = "rgb_camera_link";
                
                for (int i = 0; i < DATA_SIZE; i++)
                {
                    pnt.point.x = Coordinates[i][0];
                    pnt.point.y = Coordinates[i][1];
                    pnt.point.z = Coordinates[i][2];
                    std::cout << "\n";
                    std::cout << i
                         << pnt.point.x << " / " << pnt.point.y << "\n";
                    point_pub.publish(pnt);
                    loop_rate.sleep();
                }
                
                Coordinates.clear();

                f.sleep();
            }
        }
    }

    void find3D()
    {
        ros::ServiceClient client = nh.serviceClient<my_service::location>("location");
        my_service::location srv;

        srv.request.xi = x;
        srv.request.yi = y;
        srv.request.di = depth * 1000;

        std::cout << "send data\n";
        std::cout << "i : "
             << x
             << " / j : "
             << y
             << " / d : "
             << depth * 1000
             << "\n";

        if (client.call(srv))
        {
            std::cout << "rece data\n";
            std::cout << "x : "
                 << srv.response.xo
                 << " / y : "
                 << srv.response.zo - KINECT_BACK
                 << " / z : "
                 << -srv.response.yo + KINECT_HEIGHT
                 << "\n";
            float cart_x, cart_y, cart_z;
            cart_x = srv.response.xo;
            cart_y = srv.response.zo - KINECT_BACK;
            cart_z = -srv.response.yo + KINECT_HEIGHT;
            if (abs(cart_x) < DETECTION_AREA_RADIUS && abs(cart_y) < DETECTION_AREA_RADIUS + KINECT_BACK)
            {   
                std::cout<<"Csize : "<<Coordinates.size()<<"\n";
                Coordinates.push_back({cart_x, cart_y, cart_z});
                if (Coordinates.size() != 1 && Coordinates.size() != 0)
                {
                    if (cart_x > max_x)
                    {
                        max_x = cart_x;
                    }
                    if (cart_x < min_x)
                    {
                        min_x = cart_x;
                    }
                    if (cart_y > max_y)
                    {
                        max_y = cart_y;
                    }
                    if (cart_y < min_y)
                    {
                        min_y = cart_y;
                    }
                }
                else{
                    t1_s = ros::Time::now().sec;
                    t1_ns = ros::Time::now().nsec;
                }
            }
            else
            {
                ROS_ERROR("OUT_OF_DETECTION_AREA_ERROR");
                std::cout << "rgb / sec : / nsec : " << n1 << "\n";
                std::cout << "dph / sec : / nsec : " << n2 << "\n";
                
                circle(im_rgb, cv::Point(_x, _y), 3, cv::Scalar(0, 255, 0),CV_FILLED);
                circle(im_dph, cv::Point(_x, _y), 3, cv::Scalar(0, 255, 0),CV_FILLED);
                imshow("tracking", im_rgb); // show windows
                imshow("dph", im_dph);
                cv::waitKey(0);
                
            }
        }
        else
        {
            ROS_ERROR("SERVICE_RECEIVE_ERROR");
        }
        x = y = depth = -1;
    }

    bool findXY()
    {
        cv::Mat hsvImg;
        cv::Mat threshImg;
        cv::Mat gaussianImg;
        cv::Mat erodeImg;
        cv::Mat dilateImg;
        

        //cv::Mat mask;
        cv::Mat m = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2), cv::Point(1, 1));

        std::vector<cv::Vec3f> v3fCircles;
        int lowH = objectSet[objectID][0]; // Set Hue
        int highH = objectSet[objectID][1];

        int lowS = objectSet[objectID][2]; // Set Saturation
        int highS = objectSet[objectID][3];

        int lowV = objectSet[objectID][4]; // Set Value
        int highV = objectSet[objectID][5];

        cvtColor(im_rgb, hsvImg, CV_BGR2HSV);
        inRange(hsvImg, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), threshImg);

        GaussianBlur(threshImg, gaussianImg, cv::Size(3, 3), 0); //Blur Effect

        cv::erode(gaussianImg, erodeImg, m, cv::Point(-1, -1), 5);
        cv::dilate(erodeImg, dilateImg, m, cv::Point(-1, -1), 5);
  

        HoughCircles(dilateImg, v3fCircles, CV_HOUGH_GRADIENT, 2, dilateImg.rows / 4, 50, 35, 10, 100);

        for (int i = 0; i < v3fCircles.size(); i++)
        { // for each circle

            //std::cout << "Ball position X = " << v3fCircles[i][0] << ",\tY = " << v3fCircles[i][1] << ",\tRadius = " << v3fCircles[i][2] << "\n"; // radius of circle

            // draw small green circle at center of object detected
            //circle(im_rgb, cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]), 3, cv::Scalar(0, 255, 0), CV_FILLED); // thickness
            _x = (int)v3fCircles[i][0];
            _y = (int)v3fCircles[i][1];

            // draw red circle around object detected
            circle(im_rgb, cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]), (int)v3fCircles[i][2], cv::Scalar(0, 0, 255), 3); // thickness
            
            imshow("tracking", im_rgb); // show windows
            //imshow("dph", im_dph);
            //imshow("threshImg", threshImg);
            //imshow("gaussianImg",gaussianImg);
            //imshow("erodeImg",erodeImg);
            imshow("dilateImg",dilateImg);
            cv::waitKey(1);
            
            if (v3fCircles.size() == 1)
            {
                x = v3fCircles[0][0];
                y = v3fCircles[0][1];
                r = v3fCircles[0][2];
                //rectangle(cv_ptr->image, bbox, Scalar(255, 0, 0), 2, 1);
                //std::cout << "Object Detected\n";
                object_found = true;
                cv::Rect2d bbox(x - r, y - r, 2 * r, 2 * r);
                //tracker->init(cv_ptr->image,bbox);

                //std::cout<<"x : "<<x<<"\n";
                //std::cout<<"y : "<<y<<"\n";
                return true;
            }
            else
            {
                x = -1;
                y = -1;
                r = -1;
                return false;
            }
        }
        
        imshow("tracking", im_rgb); // show windows
        //imshow("dph", im_dph);
        //imshow("threshImg", threshImg);
        //imshow("gaussianImg",gaussianImg);
        //imshow("erodeImg",erodeImg);
        imshow("dilateImg",dilateImg);
        cv::waitKey(1);
        
    }

    void findDepth()
    {
        if (x == -1 && y == -1)
        {
            depth = -1;
            return;
        }

        cv::Mat_<float> im_dph2 = im_dph;
        depth = im_dph2(y, x);
    }

    void inverse(float m[3][3])
    {
        float det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
                    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

        float invdet = 1 / det;

        float minv[3][3] = {}; // inverse of matrix m
        minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
        minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
        minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
        minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
        minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
        minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
        minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
        minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
        minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                m[i][j] = minv[i][j];
            }
        }
    }

    void matrixP(float a[3][3], float b[3], float c[3])
    {
        inverse(a);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                c[i] += a[i][j] * b[j];
            }
        }
    }

    void curveFitting()
    {
        std::cout << "<==========Start Fitting=============>\n";
        float _x, _y, _z;
        float zero_x;
        float zero_y;
        float xsum[3][3] = {};
        float ysum[3][3] = {};
        float xzsum[3] = {};
        float yzsum[3] = {};
        float coeff_x[3] = {};
        float coeff_y[3] = {};
        std::cout << xsum[0][0] << "\n";
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                for (int k = 1; k < DATA_SIZE; k++)
                {
                    xsum[i][j] += (float)pow(Coordinates[k][0], i + j);
                    ysum[i][j] += (float)pow(Coordinates[k][1], i + j);
                }
            }
        }
        for (int i = 0; i < 3; i++)
        {
            for (int k = 1; k < DATA_SIZE; k++)
            {
                xzsum[i] += (float)pow(Coordinates[k][0], i) * Coordinates[k][2];
                yzsum[i] += (float)pow(Coordinates[k][1], i) * Coordinates[k][2];
            }
        }
        float rootx1;
        float rootx2;
        float rooty1;
        float rooty2;
        float f_x;
        float f_y;
        std::cout << "max_x : " << max_x << " / min_x : " << min_x << "\n";
        std::cout << "max_y : " << max_y << " / min_y : " << min_y << "\n";
        if (abs(max_x - min_x) > RADIUS)
        {
            matrixP(xsum, xzsum, coeff_x);
            std::cout << "firstx : " << Coordinates[1][0] << " / lastx : " << Coordinates[DATA_SIZE - 1][0] << "\n";
            if (Coordinates[DATA_SIZE - 1][0] > Coordinates[1][0])
            {
                f_x = (-coeff_x[1] - sqrt(coeff_x[1] * coeff_x[1] - 4 * coeff_x[2] * coeff_x[0])) / (2 * coeff_x[2]);
            }
            else
            {
                f_x = (-coeff_x[1] + sqrt(coeff_x[1] * coeff_x[1] - 4 * coeff_x[2] * coeff_x[0])) / (2 * coeff_x[2]);
            }
        }
        else
        {
            if (Coordinates[DATA_SIZE - 1][0] > Coordinates[1][0])
            {
                f_x = max_x + (max_x - min_x)*CALIB;
            }
            else
            {
                f_x = min_x + (min_x - max_x)*CALIB;
            }
        }

        if (abs(max_y - min_y) > RADIUS)
        {
            std::cout << "firsty : " << Coordinates[1][1] << " / lasty : " << Coordinates[DATA_SIZE - 1][1] << "\n";
            matrixP(ysum, yzsum, coeff_y);
            if (Coordinates[DATA_SIZE - 1][1] > Coordinates[1][1])
            {
                f_y = (-coeff_y[1] - sqrt(coeff_y[1] * coeff_y[1] - 4 * coeff_y[2] * coeff_y[0])) / (2 * coeff_y[2]);
            }
            else
            {
                f_y = (-coeff_y[1] + sqrt(coeff_y[1] * coeff_y[1] - 4 * coeff_y[2] * coeff_y[0])) / (2 * coeff_y[2]);
            }
        }
        else
        {
            if (Coordinates[DATA_SIZE - 1][1] > Coordinates[1][1])
            {
                f_y = max_y + (max_y - min_y)*CALIB;
            }
            else
            {
                f_y = min_y + (min_y - max_y)*CALIB;
            }
        }
        //coeff_x[0] += KINECT_HEIGHT;
        //coeff_y[0] += KINECT_HEIGHT;
        /*
        ros::Publisher dummy_point_x = nh.advertise<geometry_msgs::PointStamped>("dummy_point_x", 100);
        ros::Publisher dummy_point_y = nh.advertise<geometry_msgs::PointStamped>("dummy_point_y", 100);
        geometry_msgs::PointStamped px;
        px.header.frame_id = "rgb_camera_link";

        ros::Rate loop(1000);
        
        for (int i = -1000; i <= 1000; i++)
        {
            px.point.x = i;
            px.point.z = coeff_x[2] * i * i + coeff_x[1] * i + coeff_x[0];
            px.point.y = 0;
            dummy_point_x.publish(px);
            loop.sleep();
        }
        for (int i = -1000; i <= 1000; i++)
        {
            px.point.y = i;
            px.point.z = coeff_y[2] * i * i + coeff_y[1] * i + coeff_y[0];
            px.point.x = 0;
            dummy_point_y.publish(px);
            loop.sleep();
        }
        */
        std::cout << coeff_x[2] << "x^2 + (" << coeff_x[1] << ")x + (" << coeff_x[0] << ")\n";
        std::cout << coeff_y[2] << "y^2 + (" << coeff_y[1] << ")y + (" << coeff_y[0] << ")\n";
        
        geometry_msgs::Pose2D roots;
        roots.theta = 0;
        roots.x = f_x;
        roots.y = f_y;
        std::cout << "f_x : " << f_x << " / f_y : " << f_y << "\n";

        if(abs(f_x) > WORKSPACE_RADIUS /*abs(f_y) > WORKSPACE_RADIUS*/){
            ROS_ERROR("OUT_OF_WORKSPACE");
        }
        else{
            ros::Rate loop_rate(2000);
            for (int i = 0; i < 1; i++)
            {
                location_pub.publish(roots);
                loop_rate.sleep();
            }
        }
        max_x = -10000.0;
        min_x = 10000.0;
        max_y = -10000.0;
        min_y = 10000.0;
    }

private:
    ros::NodeHandle nh;

    ros::Publisher location_pub = nh.advertise<geometry_msgs::Pose2D>("location_std::vector", 1);
    //image_transport::ImageTransport it_;
    //cv_bridge::CvImagePtr cv_ptr;
    cv::Mat im_rgb;
    cv::Mat im_dph;
    bool object_found = false;

    //CascadeClassifier ball_cascade;
    int x = -1;
    int y = -1;
    float r = -1;
    float depth = -1;

    float max_x = -10000.0;
    float min_x = 10000.0;
    float max_y = -10000.0;
    float min_y = 10000.0;

    int n1 = 0;
    int n2 = 0;
    int _x = 0;
    int _y = 0;

    std::vector<cv::Vec3f> Coordinates;
    std::vector<cv::Vec6i> objectSet;

    int objectID = 0;

    k4a_calibration_t *calibration;
    k4a_device_t *dev = new k4a_device_t;
    k4a_float3_t *outputdata;

    long long t1_s = 0;
    long long t1_ns = 0;
    long long t2_s = 0;
    long long t2_ns = 0;
    //cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create("KCF");
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "percept_object");

    perceptObject P;

    return 0;
}
