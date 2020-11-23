#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>

#define WIDTH_MIN 0
#define WIDTH_MAX 640

using namespace cv;

static const std::string OPENCV_WINDOW = "Color Detector";
//static const std::string HSV_WINDOW = "HSV Image";
static const std::string Contour_WINDOW = "Contoured Image";

int LowH = 0;
int LowS = 43;
int LowV = 46;
int HighH = 10;
int HighS = 255;
int HighV = 255;

bool anomaly = false;

int n = 0;
double x_pose;
double y_pose;
double prev_x = 0;
double prev_y = 0;
char filename[80];

class ImageConverter{
 ros::NodeHandle nh_;
 image_transport::ImageTransport it_;
 image_transport::Subscriber image_sub_;
 image_transport::Publisher image_pub_;

public:
 ImageConverter()
   : it_(nh_){
   image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
   image_pub_ = it_.advertise("/camera/image_processed", 1);
   cv::namedWindow(OPENCV_WINDOW);
 }

 ~ImageConverter(){
   cv::destroyWindow(OPENCV_WINDOW);
 }

 void imageCb(const sensor_msgs::ImageConstPtr& msg){
   cv_bridge::CvImagePtr cv_ptr;
   try{
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e){
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }

  int iLowH, iHighH;
  int iLowS = 43, iHighS = 255, iLowV = 46, iHighV = 255;
  iLowH = 0;
  iHighH = 10;//red
  Mat imgHSV;
  cv::cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV);//转为HSV

  Mat imgThresholded;
  /*
   cv::Mat hsv_img, mask_img;
   cvtColor(cv_ptr->image, hsv_img, COLOR_BGR2HSV);
   cv::inRange(hsv_img, cv::Scalar(LowH, LowS, LowV), cv::Scalar(HighH, HighS, HighV), mask_img);
   cv::erode(mask_img, mask_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
   cv::dilate(mask_img, mask_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

   cv::dilate(mask_img, mask_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
   cv::erode(mask_img, mask_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

   //cv::namedWindow(HSV_WINDOW);
   //cv::imshow(HSV_WINDOW, hsv_img);
   //cv::waitKey(3);

   cv::imshow(OPENCV_WINDOW, mask_img);
   cv::waitKey(3);

   std::vector<std::vector<cv::Point> > contours;
   std::vector<Vec4i> hierarchy;
   cv::RNG rng(12345);
   cv::findContours(mask_img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

   cv::Mat drawing = Mat::zeros(mask_img.size(), CV_8UC3);
   for( int i = 0; i< contours.size(); i++ )
   {
    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
    }

  cv::namedWindow(Contour_WINDOW);
  cv::imshow(Contour_WINDOW, drawing);
  cv::waitKey(3);
*/
  cv::inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
  //开操作 (去除一些噪点)  如果二值化后图片干扰部分依然很多，增大下面的size
  cv::Mat element = getStructuringElement(MORPH_RECT, Size(20, 20));
  cv::morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

    //闭操作 (连接一些连通域)
  cv::morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

  //cv::namedWindow("Thresholded Image",CV_WINDOW_NORMAL);
  //cv::imshow("Thresholded Image", imgThresholded);

  std::vector<std::vector <cv::Point> > contours;
  std::vector<Vec4i> hierarchy;
  cv::findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);//查找轮廓

  int count = 0;
  int point_x ;
  cv::Point pt[512];//存储连通区域个数
  cv::Moments moment;//矩
  std::vector<Point> Center;//创建一个向量保存重心坐标

  //cv::Mat drawing = Mat::zeros(imgThresholded.size(), CV_8UC3);
  for (int i=0;i<contours.size();i++)//读取每一个轮廓求取重心
  {
      point_x = 0;
      cv::Mat temp(contours.at(i));
      Scalar color(0, 255, 255);//yellow
      moment = moments(temp, false);

      if (moment.m00 != 0)//除数不能为0
      {
        pt[i].x = cvRound(moment.m10 / moment.m00);//计算重心横坐标
        pt[i].y = cvRound(moment.m01 / moment.m00);//计算重心纵坐标
        point_x = pt[i].x;
      }
      cv::Point p = Point(pt[i].x, pt[i].y);//重心坐标
      std::cout << pt[i].x << "," << pt[i].y << std::endl;
      cv::circle(cv_ptr->image, p, 5, color, 2, 8, 0);//原图画出重心坐标
      count++;//重心点数或者是连通区域数
      Center.push_back(p);//将重心坐标保存到Center向量中
  }

    std::cout << "moment numbers：" << Center.size() << std::endl;
    std::cout << "counter numbers：" << contours.size() << std::endl;

    cv::imshow(OPENCV_WINDOW,cv_ptr->image);
    cv::waitKey(3);
    point_x = 0;

  //imshow("result", inputImage);
  //string name = "end"+int2string(color)+".jpg";
  // imwrite(name, img);
  //int result = (point_x > WIDTH_MIN) && (point_x < WIDTH_MAX) ? color : 0;
  // return result;
/*
  int largest_area = 2000; //Change value to adjust search size

  for(int i = 0; i< contours.size(); i++ ){
    double area = contourArea(contours[i],false);
    if(area>largest_area){
      if(x_pose>prev_x+0.5 || x_pose<prev_x-0.5){
        sprintf(filename,"Anomaly%d_x:%f_y:%f.png", n++, x_pose, y_pose);
        cv::imwrite(filename,cv_ptr->image);
        anomaly = true;
      }
      if(y_pose>prev_y+0.5 || y_pose<prev_y-0.5){
        sprintf(filename,"Anomaly%d_x:%f_y:%f.png", n++, x_pose, y_pose);
        cv::imwrite(filename,cv_ptr->image);
        anomaly = true;
      }
    }
  }
*/
   image_pub_.publish(cv_ptr->toImageMsg());
  }
};

ros::Subscriber odom_sub_;

void clbk_asd(const nav_msgs::Odometry::ConstPtr& asd){
  x_pose = asd->pose.pose.position.x;
  y_pose = asd->pose.pose.position.y;
  if(anomaly){
    ROS_INFO("Anomaly detected at: x: %f, y: %f",x_pose, y_pose);
    prev_x = x_pose;
    prev_y = y_pose;
    anomaly = false;
    return;
  }
}

int main(int argc, char** argv){

 cv::namedWindow("Control");
 cv::createTrackbar("LowerH:", "Control", &LowH, 179, NULL);
 cv::createTrackbar("LowerS:", "Control", &LowS, 255, NULL);
 cv::createTrackbar("LowerV:", "Control", &LowV, 255, NULL);
 cv::createTrackbar("UpperH:", "Control", &HighH, 179, NULL);
 cv::createTrackbar("UpperS:", "Control", &HighS, 255, NULL);
 cv::createTrackbar("UpperV:", "Control", &HighV, 255, NULL);
 ros::init(argc, argv, "color_detection");

 ros::NodeHandle n;
 ImageConverter ic;

 ros::spinOnce();

 //odom_sub_ = n.subscribe("/odom", 1, clbk_asd);

 ros::spin();

 return 0;
}
