
#include "main.hh"



#include <stdio.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>

#include <cv.h>
#include <highgui.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stereo_msgs/RectifiedImage.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "stereo_msgs/RectifiedImage.h"
#include <cv_bridge/cv_bridge.h>
#include "stereo_msgs/DisparityImage.h"
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include "stereo_msgs/RoadSurfacePlane.h"
#include "stereo_msgs/LanePoints.h"
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
// LaneDetection specific includes
#include "cmdline.h"
#include "LaneDetector.hh"


using namespace cv;
using namespace std;

int cnt=0;

bool debug_mode = true;
bool road_recieved = false;
/************************************Parameters***********************************/
int distanceThreshold = 35;
int max_height_ = 100;
int min_height_ = 0;
int min_depth_ = 250;
int max_depth_ = 60;

/*********************************************************************************/
/************************Lane Detection Variables*********************************/

int i=0,ans=0;
int d=0;
int flag=0,cnt1=0;
float mycount=0;
cv::Mat raw_mat1;
cv::Mat image(10,1,CV_32FC3, Scalar(0,0,0));
int argC;
char **argV;  
std_msgs::Float32MultiArray arrx;
std_msgs::Float32MultiArray arry;



#define MSG(fmt, ...) \
  (fprintf(stdout, "%s:%d msg   " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? 0 : 0)

#define ERROR(fmt, ...) \
  (fprintf(stderr, "%s:%d error " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : -1)

 
/*********************************************************************************/

namespace LaneDetector
{

gengetopt_args_info options;
CameraInfo cameraInfo;  //defined in IPM.hh

LaneDetectorConf lanesConf, stoplinesConf; 
    /**
     * This function reads lines from the input file into a vector of strings
     *
     * \param filename the input file name
     * \param lines the output vector of lines
     */


  bool ReadLines(const char* filename, vector<string> *lines)
  {
    // make sure it's not NULL
    if (!lines)
      return false;
    // resize
    lines->clear();

    ifstream  file;
    file.open(filename, ifstream::in);
    char buf[5000];
    // read lines and process
    while (file.getline(buf, 5000))
    {
      string str(buf);
      lines->push_back(str);
    }
    // close
    file.close();
    return true;
  }


    /**
     * This function processes an input image and detects lanes/stoplines
     * based on the passed in command line arguments
     *
     * \param filename the input file name
     * \param cameraInfo the camera calibration info
     * \param lanesConf the lane detection settings
     * \param stoplinesConf the stop line detection settings
     * \param options the command line arguments
     * \param outputFile the output file stream to write output lanes to
     * \param index the image index (used for saving output files)
     * \param elapsedTime if NOT NULL, it is accumulated with clock ticks for
     *        the detection operation MSG("RASHID SODHI IS A GOOD BOY");
     */
     void ProcessImage( CameraInfo& cameraInfo,
                  LaneDetectorConf& lanesConf, LaneDetectorConf& stoplinesConf,
                  gengetopt_args_info& options, ofstream* outputFile,
                  int index, clock_t *elapsedTime,cv::Mat raw_mat2)  //defined in IPM.hh
    {
      
   
IplImage* im;
      im = cvCreateImage(cvSize(raw_mat2.cols,raw_mat2.rows),8,3);
      IplImage ipltemp = raw_mat2;
      cvCopy(&ipltemp,im);
      CvMat temp;
      
      cvGetMat(im, &temp);
      CvMat *schannel_mat;
      CvMat* tchannelImage = cvCreateMat(im->height, im->width, INT_MAT_TYPE);
      cvSplit(&temp, tchannelImage, NULL, NULL, NULL);
      // convert to float
      CvMat *channelImage= cvCreateMat(im->height, im->width, FLOAT_MAT_TYPE);
      cvConvertScale(tchannelImage, channelImage, 1./255);
      vector<FLOAT> lineScores, splineScores;
      vector<Line> lanes;
      vector<Spline> splines;

      mcvGetLanes(channelImage, &temp, &lanes, &lineScores, &splines, &splineScores,   //1st image=5
                  &cameraInfo, &lanesConf, NULL);  
      
      CvMat *imDisplay = cvCloneMat(&temp);

      
     // if (options.show_flag || options.save_images_flag)
      if(1)
      {
               cout<<"DUDE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  "<<endl;

        if (lanesConf.ransacLine && !lanesConf.ransacSpline)
          {
 
            for(int i=0; i<lanes.size(); i++)
           {
            
            mcvDrawLine(imDisplay, lanes[i], CV_RGB(0,125,0), 3);
            
          }
        }
        cout<<"heu"<<endl;
        // print lanes
        if (lanesConf.ransacSpline)
        {
          for(int i=0; i<splines.size(); i++)
          {
            if (splines[i].color == LINE_COLOR_YELLOW)
            {
               CvMat *pixels = mcvGetBezierSplinePixels(splines[i], .05,
                                           cvSize(imDisplay->width, imDisplay->height),
                                           false);
              if (pixels )
              {

               // int x1=(cvGetReal2D(pixels, 0, 0));
                //int y1= cvGetReal2D(pixels, 0, 1);
                //int x2=(cvGetReal2D(pixels, 19, 0));
                //int y2=cvGetReal2D(pixels, 19, 1);
                //arrx.data.push_back(x1);
                //arrx.data.push_back(y1);
                //arrx.data.push_back(x2);
                //arrx.data.push_back(y2);
               

              }

              mcvDrawSpline(imDisplay, splines[i], CV_RGB(255,255,0), 3);
            }
            else
              {
                
                CvMat *pixels = mcvGetBezierSplinePixels(splines[i], .05,
                                               cvSize(imDisplay->width, imDisplay->height),
                                               true);
                if (pixels )
                {
                   
                  
                  
                 // int x1=(cvGetReal2D(pixels, 19, 0));
                  //int y1=cvGetReal2D(pixels, 19, 1);
                  if(1==1)
                  {
                    for(int it=0;it<=19;it++)
                    {
                      
                     // int x=(cvGetReal2D(pixels, it, 0));
                      ///int y= cvGetReal2D(pixels, it, 1);

                      //arrx.data.push_back(x);

                      //arrx.data.push_back(y);
                      
                      //cout<<x<<" "<<y<<endl;
                      //cvCircle( imDisplay, cvPoint(x,y) , 8, CV_RGB(255, 0, 0),-1 );

                   
                     
                    
                    }
                    cout<<endl;
                    //arrx.data.push_back(-1);
                    //arrx.data.push_back(-1);
                  //   mymsg = cv_bridge::CvImage(std_msgs::Header(), "32FC3", image).toImageMsg();

                  }
                  else
                  {
                    for(int it=0;it<=19;it++)
                    {
                      //int x=(cvGetReal2D(pixels, it, 0));
                      //int y= cvGetReal2D(pixels, it, 1);
                      
                     // arry.data.push_back(x);
                      //arry.data.push_back(y);
                      
                     // cvCircle( imDisplay, cvPoint(x,y) , 8, CV_RGB(0, 0, 255),-1 );
                      
                      //cvCircle( imDisplay, cvPoint(x2,y2) , 8, CV_RGB(255, 0, 0),-1 );
                    
                    }
                  }
                }
                mcvDrawSpline(imDisplay, splines[i], CV_RGB(0,255,0), 3);
              }
            
            if (options.show_lane_numbers_flag)
            {
              char str[256];
              sprintf(str, "%d", i);
              mcvDrawText(imDisplay, str,
                          cvPointFrom32f(splines[i].points[splines[i].degree]),
                                          1, CV_RGB(0, 0, 255));
            }
          }
        }
        // show?
        //if (options.show_flag)
        if(1)
        {
          cnt++;
          // set the wait value
          int wait = 1;//options.step_flag ? 0 : options.wait_arg;
          // show image with detected lanes
          SHOW_IMAGE(imDisplay, "Detected Lanes", wait);
          cout<<"DAnish SOdhi  "<<"width is "<<imDisplay->width<<" " <<"height is : "<<imDisplay->height<<endl;
          
  stringstream ss2;
  ss2 << cnt;
  string filename_out="/home/danish/Downloads/caltech-lane-detection/results/"+ ss2.str() +".jpg";
  imwrite(filename_out,Mat(imDisplay));

        }
        cvReleaseMat(&imDisplay);
      }

    }



    int initConfiguration()
    {
      // parse the command line paramters
      if (cmdline_parser (argC, argV,  &options) < 0)
        return -1;

      // read the camera configurations
      cout<<argC<<endl;
      for(int i=0;i<argC;i++)
      {
        cout<<argV[i]<<endl;
      }

      cout<< options.camera_conf_arg <<endl;
      
      mcvInitCameraInfo(options.camera_conf_arg, &cameraInfo); //defined in inverseperspectivemaping.cc
      MSG("Loaded camera file");

      // read the configurations
      if (!options.no_lanes_flag)
      {
        mcvInitLaneDetectorConf(options.lanes_conf_arg, &lanesConf); //defined in lanedetector.cc
        MSG("Loaded lanes config file");
      }
      if (!options.no_stoplines_flag)
      {
        mcvInitLaneDetectorConf(options.stoplines_conf_arg, &stoplinesConf);
        MSG("Loaded stop lines config file");
      }

      // set debug to true
      if (options.debug_flag)
         DEBUG_LINES = 0;

       
      return 0;
    }


}
using namespace LaneDetector;
int main(int argc, char **argv)
{

  
  argC = argc;
  argV = argv;
   cv::namedWindow("view");
  
  
 


  if(initConfiguration()<0)
  {
   cout<<("something is wrong BHAU!");
  }

   for(int seqid=501;seqid<=1200;seqid++)
  {     
    std::ostringstream ss;
    ss << std::setw(6) << std::setfill('0') << seqid;
    std::string s2(ss.str());
    string filename_in = "/home/danish/Desktop/Dataset1/Kitti/17/image_0/"+s2+".png";
    cout<<filename_in<<endl;

   Mat image = imread(filename_in, CV_LOAD_IMAGE_COLOR); 
     //namedWindow( "Display window", WINDOW_AUTOSIZE );
   // imshow( "Display window",image);  
 //     
   ProcessImage(cameraInfo, lanesConf, stoplinesConf,
                        options, NULL, 0, 0 ,image);

       
}                          
  return 0; 

}


// main entry point

