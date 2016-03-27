
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

LaneDetectorConf lanesConf, stoplinesConf,lanesConf1; 
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



int trackcount=0;
cv::Mat prev_frame;
CvMat* mymymcvGetBezierSplinePixels(Spline &spline, float h, CvSize box,
                                bool extendSpline)
{
 int xtndid=3;
  //get the points belonging to the spline
  CvMat *tangents = cvCreateMat(2, 2, CV_32FC1);
  CvMat * points = mcvEvalBezierSpline(spline, h, tangents);

 /* cout<<endl<<endl<<"These are Spline Pixels"<<endl;
for(int px=0;px<4;px++)
       cout<<cvPointFrom32f(spline.points[px]).x<<" "<<cvPointFrom32f(spline.points[px]).y<<endl;
  
cout<<"These are inpoints"<<endl;
 for (int i=0; i<points->height; i++)
 {
    CV_MAT_ELEM(*points, float, i, 0) = cvRound(CV_MAT_ELEM(*points, float, i, 0));
    cout<<int( CV_MAT_ELEM(*points, float, i, 0))<<" ";
    CV_MAT_ELEM(*points, float, i, 1) = cvRound(CV_MAT_ELEM(*points, float, i, 1));
    cout<<int(CV_MAT_ELEM(*points, float, i, 1))<<endl;
 }*/

  //pixelize the spline
  //CvMat *inpoints = cvCreateMat(points->height, 1, CV_8SC1);
  //cvSet(, CvScalar value, const CvArr* mask=NULL);
  list<int> inpoints;
  list<int>::iterator inpointsi;
  int lastin = -1, numin = 0;
  for (int i=0; i<points->height; i++)
  {
    //round
    CV_MAT_ELEM(*points, float, i, 0) = cvRound(CV_MAT_ELEM(*points, float, i, 0));
    CV_MAT_ELEM(*points, float, i, 1) = cvRound(CV_MAT_ELEM(*points, float, i, 1));

    //check boundaries
    if(CV_MAT_ELEM(*points, float, i, 0) >= 0 &&
      CV_MAT_ELEM(*points, float, i, 0) < box.width &&
      CV_MAT_ELEM(*points, float, i, 1) >= 0 &&
      CV_MAT_ELEM(*points, float, i, 1) < box.height)
    {
      //it's inside, so check if the same as last one
      if(lastin<0 ||
        (lastin>=0 &&
        !(CV_MAT_ELEM(*points, float, lastin, 1)==
        CV_MAT_ELEM(*points, float, i, 1) &&
        CV_MAT_ELEM(*points, float, lastin, 0)==
        CV_MAT_ELEM(*points, float, i, 0) )) )
      {
        //put inside
        //CV_MAT_ELEM(*inpoints, char, i, 0) = 1;
        inpoints.push_back(i);
        lastin = i;
        numin++;
      }
    }
  }

  //check if to extend the spline with lines
  CvMat *pixelst0, *pixelst1;

    if (extendSpline)
    {
    //get first point inside
    int p0 = inpoints.front();
    //extend from the starting point by going backwards along the tangent
    //line from that point to the start of spline
    Line line;
    
    line.startPoint = cvPoint2D32f(CV_MAT_ELEM(*points, float, p0, 0) - xtndid *
                                   CV_MAT_ELEM(*tangents, float, 0, 0),
                                   CV_MAT_ELEM(*points, float, p0, 1) - xtndid *
                                   CV_MAT_ELEM(*tangents, float, 0, 1));
    line.endPoint = cvPoint2D32f(CV_MAT_ELEM(*points, float, p0, 0),
                                 CV_MAT_ELEM(*points, float, p0, 1));
    //intersect the line with the bounding box
    mcvIntersectLineWithBB(&line, cvSize(box.width-1, box.height-1), &line);
    //get line pixels
    pixelst0 = mcvGetLinePixels(line);
    numin += pixelst0->height;

    //get last point inside
    int p1 = inpoints.back();
    //extend from end of spline along tangent
    line.endPoint = cvPoint2D32f(CV_MAT_ELEM(*points, float, p1, 0) +xtndid *
                                 CV_MAT_ELEM(*tangents, float, 1, 0),
                                 CV_MAT_ELEM(*points, float, p1, 1) + xtndid *
                                 CV_MAT_ELEM(*tangents, float, 1, 1));
    line.startPoint = cvPoint2D32f(CV_MAT_ELEM(*points, float, p1, 0),
                                   CV_MAT_ELEM(*points, float, p1, 1));
    //intersect the line with the bounding box
    mcvIntersectLineWithBB(&line, cvSize(box.width-1, box.height-1), &line);
    //get line pixels
    pixelst1 = mcvGetLinePixels(line);
    numin += pixelst1->height;
  }

  //put the results in another matrix
  CvMat *rpoints;
  if (numin>0)
    rpoints = cvCreateMat(numin, 2, CV_32SC1);
  else
  {
    return NULL;
  }


  //first put extended line segment if available
  if(extendSpline)
    {
    //copy
    memcpy(cvPtr2D(rpoints, 0, 0), pixelst0->data.fl,
           sizeof(float)*2*pixelst0->height);
  }

  //put spline pixels
  
  int ri = extendSpline ? pixelst0->height : 0;
  for (inpointsi=inpoints.begin();
  inpointsi!=inpoints.end(); ri++, inpointsi++)
  {
    CV_MAT_ELEM(*rpoints, int, ri, 0) = (int)CV_MAT_ELEM(*points,
                                                         float, *inpointsi, 0);
    CV_MAT_ELEM(*rpoints, int, ri, 1) = (int)CV_MAT_ELEM(*points,
                                                         float, *inpointsi, 1);
  }

  //put second extended piece of spline
  if(extendSpline)
    {
    //copy
    memcpy(cvPtr2D(rpoints, ri, 0), pixelst1->data.fl,
           sizeof(float)*2*pixelst1->height);
    //clear
    cvReleaseMat(&pixelst0);
    cvReleaseMat(&pixelst1);
  }


  //release
  //    cvReleaseMat(&inpoints);
  cvReleaseMat(&points);
  cvReleaseMat(&tangents);
  inpoints.clear();

  //return

 /* cout<<"These are rnpointsafter extension "<< Mat(rpoints)<<endl;
   Mat tmp_points = Mat(rpoints);
   cout<<"wait"<<tmp_points.at<int>(tmp_points.rows-1,0)<<" "<<tmp_points.at<int>(tmp_points.rows-1,1)<<endl;*/
   spline.points[0].x = CV_MAT_ELEM(*rpoints, int, 0, 0);//tmp_points.at<int>(0,0);
   spline.points[0].y=CV_MAT_ELEM(*rpoints, int, 0, 1);//tmp_points.at<int>(0,1);
  //  spline.points[3].x=CV_MAT_ELEM(*rpoints, int, rpoints->height-1, 0);//tmp_points.at<int>(tmp_points.rows,0);
  // spline.points[3].y=CV_MAT_ELEM(*rpoints, int, rpoints->height-1, 1);//tmp_points.at<int>(tmp_points.rows,1);
  
   return rpoints;
}

void learn(Mat input, Mat *output,vector<Spline> &splines)
{
    int i,j;
      trackcount++;
      if(trackcount==1)
      {
      prev_frame = cv::Mat::zeros(input.rows,input.cols, CV_8UC1);
      }
       cv::Mat track = cv::Mat::zeros(input.rows,input.cols, CV_8UC1);

       cv::Mat tmp_out = cv::Mat::zeros(input.rows,input.cols, CV_8UC1);
      cv::Mat final = cv::Mat::zeros(input.rows,input.cols, CV_8UC1);
       cv::Mat result = cv::Mat::zeros(input.rows,input.cols, CV_8UC1);

        CvMat *pixels;

          for(i=0; i<splines.size(); i++)
          {
            cout<<"herereerer"<<endl;
                  pixels = mymymcvGetBezierSplinePixels(splines[i], 0.05,
                                                   cvSize(input.cols , input.rows),
                                                  true);
                  for (int i=0; i<pixels->height-1; i++)
                  {
                    
                      int x1=(int)cvGetReal2D(pixels, i, 0); 
                      int y1=(int)cvGetReal2D(pixels, i, 1);
                      int x2=(int)cvGetReal2D(pixels, i+1, 0); 
                      int y2=(int)cvGetReal2D(pixels, i+1, 1);

                      Point pt1 =  Point(x1,y1);
                      Point pt2 =  Point(x2, y2);
                    //  track.at<uchar>(y,x)=255;
                    cv::LineIterator it(input, pt1, pt2, 4);
                    std::vector<cv::Vec3b> buf(it.count);
                    std::vector<cv::Point> points(it.count);

                    for(int i = 0; i < it.count; i++, ++it)
                    {
                      buf[i] = (const Vec3b)*it;
                       points[i] = it.pos();
                       if(points[i].y>200);
                        track.at<uchar>(points[i].y,points[i].x)=255;
                    }
                  }
         }
         // imshow( "extended spline", track );  

    for( i = 3; i < tmp_out.rows-3; i++)
      {
        for(j =3; j < tmp_out.cols-3; j++)
        {
           if(track.at<uchar>(i,j)>0 )
             {
              for(int a=-3;a<=3;a++)
              {
                for(int b=-3;b<=3;b++)
                { 
                  if(i+a>200);
                  result.at<uchar>(i+a,j+b)=20;
                } 
              } 
             }

          }
      }
          // result=track.clone();
    for( i = 200; i < result.rows; i++)
    {
      for(j = 0; j < result.cols; j++)
      {

        if((result.at<uchar>(i,j))!= 0 && (prev_frame.at<uchar>(i,j))!=0)
        {

          final.at<uchar>(i,j)=result.at<uchar>(i,j)+prev_frame.at<uchar>(i,j);
           if(final.at<uchar>(i,j)>250)
             final.at<uchar>(i,j)=255;

        }  
        else if(result.at<uchar>(i,j)!= 0 )
        {
          if(result.at<uchar>(i,j)>=10)
          final.at<uchar>(i,j)=result.at<uchar>(i,j)-10;
         else
          final.at<uchar>(i,j)=0;

        }
        else
        {
          if(prev_frame.at<uchar>(i,j)>10)
          final.at<uchar>(i,j)=prev_frame.at<uchar>(i,j)-10;
         else
          final.at<uchar>(i,j)=0;
        }
       }

    } 
    

    
     if(trackcount>1)
     {

      prev_frame=final.clone();
      *output=final;
    }

   else
   {
    prev_frame=result.clone();
    *output=result;
    imshow( "extended track", *output );  
   }

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

 //LaneDetectorConf stoplinesConf=stoplinesConf;
 //cout<<" stoplinesConf1.ipmWidth        : ::::: :: "<<stoplinesConf1.ipmWidth<<" ";
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

          if(splines.size()!=0) {
      vector<Line> subimageLines;
    vector<Spline> subimageSplines;
    vector<float> subimageLineScores, subimageSplineScores;

          Mat temp_input ;


          CvMat temp_input_cv,temp_output_cv; 

          cvtColor(Mat(imDisplay),temp_input, CV_BGR2GRAY)  ;
           cv::Mat temp_output = cv::Mat::zeros(temp_input.rows,temp_input.cols, CV_8UC1);
         
          temp_input_cv=temp_input;

          cout<<"QWERTYUIOP"<<splines.size()<<endl;

          learn(temp_input,&temp_output , splines );
        

 imshow( "my track results", temp_output );  

          temp_output_cv=temp_output;

         //SHOW_IMAGE(&temp_output_cv, "Detected Lanes");
          //sleep(1);
 //LineState* state;
 LineState newState;
  //if(!state) state = &newState;

  //LaneDetectorConf stoplinesConf1=stoplinesConf;
  lanesConf1.ipmWidth=1266;
  lanesConf1.ipmHeight=370;
  lanesConf1.groupingType=1;

  //SHOW_IMAGE(&temp_output_cv, "output");
  //SHOW_IMAGE(&temp_input_cv, "input");

          mcvGetLines(&temp_output_cv, LINE_VERTICAL, subimageLines, subimageLineScores,
       subimageSplines, subimageSplineScores, &lanesConf1,
        &newState);

        
             cout<<"YYYYYYYYYYYYYYy"<<endl;




          //mymcvDrawSpline(imDisplay, subimageSplines, CV_RGB(0,255,0), 3);
          for(int i=0; i<subimageSplines.size(); i++)
          {
            /*if (splines[i].color == LINE_COLOR_YELLOW)
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
            }*/
            //else
              //{
                
                CvMat *pixels = mcvGetBezierSplinePixels(subimageSplines[i], .05,
                                               cvSize(imDisplay->width, imDisplay->height),
                                               true);

                /*if (pixels )
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
                }*/
    
              mcvDrawSpline(imDisplay, subimageSplines[i], CV_RGB(0,255,0), 3);
              //}
            
            if (options.show_lane_numbers_flag)
            {
              char str[256];
              sprintf(str, "%d", i);
              mcvDrawText(imDisplay, str,
                          cvPointFrom32f(subimageSplines[i].points[splines[i].degree]),
                                          1, CV_RGB(0, 0, 255));
            }
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
        mcvInitLaneDetectorConf(options.lanes_conf_arg, &lanesConf); 
         mcvInitLaneDetectorConf(options.lanes_conf_arg, &lanesConf1);
         //defined in lanedetector.cc
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

   for(int seqid=0;seqid<=400;seqid++)
  {     
    std::ostringstream ss;
    ss << std::setw(6) << std::setfill('0') << seqid;
    std::string s2(ss.str());
    string filename_in = "/home/danish/Desktop/Dataset1/Kitti/04/image_0/"+s2+".png";
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

