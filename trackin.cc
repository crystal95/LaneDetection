
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "iostream"
using namespace cv;
using namespace std;



void lane_tracking(Mat input, vector<Spline> &splines,int frame)
{ 
    int max_x=0,max_y=0,min_x=input.rows,min_y=input.cols;

    int i,j,frame;
    int x0,x2,x3,x1,y0,y1,y2,y2;
    x0 = cvPointFrom32f(subimageSplines[0].points[0]).x;
    y0=cvPointFrom32f(subimageSplines[0].points[0]).y;
    x1 = cvPointFrom32f(subimageSplines[0].points[1]).x;
    y1=cvPointFrom32f(subimageSplines[0].points[1]).y;
    x2 = cvPointFrom32f(subimageSplines[0].points[2]).x;
    y2=cvPointFrom32f(subimageSplines[0].points[2]).y;
    x3 = cvPointFrom32f(subimageSplines[0].points[3]).x;
    y3=cvPointFrom32f(subimageSplines[0].points[3]).y;
    Mat track(input.rows,input.cols, CV_8UC1); 
     for( i = 0; i < input.rows; i++)
     {
       for(j = 0; j < input.cols; j++)
       {
         track.at<uchar>(x0,y0)=0;
         track.at<uchar>(x1,y1)=0;
         track.at<uchar>(x2,y2)=0;
         track.at<uchar>(x3,y3)=0;
          
       }
   
     }
     for( i = min_x; i <=max_x; i++)
     {
        for(j = min_y; j < max_y; j++)
        {
            track.at<uchar>(i,j)=0;
        }
       
     }

}
    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", track );    

} 


int main( int argc, char** argv )
{
     Mat image;
    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);  
   
    cout<<image.rows<<" "<<image.cols<<image.channels()<<endl;
    cvtColor(image,image, CV_BGR2GRAY);
    cout<<image.rows<<" "<<image.cols<<image.channels()<<endl;


    lane_tracking(&image);


                                           
    return 0;
}