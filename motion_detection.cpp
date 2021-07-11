//To compile:  	cd /root/PICAM_CV/build
//		cmake .. -Draspicam_DIR=/root/raspicam-0.1.1/build
//		make
// 		then run ./simpletest_raspicam_cv

#include <wiringPi.h>
#include <ctime>
#include <iostream>
#include <stdio.h>
#include <raspicam/raspicam_cv.h>
#include <fstream>

#include "/usr/local/include/opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "/root/opencv-2.4.10/include/opencv/cv.h"
#include "/root/opencv-2.4.10/include/opencv/highgui.h"
#include "/usr/local/include/opencv2/core/core.hpp"
//#include "/usr/local/include/opencv2/imgproc/imgproc.hpp"



using namespace std;
using namespace cv;


int main(int argc, char **argv) {
				//******** Camera object, camera prop setup, frames variables, wiringPi setup ... **********//

	
	wiringPiSetup();
	pinMode(7, OUTPUT);

	int theObject[2] = {0, 0};
	Rect objectBoundingRectangle = Rect(0,0,0,0);
	int objw;
	int objh;
	int xpos;
	int ypos;
        int i=0;
	int j=0;
	int k=0;
	int output=0;
	int xposnew;
	int yposnew;
	int tl[2] = {0, 0};
	int tr[2] = {0, 0};
        int bl[2] = {0, 0};
        int br[2] = {0, 0};
	int xmin = 224;
	int xmax = 224;
	int ymin = 168;
	int ymax = 168;
	int init_scan = 0;
	int testgpio;
	int init_start = 0;
	int count = 0;
	int inside;

	raspicam::RaspiCam_Cv Camera;
	cv::Mat frame1;
	cv::Mat frame2;
	cv::Mat diff;
	cv::Mat threshold;
	vector< vector <cv::Point> >contours;//contours are vectors of vector of points
	vector<cv::Vec4i> hierarchy;

	Camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, 448);
	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 336);
        Camera.set(CV_CAP_PROP_FPS, 20);

 	if(!Camera.open()){ cerr<<"Error opening the camera"<<endl; return -1;}

	while(1){
	// Initialize start-up process
	if(k==0 && digitalRead(12))
		init_start = 1;

	while(init_start){
		if(count == 0){
			cout <<"Started reading of min/max\n\n";
			count ++;
			delay(1000);
		}
		Camera.grab();
	        Camera.retrieve(frame1);
        	//cv::imwrite("frame1.jpg", frame1);

                                //*********** Frame Averaging ************//
        	i=0;
        	double start = clock();

		while(i<3){
                	Camera.grab();
                	Camera.retrieve(frame2);
               		// addWeighted(frame1, 0.25*(i+1), frame2, 0.25, 0.0, frame1);
                	addWeighted(frame1, 0.5, frame2, 0.5, 0.0, frame1);
                	i++;
                	}


        	Camera.grab();
        	Camera.retrieve(frame2);
        	
                                //*********** Image subtracting, thresholding, and $*/
	        cv::absdiff(frame1, frame2, diff);
		       
                //blur to get rid of noise
	        cv::threshold (diff, threshold, 40, 255, CV_THRESH_BINARY);
       		blur(threshold, threshold, cv::Size(10, 10));
        	cv::threshold(diff, threshold, 40, 255, CV_THRESH_BINARY);
        	findContours(threshold, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	       	if(contours.size() > 0){
                	//system("gpio write 7 1");
                	vector< vector<cv::Point> > largestContourVec;
                	largestContourVec.push_back(contours.at(contours.size()-1));
                	objectBoundingRectangle = boundingRect(largestContourVec.at(0));

                	xpos = objectBoundingRectangle.x;
                	ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;
		}
		//*****************************Find Boundaries************************//


	if(contours.size() > 10){
	if(init_scan == 0){
		xmin = xpos;
		xmax = xpos;
		ymin = ypos;
		ymax = ypos;
		init_scan++;
		}
	else{
		if(xpos < xmin)
			xmin = xpos;
		if(xpos > xmax)
			xmax = xpos;
		if(ypos < ymin)
			ymin = ypos;
		if(ypos > ymax)
			ymax = ypos;

		}

		if(digitalRead(12)){
			k++;
			init_start = 0;
		}

		}
		}

	if(k > 0 && digitalRead(12)){
		init_scan = 1;
		count = 0;
	}

	cout << "Minimum x: " << xmin << endl;
	cout << "Maximum x: " << xmax << endl;
	cout << "Minimum y: " << ymin << endl;
	cout << "Maximum y: " << ymax << endl << endl;
	
	// Terminate start-up process and start scanning for motion
	while(init_scan){

	if(count == 0){
		cout << "Started Scanning." << endl << endl;
		delay(1000);
		count++;
	}

	Camera.grab();
	Camera.retrieve(frame1);
	
				//*********** Frame Averaging ************//
	i=0;
	double start = clock();
		while(i<3){
		Camera.grab();
		Camera.retrieve(frame2);
               // addWeighted(frame1, 0.25*(i+1), frame2, 0.25, 0.0, frame1);
		addWeighted(frame1, 0.5, frame2, 0.5, 0.0, frame1);
		i++;
		}
	Camera.grab();
	Camera.retrieve(frame2);
	
				//*********** Image subtracting, thresholding, and blurring ********//
	cv::absdiff(frame1, frame2, diff);
	cv::threshold (diff, threshold, 40, 255, CV_THRESH_BINARY);
			       // Blur and threshold again to get rid of residue noise after 1st blurring and thresholding
	blur(threshold, threshold, cv::Size(10, 10));
	cv::threshold(diff, threshold, 40, 255, CV_THRESH_BINARY);
	findContours(threshold, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			// Reordering/push back contours if there is any
	if(contours.size() > 0){
	       	vector< vector<cv::Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size()-1));                        
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));

		xpos = objectBoundingRectangle.x;
		ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;
		objw = objectBoundingRectangle.width;
		objh = objectBoundingRectangle.height;
	//Update object position
		theObject[0] = xpos , theObject[1] = ypos;

	//Condition to turn on/off the valve
	if(contours.size() > 10/*&&  objw > 20 && objh > 60*/ && xpos > xmin && xpos < xmax && ypos > ymin && ypos < ymax)
		inside = 1;
	if(contours.size() > 10/* && objw > 20 && objh > 60*/ && (xpos < xmin || xpos > xmax || ypos < ymin || ypos > ymax))
		inside = 0;

        if(inside){
//		cout << "Object Height:             " << objh << endl;
//		cout << "Object Wideth:             " << objw << endl;
		cout << "                           " << contours.size() << endl << endl;
		digitalWrite(7, HIGH);
		cout << "Sprinkler is off" << endl;
	}
	if (!inside){
//              cout << "Object Height:             " << objh << endl;
//              cout << "Object Wideth:             " << objw << endl;
		cout << "                           " << contours.size() << endl << endl;
		digitalWrite(7, LOW);
		cout << "Sprinkler is on" << endl;
	}

        double end=clock();


	}
	}

	Camera.release();

}
