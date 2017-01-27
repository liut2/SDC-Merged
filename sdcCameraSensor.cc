/*
 * This class registers and updates the front camera sensor.
 *
 * This class also handles video processing and lane finding based
 * upon video inputs to handle lane finding and tracking using
 * methods found in the paper "A lane-curve detection based on LCF"
 */

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <numeric>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "sdcCameraSensor.hh"
#include "fadiff.h"
#include <math.h>
#include "linear.h"
#include <iomanip>
#include <map>
#include <limits>

using namespace fadbad;
using namespace gazebo;
using namespace cv;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(sdcCameraSensor)

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::MultiCameraSensorPtr parentSensor;

// Cascade Classifier information using CPU
CascadeClassifier cpu_stop_sign;
String cascade_file_path = "OpenCV/haarcascade_stop.xml";

// some constants
double MidPointHeight[5] = {0.5,2.0,4.5,8.0,12.5};
int sdcCameraSensor::cameraCnt = 0;
const int infinityInt = std::numeric_limits<int>::max();
//FADBAD-wrapped forward differentiation for gradient calculations
F<double> delG(const F<double>& x, const F<double>& y) {
	F<double> dG = sqrt(pow(x,2)+pow(y,2));
	return dG;
}

/*Helper methods for Improved CHEVP algorithm*/

//get the slope of a line
double getSlope(cv::Vec4i l){
	double vertSlope = 1000000000.00;
	if(l[2]-l[0] == 0){
		return infinityDouble;
	}
	else{
		return ((l[3] - l[1])*1.0)/((l[2]-l[0])*1.0);
	}
}

// get the distance from a point to a line segment
double getPointLineDist(cv::Point p1, cv::Vec4i l1){
 double l1Slope = getSlope(l1);
 double l1intercept = l1[1] - (l1[0]*l1Slope);
 return std::abs((l1Slope*(p1.x) + (-1)*(p1.y) + l1intercept))/sqrt(pow(l1Slope,2) +(1));
}

// test if two lines generated from Hough Transform are actually the same lines
bool isEqual(const cv::Vec4i& _l1, const cv::Vec4i& _l2) {
	 Vec4i l1(_l1), l2(_l2);

	 float length1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
	 float length2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));

	 float product = (l1[2] - l1[0])*(l2[2] - l2[0]) + (l1[3] - l1[1])*(l2[3] - l2[1]);

	 // test if the slope differs too much

	 if (fabs(product / (length1 * length2)) < cos(CV_PI / 30))
			 return false;

	 float mx1 = (l1[0] + l1[2]) * 0.5f;
	 float mx2 = (l2[0] + l2[2]) * 0.5f;

	 float my1 = (l1[1] + l1[3]) * 0.5f;
	 float my2 = (l2[1] + l2[3]) * 0.5f;
	 float dist = sqrtf((mx1 - mx2)*(mx1 - mx2) + (my1 - my2)*(my1 - my2));

	 // test if two lines are too far away from each other
	 if (dist > std::max(length1, length2) * 0.5f)
			 return false;

	 return true;
}

// calculate the angle between two midlines and store it in degree
double getAngleDifference(cv::Vec4i l1, cv::Vec4i l2){
	float length1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
	float length2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));
	float length3 = sqrtf((l2[2] - l1[0])*(l2[2] - l1[0]) + (l2[3] - l1[1])*(l2[3] - l1[1]);

	float product = (l1[2] - l1[0])*(l2[2] - l2[0]) + (l1[3] - l1[1])*(l2[3] - l2[1]);

	return fabs(product / (length1 * length2)) * 180/ CV_PI;
}

// convert resizable vector to non-resizable array
double *convertVectorToArray(vector<double> vec) {
	int size = vec.size();
	double *arr = new double[size];
	for (int i = 0; i < size; i++) {
		arr[i] = vec.at(i);
	}
	return arr;
}

// calculate line segment with slope and intercept, cut by horizon lines
// we need to catch the case when the slope is 0
int *getLineSegment(double slope, double intercept, double upperY, double lowerY) {

	int *quad = new int[4];
	if (slope == 0) {
		quad[0] = infinityInt;
		quad[1] = infinityInt;
		quad[2] = infinityInt;
		quad[3] = infinityInt;
	} else {
		quad[0] = (int)((upperY - intercept)/(slope*1.0) + 0.5);
		quad[1] = (int)(upperY + 0.5);
		quad[2] = (int)((lowerY - intercept)/(slope*1.0));
		quad[3] = (int)(lowerY);
	}
	//printf("The new line segment is %d %d %d %d\n", segment[0], segment[1], segment[2], segment[3]);
	return quad;
}

double getIntercept(cv::Vec4i l, double slope) {
	return l[1] - slope*l[0];
}

int getIndexOfClosestLine(std::vector<Vec4i> lines, Point sectionMidPoint) {
	double minDistance = infinityDouble;
	int minIndex = infinityInt;

	for(int j = 0; j < lines.size(); j++) {
			Vec4i l = lines[j];
			double xOfMidPoint = (l[0] + l[2])/2.0;
			double curDistance = abs(xOfMidPoint - sectionMidPoint.x);

			if (curDistance < minDistance) {
				minDistance = curDistance;
				minIndex = j;
			}
 		}
		return minIndex;
}


sdcCameraSensor::sdcCameraSensor(){
    this->cameraCnt ++;
    this->cameraId = this->cameraCnt;
}

void sdcCameraSensor::Init(){
    this->cameraId = this->sensor->GetId();
    //printf("this camerasId: %i \n", this->cameraId);
    this->sensorData = manager::getSensorData(cameraId);
}

void sdcCameraSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){


    //printf("sdcCamera Steer Mag: %f\n",this->sensorData.GetNewSteeringMagnitude());
   // printf("sdcCamera sensorId: %i\n",this->sensorData->sensorId);
    //sdcSensorData();
    //manager::getSensorData(1);
		//std::cout << "load the camera sensor" << std::endl;
		// Get the parent sensor.
    this->sensor = _sensor;
    this->pose = this->sensor->Pose();
    this->parentSensor = boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);

		// Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "Couldn't find a camera\n";
        return;
    }

		// Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&sdcCameraSensor::OnUpdate, this));

		// Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
		//std::cout << this->parentSensor->GetNoise() << std::endl;
    if(!cpu_stop_sign.load(cascade_file_path)) {
			std::cout << "Unable to load cascade classifier xml file!" << std::endl;
    }
   // this->link = this->parentSensor->GetLink(_sdf->Get<std::string>("link"));





}

// Called by the world update event
void sdcCameraSensor::OnUpdate() {
	//std::cout << "onupdate camera sensor" << std::endl;
	// Pull raw data from camera sensor object as an unsigned character array with 3 channels.
	const unsigned char* img = this->parentSensor->GetImageData(0);
	Mat image = Mat(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0), CV_8UC3, const_cast<unsigned char*>(img));

	//Select Region of Interest (ROI) for lane detection - currently this is the bottom half of the image.
	//set area for ROI as a rectangle
	Rect ROI = cv::Rect(0, image.rows/2, image.cols, image.rows/2);
	Mat imageROI = image(ROI);
	//blur(imageROI, imageROI, Size(3,3));

	// Canny algorithm for edge dectection
	//Mat contours, contours_thresh;
	//Canny(imageROI,contours,50,150);
	/*
	threshold(contours,contours_thresh,127,255, THRESH_BINARY);


	Mat imageGray;
	Mat imageInv = Mat(imageROI.rows, imageROI.cols, CV_8UC1, Scalar(0));
	cvtColor(imageROI,imageGray,CV_BGR2GRAY);
	adaptiveThreshold(imageGray,imageInv, 255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,201, -20 );*/

	// BEGIN STRAIGHT LANE DETECTION
	double col = imageROI.cols;
	double row = imageROI.rows;
	Rect section1 = cv::Rect(0, 0, col, (1.0/15)*row);
	Rect section2 = cv::Rect(0, row/15.0, col, (2.0/15)*row);
	Rect section3 = cv::Rect(0, (3.0/15)*row, col, (3.0/15)*row);
	Rect section4 = cv::Rect(0, (6.0/15)*row, col, (4.0/15)*row);
	Rect section5 = cv::Rect(0, (10.0/15)*row, col, (5.0/15)*row);

	std::vector<Rect> sections;
	sections.push_back(section1);
	sections.push_back(section2);
	sections.push_back(section3);
	sections.push_back(section4);
	sections.push_back(section5);


	int colors[5][3] = {{255, 0, 0}, {255, 255, 0}, {0, 255, 0}, {0, 0, 255}, {255, 0, 255}};
	double offset[5] = {0, 1.0, 3.0, 6.0, 10.0};
	double horizons[5] = {1.0, 3.0, 6.0, 10.0, 15.0};

	/*
  Mat sectionImage = imageROI(section4);
	//Mat contours;
	blur(sectionImage, sectionImage, Size(3,3));
	Canny(sectionImage,sectionImage,50,150, 3);
	vector<Vec4i> lines;
 	HoughLinesP(sectionImage, lines, 1, CV_PI/180, 50, 50, 10 );
 	for( size_t i = 0; i < lines.size(); i++ )
 	{
	 	Vec4i l = lines[i];
	 	line( imageROI, Point(l[0], l[1] + offset[3]*row/15), Point(l[2], l[3] + offset[3]*row/15), Scalar(0,0,255), 3, CV_AA);
 	}
	*/
	/*
	Point a1, a2, b1, b2, c1, c2, d1, d2;
	a1.x = b1.x = c1.x = d1.x = 0;
	a2.x = b2.x = c2.x = d2.x = col;
	a1.y = a2.y = row/15.0;
	b1.y = b2.y = (3.0/15)*row;
	c1.y = c2.y = (6.0/15)*row;
	d1.y = d2.y = (10.0/15)*row;

	line(imageROI, a1, a2, Scalar(255, 0, 0), 3, CV_AA);
	line(imageROI, b1, b2, Scalar(255, 0, 0), 3, CV_AA);
	line(imageROI, c1, c2, Scalar(255, 0, 0), 3, CV_AA);
	line(imageROI, d1, d2, Scalar(255, 0, 0), 3, CV_AA);*/

	Vec4i tempPreviousLeftLine;
	tempPreviousLeftLine[0] = 0;
	tempPreviousLeftLine[1] = 0;
	tempPreviousLeftLine[2] = 0;
	tempPreviousLeftLine[3] = row;
	Vec4i previousLeftLine = tempPreviousLeftLine;

	Vec4i tempPreviousRightLine;
	tempPreviousRightLine[0] = col;
	tempPreviousRightLine[1] = 0;
	tempPreviousRightLine[2] = col;
	tempPreviousRightLine[3] = row;
	Vec4i previousRightLine = tempPreviousRightLine;

	// iterate through each of the five sections of the ROI
	std::vector<Vec4i> twoMidlines;
	for(size_t i = 1; i < 3; i++) {
		Rect section;
		section = sections[i];
		// get the image of the current section
		Mat sectionImage = imageROI(section);
		// apply canny edge detection with min and max threshhold, 50 and 150
		Canny(sectionImage,sectionImage,50,150, 3);
		// apply hough transform and the result is stored in lines vector, 30 ,30, the larger the number of intersections,
		// the longer the line is, intuitively
		vector<Vec4i> lines;
 		HoughLinesP(sectionImage, lines, 1, CV_PI/180, 15, 15, 10);
		//std::cout << "number of lines " << lines.size() << "     ***********"<<std::endl;
		//Vec4i leftLine, rightLine;
		//double leftMinDistance, rightMinDistance;
		//these two points are the id-height points of each section - for use in
		//determining the line we will use for the vanishing point
		//Point leftMidPoint = Point(0,MidPointHeight[i]*row/15);
		//Point rightMidPoint = Point(col,MidPointHeight[i]*row/15);

		// iterate through each of the lines in current section
		/*
 		for(size_t j = 0; j < lines.size(); j++)
 		{
			Vec4i l = lines[j];
			// calculate the slope of current line
			double tempLeftDistance = getPointLineDist(leftMidPoint,l);
			double tempRightDistance = getPointLineDist(rightMidPoint,l);
			if(j == 0){
				leftMinDistance = tempLeftDistance;
				rightMinDistance = tempRightDistance;
				leftLine = l;
				rightLine = l;
			}else{
				if(tempLeftDistance < leftMinDistance){
					leftMinDistance = tempLeftDistance;
					leftLine = l;
				}
				if (tempRightDistance < rightMinDistance) {
					rightMinDistance = tempRightDistance;
					rightLine = l;
				}
			}
			// draw all the lines from Hough transform in current section
	 		line(imageROI, Point(l[0], l[1] + offset[i]*row/15), Point(l[2], l[3] + offset[i]*row/15), Scalar(colors[j][0],colors[j][1],colors[j][2]), 3, CV_AA);
 		}*/

		// edge case checking: case1: 0 or 1 line detected; case2: two lines exsist on the same side
		/*
		if ((lines.size() == 0 || lines.size() == 1 || isTooClose(leftLine, rightLine, i, row, col)) && (i != 0)) {
			if (previousLeftLine != tempPreviousLeftLine) {
				leftLine = extendLine(previousLeftLine, createLine(0, horizons[i-1]*row/15, col, horizons[i-1]*row/15),
				createLine(0, horizons[i]*row/15, col, horizons[i]*row/15));
			}
			if (previousRightLine != tempPreviousRightLine) {
				rightLine = extendLine(previousRightLine, createLine(0, horizons[i-1]*row/15, col, horizons[i-1]*row/15),
				createLine(0, horizons[i]*row/15, col, horizons[i]*row/15));
			}
		}

		previousLeftLine = leftLine;
		previousRightLine = rightLine;
		// construct the horizontal line at the bottom of each section
		Vec4i horizon;
		horizon[0] = 0;
		horizon[1] = horizons[i]*row/15;
		horizon[2] = col;
		horizon[3] = horizons[i]*row/15;
		// get intersections with section horizons
		Point leftEnd = getIntersectionPoint(leftLine, horizon);
		Point rightEnd = getIntersectionPoint(rightLine, horizon);
		// get the mid point on horizon
		Point midPoint = Point((leftEnd.x + rightEnd.x)/2, leftEnd.y);
		Point vanishPoint = getIntersectionPoint(leftLine, rightLine);
		*/
		// we should use section 2,3,4, which are indexed at 1,2,3
		if (true) {
			//line(imageROI, Point(vanishPoint.x, vanishPoint.y), Point(midPoint.x, midPoint.y + offset[i]*row/15), Scalar(colors[i][0],colors[i][1],colors[i][2]), 3, CV_AA);
			// update the turn angle
			//double newAngle = getNewTurningAngle(createLine(vanishPoint.x, vanishPoint.y, midPoint.x, midPoint.y + offset[i]*row/15));
			//this->sensorData->UpdateSteeringMagnitude(newAngle);
            //this->sensorData.sensorId++;
           // printf("sensorData steering mag: %f\n", this->sensorData.GetNewSteeringMagnitude());
            //printf("GET mag sensorID: %i\n", this->sensorData.sensorId);
           // printf("CamData newAngle : %f carId: %i\n", newAngle, cameraId);

			// START OF THE NEW CHEVP algorithm
			// apply partition fuction to clusterize lines of the same one
			std::vector<int> labels;
			int numberOfClusters = cv::partition(lines, labels, isEqual);
			/* merge close lines detected by partition function */
			// first pass to store close lines into a map
			map<int, vector<double>> xMap;
			map<int, vector<double>> yMap;
			for(size_t j = 0; j < lines.size(); j++)
			{
				int clusterNumber = labels[j];
				Vec4i curLine = lines[j];
				double curSlope = getSlope(curLine);
				double curIntercept = getIntercept(curLine, curSlope);

				if (!xMap.count(clusterNumber)) {
					//printf("Enter the map!\n");
					vector<double> xVector;
					vector<double> yVector;
					xMap[clusterNumber] = xVector;
					yMap[clusterNumber] = yVector;
				}
				for (int t = curLine[0]; t <= curLine[2]; t++) {
					xMap[clusterNumber].push_back(t);
					yMap[clusterNumber].push_back(t*curSlope + curIntercept);
					//xMap[clusterNumber].push_back(curLine[2]);
					//yMap[clusterNumber].push_back(curLine[3]);
				}
			}
			// second pass to merge the lines by clusterNumber and store them into mergeLines vector
			std::vector<Vec4i> mergedLines;
			for (size_t j = 0; j < numberOfClusters; j++) {
				Maths::Regression::Linear A(xMap[j].size(), convertVectorToArray(xMap[j]), convertVectorToArray(yMap[j]));
				double slope = A.getSlope();
				double intercept = A.getIntercept();

				Vec4i segment;
				int *segmentArr = getLineSegment(slope, intercept, 0, horizons[i]*row/15 - horizons[i-1]*row/15);
				segment[0] = segmentArr[0];
				segment[1] = segmentArr[1];
				segment[2] = segmentArr[2];
				segment[3] = segmentArr[3];

				// check if the line is horizontal or near horizontal. if so, filter them out
				if (segment[0] != infinityInt && abs(getSlope(segment)) > 0.1) {
					mergedLines.push_back(segment);
					//line(imageROI, Point(segment[0], segment[1] + offset[i]*row/15), Point(segment[2], segment[3] + offset[i]*row/15), Scalar(colors[i-1][0],colors[i-1][1],colors[i-1][2]), 3, CV_AA);
				}
			}
			// find the two closest lines to the center of the section, selected as the left and right lane boundaries
			std::vector<Vec4i> leftBoundaries;
			std::vector<Vec4i> rightBoundaries;
			// first, split the merged lines into two groups, those appear to the left of the center, and those appear to the right of the center
			for (int j = 0; j < mergedLines.size(); j++) {
				Vec4i curLine = mergedLines.at(j);
				double xOfMidPoint = (curLine[0] + curLine[2])/2.0;
				double xOfSectionCenter = col/2.0;
				if (xOfMidPoint <= xOfSectionCenter) {
					leftBoundaries.push_back(curLine);
				} else {
					rightBoundaries.push_back(curLine);
				}
			}
			// next, find the closest line on the left and on the right
			int xOfSectionCenter = col/2;
			int yOfSectionCenter = 0;
			Point sectionMidPoint = Point(xOfSectionCenter, yOfSectionCenter);
			int leftBoundaryIndex = getIndexOfClosestLine(leftBoundaries, sectionMidPoint);
			int rightBoundaryIndex = getIndexOfClosestLine(rightBoundaries, sectionMidPoint);
			if (leftBoundaryIndex != infinityInt && rightBoundaryIndex != infinityInt) {
				Vec4i l1 = leftBoundaries[leftBoundaryIndex];
				Vec4i l2 = rightBoundaries[rightBoundaryIndex];

				Point vanishPoint = getIntersectionPoint(l1, l2);
				Point midPoint = Point((l1[2] + l2[2])/2, l1[3]);

				line(imageROI, Point(vanishPoint.x, vanishPoint.y + offset[i]*row/15), Point(midPoint.x, midPoint.y + offset[i]*row/15), Scalar(colors[i][0],colors[i][1],colors[i][2]), 3, CV_AA);
				line(imageROI, Point(l1[0], l1[1] + offset[i]*row/15), Point(l1[2], l1[3] + offset[i]*row/15), Scalar(colors[i-1][0],colors[i-1][1],colors[i-1][2]), 3, CV_AA);
				line(imageROI, Point(l2[0], l2[1] + offset[i]*row/15), Point(l2[2], l2[3] + offset[i]*row/15), Scalar(colors[i-1][0],colors[i-1][1],colors[i-1][2]), 3, CV_AA);
				// find the midline we want
				Vec4i realMidline;
				realMidline[0] = vanishPoint.x;
				realMidline[1] = vanishPoint.y;
				realMidline[2] = midPoint.x;
				realMidline[3] = midPoint.y;

				twoMidlines.push_back(realMidline);
			}
		}
	}

	if (twoMidlines.size() == 2) {
		double degree = getAngleDifference(twoMidlines.at(0), twoMidlines.at(1));
		this->sensorData->setMidlineAngle(degree);
	}

	// assume we have the three midlines from section 2,3,4, we need to change the accelaration and direction based the angle
	// here we will try to use the midlines from section 2 and 3 first
	//sdcCar::Brake();
	//printf("The speed of the car is %f\n", sdcCar::getSpeed());
	// Display results to GUI
	namedWindow("Camera View", WINDOW_AUTOSIZE);
	imshow("Camera View", imageROI);
	waitKey(4);
}

Point sdcCameraSensor::getIntersectionPoint(cv::Vec4i l1, cv::Vec4i l2){
	double l1Slope = getSlope(l1);
	double l2Slope = getSlope(l2);
	double l1intercept = l1[1] - (l1[0]*l1Slope);
	double l2intercept = l2[1] - (l2[0]*l2Slope);
	//
	double xVal, yVal;
	if (l1Slope - l2Slope == 0){
		xVal = -1.0;
		yVal = -1.0;
	}
	else{
		xVal = (l2intercept - l1intercept)/(l1Slope - l2Slope);
		yVal = (l1Slope * xVal) + l1intercept;
	}
	return Point(std::round(xVal), std::round(yVal));
}

bool sdcCameraSensor::isTooClose(cv::Vec4i leftLine, cv::Vec4i rightLine, int i, double row, double col) {
	/*
	double leftDis1 = getPointLineDist(Point(0,MidPointHeight[i]*row/15), leftLine);
	double leftDis2 = getPointLineDist(Point(0,MidPointHeight[i]*row/15), rightLine);
	double rightDis1 = getPointLineDist(Point(col,MidPointHeight[i]*row/15), leftLine);
	double rightDis2 = getPointLineDist(Point(col,MidPointHeight[i]*row/15), rightLine);
	double threshold = col / 16;
	if ((leftDis1 - leftDis2 <= threshold) || (rightDis1 - rightDis2 <= threshold)) {
		return true;
	}
	return false;
	*/
	double threshold = col / 8;
	double dis1 = std::abs(leftLine[0] - rightLine[0]);
	double dis2 = std::abs(leftLine[2] - rightLine[2]);
	double larger = dis1 > dis2? dis1 : dis2;
	if (larger < threshold) {
		return true;
	}
	return false;
}

cv::Vec4i sdcCameraSensor::extendLine(cv::Vec4i line, cv::Vec4i topHorizontal, cv::Vec4i bottomHorizontal) {
	Point point1 = getIntersectionPoint(line, topHorizontal);
	Point point2 = getIntersectionPoint(line, bottomHorizontal);
	Vec4i extended = createLine(point1.x, point1.y, point2.x, point2.y);
	return extended;
 }

 cv::Vec4i sdcCameraSensor::createLine(double x1, double y1, double x2, double y2) {
	 Vec4i line;
	 line[0] = x1;
	 line[1] = y1;
	 line[2] = x2;
	 line[3] = y2;
	 return line;
 }

/*
   the main logic to calculate the new turning angle
*/
 double sdcCameraSensor::getNewTurningAngle(cv::Vec4i midLine) {
	 double result = 0;
	 //std::cout << "in newturningangle" << std::endl;
	 if(midLine[1] - midLine[3] == 0) {
		// std::cout << "Vertical line" << std::endl;
		 result = 0;
	 }else{
		// std::cout << "In else" << std::endl;

		 double tempVal = (std::abs(midLine[0]-midLine[2]))/(std::abs(midLine[1] - midLine[3]));
		 double atanTemp = atan(tempVal);
		// std::cout << "arctan: " << std::endl;
		// std::cout << atanTemp << std::endl;
		// std::cout <<" tempval" << std::endl;
		 //std::cout << tempVal << std::endl;
		 if(-1.57 > atanTemp || atanTemp > 1.57) {
			// std::cout << "Bad range of angles" << std::endl;
			 //math::Vector3 velocity = this->velocity;
	     //return atan2(velocity.y, velocity.x);
			 result = 0;
		 }
		 if(getSlope(midLine) < 0){
			// std::cout << "Turn left!" << std::endl;
			 result = -atanTemp;
		 }else
		 {
			// std::cout << "Turn right!" << std::endl;
			 result = atanTemp;
		 }
	 }
	 if(result < 1 && result > -1 ){
		 return 0;
	 }
	 //return 10*result;
	 return 20*result;
 }
