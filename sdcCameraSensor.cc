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

std::vector<int> previousMidline;
double estimatedRoadWidth = 0;
double roadWidthSum = 0;
double roadUpdateCounter = 0;


/*Helper methods for Improved CHEVP algorithm*/

/* get the slope of a line */
double getSlope(cv::Vec4i l){
	double vertSlope = 1000000000.00;
	if(l[2]-l[0] == 0){
		return infinityDouble;
	}
	else{
		return ((l[3] - l[1])*1.0)/((l[2]-l[0])*1.0);
	}
}

/* get the distance from a point to a line segment */
double getPointLineDist(cv::Point p1, cv::Vec4i l1){
 double l1Slope = getSlope(l1);
 double l1intercept = l1[1] - (l1[0]*l1Slope);
 return std::abs((l1Slope*(p1.x) + (-1)*(p1.y) + l1intercept))/sqrt(pow(l1Slope,2) +(1));
}

/* test if two lines generated from Hough Transform are actually the same lines */
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

/* calculate the angle between two midlines and store it in degree */
double getAngleDifference(cv::Vec4i l1, cv::Vec4i l2){
	int *vector1 = new int[2];
	int *vector2 = new int[2];
	vector1[0] = l1[0] - l1[2];
	vector1[1] = l1[1] - l1[3];
	vector2[0] = l2[0] - l2[2];
	vector2[1] = l2[1] - l2[3];

	float length1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
	float length2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));
	float product = vector1[0]*vector2[0] + vector1[1]*vector2[1];
	double cosValue = product / (length1 * length2);
	double degree =  acos(cosValue) * 180.0 / CV_PI;
	// add the direction of the angle, left or right
	if (l1[0] - l1[2] >= 0) {
		return degree;
	} else {
		return (-1)*degree;
	}
}

/* convert resizable vector to non-resizable array */
double *convertVectorToArray(vector<double> vec) {
	int size = vec.size();
	double *arr = new double[size];
	for (int i = 0; i < size; i++) {
		arr[i] = vec.at(i);
	}
	return arr;
}

/* calculate line segment with slope and intercept, cut by horizon lines we need to catch the case when the slope is 0 */
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
	return quad;
}

/* get the intercept of a line */
double getIntercept(cv::Vec4i l, double slope) {
	return l[1] - slope*l[0];
}

/*get the index of the closest line */
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
}

void sdcCameraSensor::Init(){
    this->cameraId = this->sensor->GetId();
		previousMidline.push_back(infinityInt);
		previousMidline.push_back(infinityInt);
		previousMidline.push_back(infinityInt);
		previousMidline.push_back(infinityInt);
    this->sensorData = manager::getSensorData(cameraId);
}

void sdcCameraSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
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
    if(!cpu_stop_sign.load(cascade_file_path)) {
			std::cout << "Unable to load cascade classifier xml file!" << std::endl;
    }
}

/* Called by the world update event */
void sdcCameraSensor::OnUpdate() {
	// Pull raw data from camera sensor object as an unsigned character array with 3 channels.
	const unsigned char* img = this->parentSensor->GetImageData(0);
	Mat image = Mat(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0), CV_8UC3, const_cast<unsigned char*>(img));

	//Select Region of Interest (ROI) for lane detection - currently this is the bottom half of the image.
	//set area for ROI as a rectangle
	Rect ROI = cv::Rect(0, image.rows/2, image.cols, image.rows/2);
	Mat imageROI = image(ROI);

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

	// iterate through each of the five sections of the ROI
	// we should use section 2,3 which are indexed at 1,2
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

		if (this->cameraId == 6238 || this->cameraId == 6268) {
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
					vector<double> xVector;
					vector<double> yVector;
					xMap[clusterNumber] = xVector;
					yMap[clusterNumber] = yVector;
				}
				for (int t = curLine[0]; t <= curLine[2]; t++) {
					xMap[clusterNumber].push_back(t);
					yMap[clusterNumber].push_back(t*curSlope + curIntercept);
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

				line(imageROI, Point(l1[0], l1[1] + offset[i]*row/15), Point(l1[2], l1[3] + offset[i]*row/15), Scalar(colors[i-1][0],colors[i-1][1],colors[i-1][2]), 3, CV_AA);
				line(imageROI, Point(l2[0], l2[1] + offset[i]*row/15), Point(l2[2], l2[3] + offset[i]*row/15), Scalar(colors[i-1][0],colors[i-1][1],colors[i-1][2]), 3, CV_AA);
				// find the midline we want
				Vec4i realMidline;
				realMidline[0] = vanishPoint.x;
				realMidline[1] = vanishPoint.y;
				realMidline[2] = midPoint.x;
				realMidline[3] = midPoint.y;

				twoMidlines.push_back(realMidline);

				// estimate the road distance based on pixel
				double l1MidpointX = (l1[0] + l1[2])/2.0;
				double l2MidpointX = (l2[0] + l2[2])/2.0;
				double xDifference = std::abs(l1MidpointX - l2MidpointX);
				Vec4i verticalLine;
				verticalLine[0] = col/2;
				verticalLine[1] = 0;
				verticalLine[2] = col/2;
				verticalLine[3] = 10;
				double verticalDifference = std::abs(getAngleDifference(realMidline, verticalLine));

				if (i == 2 && verticalDifference <= 10) {
					roadWidthSum += xDifference;
					roadUpdateCounter++;
					double roadWidth = roadWidthSum/roadUpdateCounter;
					this->sensorData->setRoadWidth(roadWidth/105);
				}

			}
		}
	}

	if (twoMidlines.size() == 2) {
		double degree = getAngleDifference(twoMidlines.at(0), twoMidlines.at(1));
		// if the angle between two frames' midlines are very large, we don't update the midline
		Vec4i previousMidlineInVector;
		previousMidlineInVector[0] = previousMidline.at(0);
		previousMidlineInVector[1] = previousMidline.at(1);
		previousMidlineInVector[2] = previousMidline.at(2);
		previousMidlineInVector[3] = previousMidline.at(3);

		double degreeBetweenSameMidlines = std::abs(getAngleDifference(twoMidlines.at(0), previousMidlineInVector));
		if (previousMidlineInVector[0] == infinityInt) {
			previousMidline.clear();
			previousMidline.push_back(twoMidlines.at(0)[0]);
			previousMidline.push_back(twoMidlines.at(0)[1]);
			previousMidline.push_back(twoMidlines.at(0)[2]);
			previousMidline.push_back(twoMidlines.at(0)[3]);
		}
		else if (previousMidline[0] != infinityInt && degreeBetweenSameMidlines <= 55) {
			previousMidline.clear();
			previousMidline.push_back(twoMidlines.at(0)[0]);
			previousMidline.push_back(twoMidlines.at(0)[1]);
			previousMidline.push_back(twoMidlines.at(0)[2]);
			previousMidline.push_back(twoMidlines.at(0)[3]);
		}

		this->sensorData->setMidlineAngle(degree);
		// set the angle difference between the vertical line and bottomm midline
		Vec4i verticalLine;
		verticalLine[0] = col/2;
		verticalLine[1] = 0;
		verticalLine[2] = col/2;
		verticalLine[3] = 10;

		double verticalDifference = getAngleDifference(twoMidlines.at(1), verticalLine);
		this->sensorData->setVerticalDifference(verticalDifference);
		// draw two midlines
		line(imageROI, Point(twoMidlines.at(0)[0], twoMidlines.at(0)[1] + offset[1]*row/15), Point(twoMidlines.at(0)[2], twoMidlines.at(0)[3] + offset[1]*row/15), Scalar(colors[4][0],colors[4][1],colors[4][2]), 3, CV_AA);
		line(imageROI, Point(twoMidlines.at(1)[0], twoMidlines.at(1)[1] + offset[2]*row/15), Point(twoMidlines.at(1)[2], twoMidlines.at(1)[3] + offset[2]*row/15), Scalar(colors[2][0],colors[2][1],colors[2][2]), 3, CV_AA);
	}

	else if (twoMidlines.size() == 1 && previousMidline.at(0) != infinityInt){
		Vec4i previousMidlineInVector;
		previousMidlineInVector[0] = previousMidline.at(0);
		previousMidlineInVector[1] = previousMidline.at(1);
		previousMidlineInVector[2] = previousMidline.at(2);
		previousMidlineInVector[3] = previousMidline.at(3);

		double degree = getAngleDifference(previousMidlineInVector, twoMidlines.at(0));
		this->sensorData->setMidlineAngle(degree);

		Vec4i verticalLine;
		verticalLine[0] = col/2;
		verticalLine[1] = 0;
		verticalLine[2] = col/2;
		verticalLine[3] = 10;

		double verticalDifference = getAngleDifference(twoMidlines.at(0), verticalLine);
		this->sensorData->setVerticalDifference(verticalDifference);
		// draw midlines
		line(imageROI, Point(previousMidlineInVector[0], previousMidlineInVector[1] + offset[1]*row/15), Point(previousMidlineInVector[2], previousMidlineInVector[3] + offset[1]*row/15), Scalar(colors[4][0],colors[4][1],colors[4][2]), 3, CV_AA);
		line(imageROI, Point(twoMidlines.at(0)[0], twoMidlines.at(0)[1] + offset[2]*row/15), Point(twoMidlines.at(0)[2], twoMidlines.at(0)[3] + offset[2]*row/15), Scalar(colors[2][0],colors[2][1],colors[2][2]), 3, CV_AA);
	}

	// Display results to GUI
	namedWindow("Camera View", WINDOW_AUTOSIZE);
	if (this->cameraId == 6238 || this->cameraId == 6268) {
		imshow("Camera View", imageROI);
	}
	waitKey(4);
}

/*get the intersection of two lines */
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

/*see if two lines are too close */
bool sdcCameraSensor::isTooClose(cv::Vec4i leftLine, cv::Vec4i rightLine, int i, double row, double col) {
	double threshold = col / 8;
	double dis1 = std::abs(leftLine[0] - rightLine[0]);
	double dis2 = std::abs(leftLine[2] - rightLine[2]);
	double larger = dis1 > dis2? dis1 : dis2;
	if (larger < threshold) {
		return true;
	}
	return false;
}

/*
   the main logic to calculate the new turning angle
*/
 double sdcCameraSensor::getNewTurningAngle(cv::Vec4i midLine) {
	 double result = 0;
	 if(midLine[1] - midLine[3] == 0) {
		 result = 0;
	 }else{
		 double tempVal = (std::abs(midLine[0]-midLine[2]))/(std::abs(midLine[1] - midLine[3]));
		 double atanTemp = atan(tempVal);

		 if(-1.57 > atanTemp || atanTemp > 1.57) {
			 result = 0;
		 }
		 if(getSlope(midLine) < 0){
			 result = -atanTemp;
		 }else
		 {
			 result = atanTemp;
		 }
	 }
	 if(result < 1 && result > -1 ){
		 return 0;
	 }
	 return 20*result;
 }
