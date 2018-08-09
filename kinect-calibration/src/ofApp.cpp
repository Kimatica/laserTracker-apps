#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

const float diffThreshold = 2.5; // maximum amount of movement
const float timeThreshold = 3; // minimum time between snapshots
const int startCleaning = 10; // start cleaning outliers after this many samples

void ofApp::setup() {
	ofSetVerticalSync(true);
    
    kinect.init(true);
    kinect.open();
	
	FileStorage settings(ofToDataPath("settings.yml"), FileStorage::READ);
	if(settings.isOpened()) {
		int xCount = settings["xCount"], yCount = settings["yCount"];
		calibration.setPatternSize(xCount, yCount);
		float squareSize = settings["squareSize"];
		calibration.setSquareSize(squareSize);
		CalibrationPattern patternType;
		switch(settings["patternType"]) {
			case 0: patternType = CHESSBOARD; break;
			case 1: patternType = CIRCLES_GRID; break;
			case 2: patternType = ASYMMETRIC_CIRCLES_GRID; break;
		}
		calibration.setPatternType(patternType);
	}
	
	imitate(undistorted, kinect.getPixels());
	imitate(previous, kinect.getPixels());
	imitate(diff, kinect.getPixels());
	
	lastTime = 0;
	
	active = true;
}

void ofApp::update() {
	kinect.update();
	if(kinect.isFrameNew()) {
		Mat camMat = toCv(kinect.getPixels());
		Mat prevMat = toCv(previous);
		Mat diffMat = toCv(diff);
		
		absdiff(prevMat, camMat, diffMat);	
		camMat.copyTo(prevMat);
		
		diffMean = mean(Mat(mean(diffMat)))[0];
		
		float curTime = ofGetElapsedTimef();
		if(active && curTime - lastTime > timeThreshold && diffMean < diffThreshold) {
			if(calibration.add(camMat)) {
				cout << "re-calibrating" << endl;
				calibration.calibrate();
				if(calibration.size() > startCleaning) {
					calibration.clean();
				}
				calibration.save("calibration-kinect.yml");
				lastTime = curTime;
			}
		}
		
		if(calibration.size() > 0) {
			calibration.undistort(toCv(kinect.getPixels()), toCv(undistorted));
			undistorted.update();
		}
	}
}

void ofApp::draw() {
	ofSetColor(255);
	kinect.draw(0, 0);
	undistorted.draw(640, 0);
	
	stringstream intrinsics;
	intrinsics << "fov: " << toOf(calibration.getDistortedIntrinsics().getFov()) << " distCoeffs: " << calibration.getDistCoeffs();
    string oneLine = intrinsics.str();
    ofStringReplace(oneLine, "\n", "");
	ofDrawBitmapStringHighlight(oneLine, 10, 20, yellowPrint, ofColor(0));
	ofDrawBitmapStringHighlight("movement: " + ofToString(diffMean), 10, 40, cyanPrint);
	ofDrawBitmapStringHighlight("reproj error: " + ofToString(calibration.getReprojectionError()) + " from " + ofToString(calibration.size()), 10, 60, magentaPrint);
	for(int i = 0; i < calibration.size(); i++) {
		ofDrawBitmapStringHighlight(ofToString(i) + ": " + ofToString(calibration.getReprojectionError(i)), 10, 80 + 16 * i, magentaPrint);
	}
}

void ofApp::keyPressed(int key) {
	if(key == ' ') {
		active = !active;
	}
}
