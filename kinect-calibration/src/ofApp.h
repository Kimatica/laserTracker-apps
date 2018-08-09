#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinect.h"

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	
//	ofVideoGrabber cam;
    ofxKinect kinect;
    ofImage undistorted;
	ofPixels previous;
	ofPixels diff;
	float diffMean;
	
	float lastTime;
	bool active;
	
	ofxCv::Calibration calibration;
};
