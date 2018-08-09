#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"

/* 
 Estimate the pose of a cheesboard relative to a camera.
 Needs intrinsics (calibrated camera).
 */
class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
    void keyPressed(int key);
    
    void drawFreeCamScene();
    void drawOpencvCamScene();
    void drawCameraFrustum();
	
    vector<ofImage> images;
    int imageIndex;
    bool indexChanged;
    
	ofxCv::Calibration calibration;
	vector<cv::Point3f> objectPoints;
	vector<cv::Point2f> imagePoints;
	ofMatrix4x4 modelMatrix;
	bool found;
	cv::Size patternSize;
    float squareSize;
    
    ofEasyCam easyCam;
    bool bUseEasyCam;
    
    ofxPanel gui;
};
