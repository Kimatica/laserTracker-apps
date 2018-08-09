#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxKinect.h"

/*
 We want to find the transformation of the camera relative to the world origin.
 Needs kinect IR sensor intrinsics.
 The chessboard will be placed at a know position in the world reference frame.
 SolvePnp will calculate the transformation to bring the points from the chessboard reference frame to the camera reference frame.
 With that information, we can place the camera in the right position within the scene.
*/

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
    void exit();
    void keyPressed(int key);
    
    void drawFreeCamScene();
    void drawOpencvCamScene();
    void drawCameraFrustum(float scale);
	
    vector<ofImage> images;
    int imageIndex;
    bool indexChanged;
    
	ofxCv::Calibration calibration;
	vector<cv::Point3f> objectPoints;
	vector<cv::Point2f> imagePoints;
	ofMatrix4x4 modelMatrix; // the trnasformation of the chessboard in the camera reference frame
	bool found;
	cv::Size patternSize;
    float squareSize;
    
    ofEasyCam easyCam;
    bool bUseEasyCam;
    
    ofVec3f chessboardTranslation; // the position of the chessboard in the global reference frame
    
    // gui
    ofxPanel gui;
    ofParameter<bool> bdrawBoardTranslation;
    ofParameter<bool> bdrawBoard;
    ofParameter<bool> bDrawGround;
    
    ofxKinect kinect;
    void drawPointCloud();
};
