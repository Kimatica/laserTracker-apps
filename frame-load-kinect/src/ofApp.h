#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "libfreenect_registration.h"

class ofApp : public ofBaseApp{

public:
    void setup();
    void update();
    void draw();
    void keyPressed(int key);

    void drawPointCloud();
    
    float getDistanceAt(int x, int y) const;
    ofVec3f getWorldCoordinateAt(int x, int y) const;
    ofVec3f getWorldCoordinateAt(float cx, float cy, float wz) const;
    
    ofEasyCam easyCam;
    
    ofShortImage raw;
    ofImage ir;
    ofTexture textureRawDepth;
    
    freenect_device* kinectDevice;      ///< kinect device handle
};
