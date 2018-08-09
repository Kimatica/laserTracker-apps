#include "ofApp.h"


void ofApp::setup(){
    ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofBackground(0);
    
//    kinect.init(true);
//    kinect.open();
//    kinect.
    
    raw.load("../../../common/snapshots/ir-projector-on/0-depth.png");
    ir.load("../../../common/snapshots/ir-projector-on/0-ir.png");
    
    for (int y=0; y<480; y+=2) {
        for (int x=0; x<640; x+=2) {
            cout << raw.getColor(x, y).r << endl;
        }
    }
}

void ofApp::update(){
//    kinect.update();
}

void ofApp::draw(){
    ir.draw(0, 0, 320, 240);
    raw.draw(320, 0, 320, 240);
    
    easyCam.begin();
    {
        drawPointCloud();
    }
    easyCam.end();
}

void ofApp::drawPointCloud() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(getDistanceAt(x, y) > 0) {
                mesh.addColor(ir.getColor(x,y));
                mesh.addVertex(getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    {
        // the projected points are 'upside down' and 'backwards'
        ofScale(1, -1, -1);
        ofTranslate(0, 0, -1000); // center the points a bit
    
        ofEnableDepthTest();
    
        ofDrawAxis(1000);
        mesh.drawVertices();
    
        ofDisableDepthTest();
    }
    ofPopMatrix();
}

//------------------------------------
float ofApp::getDistanceAt(int x, int y)  const{
    return raw.getPixels()[y * 640 + x];
}

ofVec3f ofApp::getWorldCoordinateAt(int x, int y)  const{
    return getWorldCoordinateAt(x, y, getDistanceAt(x, y));
}

ofVec3f ofApp::getWorldCoordinateAt(float cx, float cy, float wz)  const{
    double wx, wy;
    freenect_camera_to_world(kinectDevice, cx, cy, wz, &wx, &wy);
    return ofVec3f(wx, wy, wz);
}
//------------------------------------


void ofApp::keyPressed(int key){
    
}