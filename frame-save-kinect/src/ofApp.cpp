#include "ofApp.h"


void ofApp::setup(){
    ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofBackground(0);
    
    kinect.init(true);
    kinect.open();
    
    bTakeSnapshot = false;
    snapshotCount = 0;
}

void ofApp::update(){
    kinect.update();
    
    if(kinect.isFrameNew()) {
        textureRawDepth.loadData(kinect.getRawDepthPixels());
    
        if(bTakeSnapshot) {
            saveSnapshot();
            bTakeSnapshot = false;
        }
    }
}

void ofApp::draw(){
    kinect.draw(0, 0, 320, 240);
    kinect.drawDepth(320, 0, 320, 240);
    textureRawDepth.draw(640, 0, 320, 240);
    
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
            if(kinect.getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
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

void ofApp::saveSnapshot() {
    ofShortImage raw(kinect.getRawDepthPixels());
    ofImage ir(kinect.getPixels());
    raw.save(ofToString(snapshotCount) + "-depth.png");
    ir.save(ofToString(snapshotCount) + "-ir.png");
    snapshotCount++;
}

void ofApp::exit() {
    kinect.close();
}

void ofApp::keyPressed(int key){
    switch (key) {
        case ' ':
            bTakeSnapshot = true;
            break;
        default:
            break;
    }
}