#include "ofApp.h"

using namespace ofxCv;
using namespace cv;


void ofApp::setup() {
	ofSetVerticalSync(true);
    ofEnableAntiAliasing();
    ofBackground(0);
    
    string dir = "kinect/";
    
    for (int i=0; i<5; i++) {
        //ofImage img(dir + "img" + ofToString(i) + ".jpg");
        ofImage img(dir + ofToString(i) + "-ir.png");
        images.push_back(img);
    }
    imageIndex = 0;
    indexChanged = true;
	
	calibration.load(dir + "calibration.yml");
    // NOTE: squareSize determines the world units.
    // squareSize=1     world units are squares
    // squareSize=3.7   world units are cm
    // squareSize=0.037 world units are m
    squareSize = 1;
    patternSize = cv::Size(10, 7);
	objectPoints = Calibration::createObjectPoints(patternSize, squareSize, CHESSBOARD);
	found = false;
    
    // match ofCamera params with opencv calibration
    // note that it doesn't quite match the matrix created by Intrinsics::loadProjectionMatrix()
    // --------------------
    const Intrinsics & intrinsics = calibration.getDistortedIntrinsics();
    cv::Mat cameraMatrix = intrinsics.getCameraMatrix();
    float fx = cameraMatrix.at<double>(0, 0);
    float fy = cameraMatrix.at<double>(1, 1);
    cv::Point2d principalPoint = intrinsics.getPrincipalPoint();
    float cx = principalPoint.x;
    float cy = principalPoint.y;
    ofVec2f lensOffset;
    lensOffset.x = (320 - cx) / fx * 2; // TODO: explain this
    lensOffset.y = (cy - 240) / fy * 2;
    
    easyCam.setFov(intrinsics.getFov().y);
    easyCam.setLensOffset(lensOffset);
    easyCam.setDistance(0.01);
    easyCam.setNearClip(0.01);
    easyCam.setFarClip(200);
    // --------------------
}

void ofApp::update() {    
    if(indexChanged){
        found = calibration.findBoard(toCv(images[imageIndex]), imagePoints);
        if(found){
            Mat cameraMatrix = calibration.getDistortedIntrinsics().getCameraMatrix();
            Mat rvec, tvec;
            solvePnP(Mat(objectPoints), Mat(imagePoints), cameraMatrix, calibration.getDistCoeffs(), rvec, tvec);
            modelMatrix = makeMatrix(rvec, tvec);
            
            // output rounded vals
            double toDegrees = 180.0/PI;
            double squareSize = calibration.getSquareSize();
            cout << "Rotation (degrees):" << endl;
            cout << ofToString(rvec.at<double>(0,0) * toDegrees, 2) << " | ";
            cout << ofToString(rvec.at<double>(1,0) * toDegrees, 2) << " | ";
            cout << ofToString(rvec.at<double>(2,0) * toDegrees, 2) << endl;
            cout << "Translation:" << endl;
            cout << ofToString(tvec.at<double>(0,0), 2) << " | ";
            cout << ofToString(tvec.at<double>(1,0), 2) << " | ";
            cout << ofToString(tvec.at<double>(2,0), 2) << endl;
            cout << "-----------------------------" << endl;
            
            // not rounded
            //cout << tvec << endl;
            //cout << tvec << endl;
        }
        indexChanged = false;
    }
}

void ofApp::draw() {
	if(found) {
        if(!bUseEasyCam) {
            drawOpencvCamScene();
        }
        else {
            drawFreeCamScene();
        }
	}
}

// draw object points using a projection matrix constructed from intrinsics
// the scene is viewed from the calibrated camera perspective
// note that **coordinates use opencv convention** (y and z axys are inverted from opengl)
// and that the camera is also at the origin of the world (hasn't been rotated or translated)
void ofApp::drawOpencvCamScene() {
    ofEnableAlphaBlending();
    ofPushStyle();
    ofSetColor(255, 100);
    images[imageIndex].draw(0,0);
    ofPopStyle();
    ofDisableAlphaBlending();
    
    ofPushMatrix();
    {
        calibration.getDistortedIntrinsics().loadProjectionMatrix();

        applyMatrix(modelMatrix);

        ofMesh mesh;
        mesh.setMode(OF_PRIMITIVE_POINTS);
        for(int i = 0; i < objectPoints.size(); i++) {
            mesh.addVertex(toOf(objectPoints[i]));
        }
        glPointSize(3);
        ofSetColor(magentaPrint);
        mesh.drawVertices();

        ofDrawAxis(1);
    }
    ofPopMatrix();
}

// draw the scene as viewed from a free moving camera
// note that **coordinates use opengl convention** (y and z axys are inverted from opencv)
// the camera is assumed to be at the origin
void ofApp::drawFreeCamScene() {
    ofPushStyle();
    ofEnableDepthTest();
    easyCam.begin();
    {
        // this converts the scene points from opencv to opengl reference frame
        // (opencv coords are upside-down and backwards in opengl reference frame)
        ofScale(1, -1, -1);
        
        // draw camera:
        // ------------
        ofSetColor(150);
        ofDrawAxis(2); // opencv's camera origin
        drawCameraFrustum();
        
        // draw chessboard points:
        // -----------------------
        applyMatrix(modelMatrix); // this is the same as: ofMultMatrix(modelMatrix);
        
        ofMesh mesh;
        mesh.setMode(OF_PRIMITIVE_POINTS);
        for(int i = 0; i < objectPoints.size(); i++) {
            mesh.addVertex(toOf(objectPoints[i]));
        }
        glPointSize(3);
        ofSetColor(magentaPrint);
        mesh.drawVertices();
        ofDrawAxis(1); // chessboard origin
        
        // draw chesboard rect
        ofSetColor(50);
        ofDrawRectangle(-squareSize/2, -squareSize/2, 0,
                        calibration.getPatternSize().width * squareSize,
                        calibration.getPatternSize().height * squareSize);
    }
    easyCam.end();
    ofDisableDepthTest();
    ofPopStyle();
}


// code from ProCamToolkit
// https://github.com/YCAMInterlab/ProCamToolkit/blob/master/SharedCode/ofxProCamToolkit.h
void ofApp::drawCameraFrustum() {
    Intrinsics intrinsics = calibration.getDistortedIntrinsics();
    int w = intrinsics.getImageSize().width;
    int h = intrinsics.getImageSize().height;
    
    Mat camInv = intrinsics.getCameraMatrix().inv();
    // homogenous coordinates
    Mat1d hnw = (Mat1d(3,1) << 0, 0, 1);
    Mat1d hne = (Mat1d(3,1) << w, 0, 1);
    Mat1d hsw = (Mat1d(3,1) << 0, h, 1);
    Mat1d hse = (Mat1d(3,1) << w, h, 1);
    // world coordinates
    Mat1d pnw = camInv * hnw;
    Mat1d pne = camInv * hne;
    Mat1d psw = camInv * hsw;
    Mat1d pse = camInv * hse;
    
    float scale = 1;
    
    ofMesh c;
    c.setMode(OF_PRIMITIVE_LINES);
    #define av(x,y,z) addVertex(ofVec3f(x,y,z))
    #define amv(x) addVertex(ofVec3f(x(0),x(1),x(2)))
    c.av(0,0,0);
    c.amv(pnw);
    c.av(0,0,0);
    c.amv(pne);
    c.av(0,0,0);
    c.amv(pse);
    c.av(0,0,0);
    c.amv(psw);
    
    c.amv(psw);
    c.amv(pnw);
    c.amv(pnw);
    c.amv(pne);
    c.amv(pne);
    c.amv(pse);
    c.amv(pse);
    c.amv(psw);
    
    ofPushMatrix();
    {
        ofScale(scale, scale, scale);
        c.draw();
    }
    ofPopMatrix();
}

void ofApp::keyPressed(int key){
    switch (key) {
        case OF_KEY_LEFT:
            imageIndex = (imageIndex == 0) ? images.size()-1 : imageIndex-1;
            indexChanged = true;
            break;
        case OF_KEY_RIGHT:
            imageIndex = (imageIndex == images.size()-1) ? 0 : imageIndex+1;
            indexChanged = true;
            break;
        case ' ':
            bUseEasyCam = !bUseEasyCam;
            break;
        default:
            break;
    }
}
