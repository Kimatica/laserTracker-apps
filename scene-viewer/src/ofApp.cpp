#include "ofApp.h"

using namespace ofxCv;
using namespace cv;


void ofApp::setup() {
	ofSetVerticalSync(true);
    ofBackground(0);
    ofEnableAntiAliasing();
    
    string dir = "kinect/";
    
    // load images
    // -----------
    for (int i=0; i<5; i++) {
        //ofImage img(dir + "img" + ofToString(i) + ".jpg");
        ofImage img(dir + ofToString(i) + "-ir.png");
        images.push_back(img);
    }
    imageIndex = 0;
    indexChanged = true;
	
    // generate object points
    // ----------------------
	calibration.load(dir + "calibration.yml");
    squareSize = 3.4; // world units will be in cm (we are using chessboard's square size in cm)
    patternSize = cv::Size(10, 7);
	objectPoints = Calibration::createObjectPoints(patternSize, squareSize, CHESSBOARD);
	found = false;
    
    // set chessboard position (in cm)
    // we need to measure the real scene to get this values
    // -------------------------------
    chessboardTranslation.set(-20, 40, 0);
    
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
    // --------------------
    easyCam.setDistance(100);
    easyCam.setNearClip(5);
    easyCam.setFarClip(1000);
    
    bUseEasyCam = true;
    
    // gui
    gui.setup();
    gui.add(bdrawBoardTranslation.set("view translation vec.", false));
    gui.add(bdrawBoard.set("view board", true));
    gui.add(bDrawGround.set("view ground", true));
    gui.setPosition(10,200);
    
    // kinect
    kinect.init(true);
    kinect.open();
}

void ofApp::update() {
    if(indexChanged){
        found = calibration.findBoard(toCv(images[imageIndex]), imagePoints);
        if(found){
            Mat cameraMatrix = calibration.getDistortedIntrinsics().getCameraMatrix();
            Mat rvec, tvec;
            solvePnP(Mat(objectPoints), Mat(imagePoints), cameraMatrix, calibration.getDistCoeffs(), rvec, tvec);
            modelMatrix = makeMatrix(rvec, tvec);
        }
        indexChanged = false;
    }
    
    if(bUseEasyCam) {
        kinect.update();
    }
}

void ofApp::draw() {
	if(found) {
        if(!bUseEasyCam) {
            drawOpencvCamScene();
        }
        else {
            drawFreeCamScene();
            
            // draw image thumbnail
            ofPushMatrix();
            {
                ofScale(0.4, 0.4, 0.4);
                images[imageIndex].draw(0,0);
            }
            ofPopMatrix();
            
            gui.draw();
        }
	}
}

// draw object points using a projection matrix constructed from intrinsics
// the scene is viewed from the calibrated camera perspective
// note that **coordinates use opencv convention** (y and z axys are inverted from opengl)
// and that the camera is also at the origin of the world (hasn't been rotated or translated)
void ofApp::drawOpencvCamScene() {
    ofPushStyle();
    ofSetColor(100); // make image darker
    images[imageIndex].draw(0,0);
    ofPopStyle();
    
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

        ofDrawAxis(squareSize);
    }
    ofPopMatrix();
}

// draw the scene as viewed from a free moving camera
// note that **coordinates use opengl convention** (y and z axys are inverted from opencv)
void ofApp::drawFreeCamScene() {
    ofEnableDepthTest();
    easyCam.begin();
    {
        // draw translation vector
        if(bdrawBoardTranslation) {
            ofDrawArrow(ofVec3f(0,0,0), chessboardTranslation, 0.5);
        }
        
        // draw ground plane
        if (bDrawGround) {
            ofSetColor(50);
            ofPushMatrix();
            {
                ofRotateZ(90);
                ofDrawGridPlane(5, 20); // each square side will be 5cm
            }
            ofPopMatrix();
        }
        
        //
        ofTranslate(chessboardTranslation);
        
        // this converts the scene points from opencv to opengl reference frame
        ofScale(1, -1, -1);
        
        ofPushMatrix();
        {
            ofMultMatrix(modelMatrix.getInverse());
            
            ofDrawAxis(squareSize*2);
            
            // draw camera:
            ofSetColor(150);
            drawCameraFrustum(10);
            
            // draw pointcloud
            ofPushMatrix();
            {
                float scale = 0.1; // mm to cm
                ofScale(scale, scale, scale);
                drawPointCloud();
            }
            ofPopMatrix();
        }
        ofPopMatrix();
        
        // draw chessboard points:
        if(bdrawBoard) {
            ofMesh mesh;
            mesh.setMode(OF_PRIMITIVE_POINTS);
            for(int i = 0; i < objectPoints.size(); i++) {
                mesh.addVertex(toOf(objectPoints[i]));
            }
            glPointSize(3);
            ofSetColor(magentaPrint);
            mesh.drawVertices();
            ofDrawAxis(squareSize);
            
            ofSetColor(100);
            ofDrawRectangle(-squareSize/2, -squareSize/2, 0,
                            calibration.getPatternSize().width * squareSize,
                            calibration.getPatternSize().height * squareSize);
        }
    }
    easyCam.end();
    ofDisableDepthTest();
    
    ofSetColor(255);
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

    mesh.drawVertices();
}


// code from ProCamToolkit
// https://github.com/YCAMInterlab/ProCamToolkit/blob/master/SharedCode/ofxProCamToolkit.h
void ofApp::drawCameraFrustum(float scale) {
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
        case 'f':
            ofToggleFullscreen();
            break;
        default:
            break;
    }
}


void ofApp::exit() {
    kinect.close();
}