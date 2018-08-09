#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
//	ofSetVerticalSync(true);
	
	bSendSerialMessage = false;
	ofBackground(100);
//	ofSetLogLevel(OF_LOG_VERBOSE);
	
//	font.load("DIN.otf", 64);
	
	serial.listDevices();
	vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
	
	// this should be set to whatever com port your serial device is connected to.
	// (ie, COM4 on a pc, /dev/tty.... on linux, /dev/tty... on a mac)
	// arduino users check in arduino app....
    int baud = 115200;
	serial.setup("tty.wchusbserial1410", baud); //open the first device
	//serial.setup("COM4", baud); // windows example
	//serial.setup("/dev/tty.usbserial-A4001JEC", baud); // mac osx example
	//serial.setup("/dev/ttyUSB0", baud); //linux example
	
//	nTimesRead = 0;
//	nBytesRead = 0;
//	readTime = 0;
//	memset(bytesReadString, 0, 4);
    
    bLaserOn = false;
}

//--------------------------------------------------------------
void ofApp::update(){
	
	if (bSendSerialMessage){
        
        // (1) write
        float panPct = float(ofGetMouseX()) / float(ofGetWidth());
        float tiltPct = float(ofGetMouseY()) / float(ofGetHeight());
        
        panPct = ofClamp(panPct, 0.0, 1.0);
        tiltPct = ofClamp(tiltPct, 0.0, 1.0);
		
        // floats to char array
        // https://stackoverflow.com/questions/2988791/converting-float-to-char
        snprintf(bufferPan, sizeof bufferPan, "%f", panPct);
        snprintf(bufferTilt, sizeof bufferTilt, "%f", tiltPct);
        
        // cast to use writeBites
        // https://stackoverflow.com/questions/40820341/c-cast-char-to-unsigned-char
        unsigned char* convert_bufferPan = reinterpret_cast<unsigned char*>(&bufferPan); // (c++ way)
        unsigned char* convert_bufferTilt = reinterpret_cast<unsigned char*>(&bufferTilt); // (c++ way)
        
        unsigned char laserState = bLaserOn ? '1' : '0';
        
        serial.writeByte('<');
        serial.writeBytes(convert_bufferPan, 5);
        serial.writeByte(',');
        serial.writeBytes(convert_bufferTilt, 5);
        serial.writeByte(',');
        serial.writeByte(laserState);
        serial.writeByte('>');
        
		// (2) read
		// now we try to read 3 bytes
		// since we might not get them all the time 30 - but sometimes 0, 36, or something else,
		// we will try to read three bytes, as much as we can
		// otherwise, we may have a "lag" if we don't read fast enough
		// or just read three every time. now, we will be sure to 
		// read as much as we can in groups of 30...
		
		nTimesRead = 0;
		nBytesRead = 0;
		int nRead  = 0;  // a temp variable to keep count per read
		
		unsigned char bytesReturned[30];
		
		memset(bytesReadString, 0, 31);
		memset(bytesReturned, 0, 30);
		
		while( (nRead = serial.readBytes( bytesReturned, 30)) > 0){
			nTimesRead++;	
			nBytesRead = nRead;
		};
		
		memcpy(bytesReadString, bytesReturned, 30);
		
		readTime = ofGetElapsedTimef();
        
        cout << bytesReadString << endl;
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofDrawLine(ofGetWidth()/2 - 10, ofGetHeight()/2, ofGetWidth()/2 + 10, ofGetHeight()/2);
    ofDrawLine(ofGetWidth()/2, ofGetHeight()/2 - 10, ofGetWidth()/2, ofGetHeight()/2 + 10);
    
    ofDrawBitmapStringHighlight("left click and hold and drag to control pan/tilt", 10, 20);
    ofDrawBitmapStringHighlight("click spacebar and hold to switch laser on", 10, 35);
}

//--------------------------------------------------------------
void ofApp::keyPressed  (int key){ 
    if(key == ' ') {
        bLaserOn = true;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){ 
    if(key == ' ') {
        bLaserOn = false;
    }
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){
	
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	bSendSerialMessage = true;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
	bSendSerialMessage = false;
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
	
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
	
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
	
}

