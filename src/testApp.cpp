#include "testApp.h"
#include "ofAppRunner.h"

//--------------------------------------------------------------
void testApp::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
    
    // allocate memory in ofImages
    colorImg.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
    grayImage.allocate(kinect.width, kinect.height);
    
    grayOverall.allocate(kinect.width, kinect.height);
    grayOverall.set(0);
    
    grayImage_avg.allocate(kinect.width, kinect.height);
    grayImage_avg.set(1);
    
    // get current time
    startTimeout = ofGetElapsedTimef();
    
    // load 20x1 pixel image for color gradients
    if(!gradient.loadImage("gradient.png")) {
        ofLog(OF_LOG_ERROR, "Error while loading image");
    }
    
    ofSetFrameRate(60);
    ofSetLogLevel(OF_LOG_VERBOSE);
}

//--------------------------------------------------------------
void testApp::update(){
    ofBackground(0, 0, 0);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        // add effects to smooth signal and reduce noise
        grayImage.blur(7);
        grayImage.dilate();

        
        //for (int i=0; i < nearThreshold- farThreshold; i+=1) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(farThreshold + 1 + 1, true);
            grayThreshFar.threshold(farThreshold + 1);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        
            cvAddWeighted(grayImage.getCvImage(), .9, grayImage_avg.getCvImage(), .1, 0.0, grayImage_avg.getCvImage());
        cvSmooth(grayImage_avg.getCvImage(), grayImage_avg.getCvImage());
        
            contourFinder.findContours(grayImage_avg, 200, (340*240)/3, 10, true);
        //}
        
        
        //cvAnd(grayImage.getCvImage(), grayOverall.getCvImage(), grayOverall.getCvImage(), NULL);
        
        
        
        /*
         * Edit data through pixel manipulation
         */
        
        /*
        unsigned char * pix = grayImage.getPixels();

        // draw image
        int diffThreshold = nearThreshold - farThreshold;
        
        // iterate through pixel data
        for(int w = 0; w < grayImage.getWidth(); w++) {
            for(int h=0; h < grayImage.getHeight(); h++) {
                
                // average previous pixels with current reading
                int index = h * int(grayImage.getWidth()) + w;
                double currentDepth = pix[index];
                double prevDepth = prevPix[index];
                prevPix[index] = prevDepth* .90 + currentDepth * .1;
                double  depth = prevPix[index];
                
                // boolean operations if inside sandbox boundaries and depth boundaries
                bool isInBoundary = w<xRightBoundary && w>xLeftBoundary && h<yBottomBoundary && h>yTopBoundary;
                bool isInDepthBoundary = currentDepth < nearThreshold && currentDepth > farThreshold;
                
                // set Timeout zone
                int lowerBoundary = 210;
                int upperBoundary = 400;
                bool isInActiveZone = currentDepth<upperBoundary && currentDepth>lowerBoundary;
                
                if (isInBoundary && isInActiveZone) {
                    isTimeout = false;
                    startTimeout = ofGetElapsedTimef();
                }
                if (ofGetElapsedTimef() == startTimeout + timeout) {
                    isTimeout = true;
                }
                
                // set pixels of colorImg based on depth ratio
                if( isInDepthBoundary && isInBoundary && !isTimeout) {
                    double diffToThreshold = depth - farThreshold;
                    double ratioInDepth = diffToThreshold/diffThreshold;
                        
                    ofColor color = gradient.getColor(floor(ratioInDepth * 20), 0);
                    colorImg.setColor(w,h, color);
                }

            }
        }
        
        colorImg.update();
         */
        
		// update the cv images
		grayImage.flagImageChanged();
    }
}

//--------------------------------------------------------------
void testApp::draw() {
    
    // enlarge image by multiplier
    double multiplier = 2.0;
    /*
    double width = colorImg.width * multiplier;
    double height = colorImg.height * multiplier;
    
    ofPushMatrix();
    ofTranslate(width/2, height/2);
    
    // 2 degrees to match the projector angle
    ofRotate(180 + 2);
    ofPushMatrix();
    int offsetX = 100;
    int offsetY = 180;
    ofTranslate(-width/2 + offsetX, -height/2 + offsetY);
    
    //Additional translation to fit projection
    ofTranslate(-63, -39);
    
        colorImg.draw(0, 0, width, height);
    ofPopMatrix();
    ofPopMatrix();
    
    */
    //grayImage_avg.draw(10,10);
    ofSetColor(255, 0, 0);

    
    for (int i = 0; i < contourFinder.nBlobs; i++){
        contourFinder.blobs[i].draw(10,10);
    }
     
}

//--------------------------------------------------------------
    void testApp::exit() {
        kinect.setCameraTiltAngle(0); // zero the tilt on exit
        kinect.close();
    }
//--------------------------------------------------------------
void testApp::keyPressed(int key){
    if (key == 'f'){
        ofToggleFullscreen();
    }
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
    cout << x << ", " << y << endl;
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}
