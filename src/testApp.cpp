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
    
    colorImg.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
    diffImg.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
    grayImage.allocate(kinect.width, kinect.height);
    grayOverall.allocate(kinect.width, kinect.height);
    grayOverall.set(254);
    
    grayImage_avg.allocate(kinect.width, kinect.height);
    grayImage_avg.set(1);
    
    startTimeout = ofGetElapsedTimef();
    if(!gradient.loadImage("gradient.png")) {
        ofLog(OF_LOG_ERROR, "Error while loading image");
    }
    
    ofSetFrameRate(60);
    ofSetLogLevel(OF_LOG_VERBOSE);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
}

//--------------------------------------------------------------
void testApp::update(){
    ofBackground(0, 0, 0);
    
    bool getDiffPixels;
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        // add effects to smooth signal and reduce noise
        grayImage.blur(3);
        grayImage.dilate();

        
        //for (int threshold=farThreshold; threshold < nearThreshold; threshold+=1) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(farThreshold + 2, true);
            grayThreshFar.threshold(farThreshold + 1);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        
            contourFinder.findContours(grayImage, 20, (340*240)/3, 10, true);
        
            //cvAnd(grayImage.getCvImage(), grayOverall.getCvImage(), grayOverall.getCvImage(), NULL);
       // }
        
        //cvAddWeighted(grayImage.getCvImage(), .8, grayImage_avg.getCvImage(), .2, 0.0, grayImage_avg.getCvImage());
        
        /*
        
        unsigned char * pix = grayImage.getPixels();
        
        if (ofGetFrameNum() == 1) {
            int size = 39149988;
            for (int i=0; i<size; i++) {
                prevPix[i] = 100;
            }
        }
        
        // find mode
        double playModeHeight = pix[playModeCoords[1] * int(grayImage.getWidth()) + playModeCoords[0]];
        double diffModeHeight = pix[diffModeCoords[1] * int(grayImage.getWidth()) + diffModeCoords[0]];
       
        if (playModeHeight >= 200 && mode!="play") {
            if (ofGetFrameNum() == prevFrameCount + frameThreshold) {
                mode = "play";
            } else if (ofGetFrameNum() > prevFrameCount + frameThreshold){
                prevFrameCount = ofGetFrameNum();
            }
        }
        
        else if (diffModeHeight >= 200 && mode!="diff") {
            if (ofGetFrameNum() == prevFrameCount + frameThreshold) {
                //mode = "diff";
                getDiffPixels = true;
            } else if (ofGetFrameNum() > prevFrameCount + frameThreshold){
                prevFrameCount = ofGetFrameNum();
            }
        }

        // draw image
        int diffThreshold = nearThreshold - farThreshold;
        for(int w = 0; w < grayImage.getWidth(); w++) {
            for(int h=0; h < grayImage.getHeight(); h++) {
                // average previous pixels with current reading
                int index = h * int(grayImage.getWidth()) + w;
                double currentDepth = pix[index];
                double prevDepth = prevPix[index];
                prevPix[index] = prevDepth* .8 + currentDepth * .2;
                double  depth = prevPix[index];
                
                if (getDiffPixels) {
                    diffPixels[w][h] = depth;
                }
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
                if( isInDepthBoundary && isInBoundary && !isTimeout) {
                    
                    // play mode
                    if (mode == "play") {
                        double diffToThreshold = depth - farThreshold;
                        double ratioInDepth = diffToThreshold/diffThreshold;
                        
                        ofColor color = gradient.getColor(floor(ratioInDepth * 20), 0);
                        colorImg.setColor(w,h, color);
                    }
                    else if (mode == "diff" && diffPixels[0]>=0){
                        double diffPixel = diffPixels[w][h];
                        int saturation = 50+(diffPixel - depth) /4. * 255.;
                        ofColor gradient = ofColor::white;
                        if (saturation>0) {
                            gradient = ofColor::fromHsb(255*2/3, saturation, 255);
                        }
                        else if (saturation<0) {
                            gradient = ofColor::fromHsb(0,-saturation,255);
                        }
                        diffImg.setColor(w,h, gradient);
                    }
                    
                } else {
                    colorImg.setColor(w,h, ofColor::black);
                    diffImg.setColor(w,h, ofColor::black);
                }	
            }
        }
        getDiffPixels = false;
        
        colorImg.update();
        diffImg.update();
         
        */
		// update the cv images
		grayImage.flagImageChanged();
    }
}

//--------------------------------------------------------------
void testApp::draw() {
    
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
    

    if (mode=="diff") {
        diffImg.draw(0,0, width, height);
    } else {
        colorImg.draw(0, 0, width, height);
    }
    ofPopMatrix();
    ofPopMatrix();
    */
    //grayOverall.draw(10,10);
    ofSetColor(255, 0, 0);
    grayImage.draw(10,10);
    
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
