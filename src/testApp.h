#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
        ofxKinect kinect;
    
    // depth boundaries
    int nearThreshold = 202;
	int farThreshold = 191;
    
    // pixel boundaries of sand
    int xLeftBoundary = 240;
    int xRightBoundary = 456;
    int yTopBoundary = 98;
    int yBottomBoundary = 313;
    
    // setting the screen black after <timeout> seconds
    float timeout = 60; //seconds
    int startTimeout; //seconds
    bool isTimeout = false;
    
    ofImage colorImg, gradient;
    
    ofxCvGrayscaleImage grayImage, grayImage_avg, grayThreshNear, grayThreshFar, grayOverall; // grayscale depth image
    // array to take  averages
    double prevPix[39149988];
    ofxCvContourFinder contourFinder;
		
};
