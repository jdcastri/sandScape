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
    
    int nearThreshold = 202;
	int farThreshold = 191;
    
    int xLeftBoundary = 240;
    int xRightBoundary = 456;
    int yTopBoundary = 98;
    int yBottomBoundary = 313;
    
    int startTimeout; //seconds
    float timeout = 60; //seconds
    bool isTimeout = false;
    string mode = "none";
    
    int playModeCoords [2] = {500,280};
    int diffModeCoords [2] = {500,200};
    double diffPixels[640][640];
	
    int angle;
    int prevFrameCount=0;
    int frameThreshold=20;
    
    ofImage colorImg, diffImg, gradient;
    
    vector<int> contour;
    
    ofxCvGrayscaleImage grayImage, grayImage_avg, grayThreshNear, grayThreshFar, grayOverall; // grayscale depth image
    double prevPix[39149988];
    ofxCvContourFinder contourFinder;
		
};
