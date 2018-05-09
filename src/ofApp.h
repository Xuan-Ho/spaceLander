#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include  "ofxAssimpModelLoader.h"
#include "box.h"
#include "ray.h"
#include "ParticleEmitter.h"
#include "ParticleSystem.h"
#include "Particle.h"

class TreeNode {
	public:
		Box Box;
		vector<int> points;
		vector<TreeNode> children;
		

};
class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawAxis(ofVec3f);
		void initLightingAndMaterials();
		void savePicture();
		void toggleWireframeMode();
		void togglePointsDisplay();
		void toggleSelectTerrain();
		void setCameraTarget();
		bool  doPointSelection();
		void drawBox(const Box &box);
		Box meshBounds(const ofMesh &);
		void subDivideBox8(const Box &b, vector<Box> & boxList);
		void buildTree(TreeNode & root, int interator);
		bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);
		void drawOct(TreeNode & node, int numLevels, int level);
		void recSearch(Ray ray, TreeNode &node, ofVec3f &selected);

		ofEasyCam cam, cam2, cam3; // Added new cameras
		int camNum = 1; // Value used to cycle through cameras
		//ofPoint modelPos; // Value will keep track of the position of the 3D model may not be necessary

		ofLight light;
		Box boundingBox;
		vector<Box> level1, level2, level3;
	
		bool bAltKeyDown;
		bool bCtrlKeyDown;
		bool bWireframe;
		bool bDisplayPoints;
		bool bPointSelected;
		
		//Octree part
		bool bRoverLoaded;
		bool bTerrainSelected;
		float responseTime;
		ofVec3f selectedPoint;
		ofVec3f intersectPoint;
		ofMesh mesh;
		TreeNode root;
		const float selectionRange = 4.0;

		////lighting & lander part
		ofxAssimpModelLoader moon, lander; //model = lander
		ofLight keyLight, fillLight, rimLight;
		ofPlanePrimitive plane;
		ofMaterial planeMaterial;

		bool bModelLoaded = false;
		bool bPlaneLoaded = false;
		bool bWireFrame = false;

		////emitter part
		ParticleEmitter emitter;
		// some simple sliders to play with parameters
		//
		bool bHide;
		ofxFloatSlider mass;
		ofxFloatSlider gravity;
		ofxFloatSlider damping;
		ofxFloatSlider radius;
		ofxVec3Slider velocity;
		ofxFloatSlider lifespan;
		ofxFloatSlider randomizeLifespan;
		ofxFloatSlider rate;
		ofxPanel gui;
		
};
