
//--------------------------------------------------------------
//
//  Kevin M. Smith
//
//  Mars HiRise Project - startup scene
// 
//  This is an openFrameworks 3D scene that includes an EasyCam
//  and example 3D geometry which I have reconstructed from Mars
//  HiRis photographs taken the Mars Reconnaisance Orbiter
//
//  You will use this source file (and include file) as a starting point
//  to implement assignment 5  (Parts I and II)
//
//  Please do not modify any of the keymappings.  I would like 
//  the input interface to be the same for each student's 
//  work.  Please also add your name/date below.

//  Please document/comment all of your work !
//  Have Fun !!
//
//  Student Name:   Justin M. Armijo, Luis Rios, Xuan Ho
//  Date: 10 May 2018


#include "ofApp.h"
#include "Util.h"



//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup(){

	bWireframe = false;
	bDisplayPoints = false;
	bAltKeyDown = false;
	bCtrlKeyDown = false;
	bRoverLoaded = false;
	bTerrainSelected = true;
//	ofSetWindowShape(1024, 768);

	// Set up Camera 1
	cam.setDistance(10);
	cam.setNearClip(.1);
	cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
	ofSetVerticalSync(true);
	cam.disableMouseInput();
	ofEnableSmoothing();
	//ofEnableDepthTest(); //If enable, it would blackout the slider GUI so i disabled it 
	ofEnableLighting();

	// Set up Camera 2
	cam2.setDistance(50);
	cam2.setNearClip(.1);
	cam2.setFov(80);
	cam2.disableMouseInput();

	// Set up Camera 3
	cam3.setDistance(10);
	cam3.setNearClip(.1);
	cam3.setFov(80);
	cam3.disableMouseInput();

	// Set up Camera 4
	cam4.setDistance(50);
	cam4.setNearClip(.1);
	cam4.setFov(80);
	cam4.disableMouseInput();

	// Setup 3 - Light System
	// 
	keyLight.setup();
	keyLight.enable();
	keyLight.setAreaLight(1, 1);
	keyLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	keyLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	keyLight.setSpecularColor(ofFloatColor(1, 1, 1));

	keyLight.rotate(45, ofVec3f(0, 1, 0));
	keyLight.rotate(-45, ofVec3f(1, 0, 0));
	keyLight.setPosition(5, 5, 5);

	fillLight.setup();
	fillLight.enable();
	fillLight.setSpotlight();
	fillLight.setScale(.05);
	fillLight.setSpotlightCutOff(15);
	fillLight.setAttenuation(2, .001, .001);
	fillLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	fillLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	fillLight.setSpecularColor(ofFloatColor(0, 34, 0));
	fillLight.rotate(-10, ofVec3f(1, 0, 0));
	fillLight.rotate(-45, ofVec3f(0, 1, 0));
	fillLight.setPosition(-5, 5, 5);

	rimLight.setup();
	rimLight.enable();
	rimLight.setSpotlight();
	rimLight.setScale(.05);
	rimLight.setSpotlightCutOff(30);
	rimLight.setAttenuation(.2, .001, .001);
	rimLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	rimLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	rimLight.setSpecularColor(ofFloatColor(1, 1, 1));
	rimLight.rotate(180, ofVec3f(0, 1, 0));
	rimLight.setPosition(0, 5, -7);


	// Particle Slider Setup
	gui.setup();
	gui.add(velocity.setup("Initial Velocity", ofVec3f(0, 20, 0), ofVec3f(0, 0, 0), ofVec3f(100, 100, 100)));
	gui.add(lifespan.setup("Lifespan", 2.0, .1, 10.0));
	gui.add(rate.setup("Rate", 1.0, .5, 60.0));
	gui.add(damping.setup("Damping", .99, .1, 1.0));
	gui.add(gravity.setup("Gravity", .5, .1, 20));
	gui.add(radius.setup("Radius", .1, .01, .3));
	gui.add(mass.setup("Mass", 10, .01, 100));
	gui.add(restitution.setup("Restitution", .6, 0, 1));
	bHide = false;

	//emitter Setup 
	emitter.setOneShot(true);
	emitter.particleColor = ofColor::chocolate;
	emitter.setEmitterType(RadialEmitter);
	emitter.setGroupSize(10);
	emitter.setLifespan(0.4);
	emitter.setVelocity(ofVec3f(550, 300, 100));
	emitter.setParticleRadius(10);
	emitter.setPosition(ofVec3f(ofGetWindowWidth() / 2 - 650, 30, 0));
	emitter.sys->addForce(new TurbulenceForce(ofVec3f(-2, -1, -3), ofVec3f(1, 2, 5)));
	emitter.sys->addForce(new GravityForce(ofVec3f(0, -gravity, 0)));
	emitter.sys->addForce(new ImpulseRadialForce(5000));

	// setup rudimentary lighting 
	//
	initLightingAndMaterials();

	//load moon model  moon-houdini
	lander.loadModel("geo/lander.obj");
	lander.setScaleNormalization(false);
	moon.loadModel("geo/moon-houdini.obj");
	moon.setScaleNormalization(false);


	//This block of code by Justin Armijo
	ship.position.set(0, 20, 0);
	ship.lifespan = 1000000;
	
	ship.radius = .1;
	grav = new GravityForce(ofVec3f(0, -gravity, 0));
	lander.setPosition(ship.position.x, 25, ship.position.z);
	sys.add(ship);
	engine.setRate(20);
	engine.setParticleRadius(.10);
	engine.visible = false;
	sys.addForce(&thruster);
	
	sys.addForce(grav);
	sys.addForce(&impulseForce);
	boundingBox = meshBounds(moon.getMesh(0));

	//  Test Box Subdivide
	//
	//subDivideBox8(boundingBox, level1);
	//subDivideBox8(level1[0], level2);
	//subDivideBox8(level2[0], level3);
	mesh = moon.getMesh(0);
	for (int i = 0; i < mesh.getNumVertices(); i++) {//code from Justin Armijo Octree implementation
		root.points.push_back(i);
	}
	root.Box = boundingBox;
	buildTree(root, -1);
	
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);//gls added by Justin Armijo to fix depth issue
	glDepthFunc(GL_LEQUAL);
	responseTime = 0.0;
}

//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {
	//this block of code by Justin Armijo
	collisionDetect();
	engine.setPosition(sys.particles[0].position);
	emitter.update();
	ofSeedRandom();

	engine.update();
	//First emitter
	lander.setPosition(sys.particles[0].position.x, sys.particles[0].position.y, sys.particles[0].position.z);
	sys.update();
	moon.update();
	emitter.setLifespan(lifespan);
	emitter.setVelocity(velocity);
	emitter.setRate(rate);
	emitter.setParticleRadius(radius);
	emitter.setMass(mass);
	emitter.update();

	//emitter.setPosition(ofVec3f(v.x, v.y, v.z));
	cam2.lookAt(lander.getPosition()); // this should be keeping track of the 3D model
	cam3.setPosition((lander.getPosition() + ofVec3f(0, 0, 10))); // camera 3 should be attached to 3D model, would need to test with moving model
	cam4.setPosition(lander.getPosition()); // camera 4 attached to 3D model, but should be looking at ground
	cam4.lookAt(ofVec3f(cam4.getPosition().x, cam4.getPosition().y - 10, cam4.getPosition().z));
}
//--------------------------------------------------------------
void ofApp::draw(){
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	ofBackgroundGradient(ofColor(20), ofColor(0));   // pick your own backgroujnd
	ofBackground(ofColor::black);
//	cout << ofGetFrameRate() << endl;

	// draw the GUI
	//if (!bHide) gui.draw();

	// If statement used to display camera based on camNum
	if (camNum == 1)
		cam.begin();
	else if (camNum == 2)
		cam2.begin();
	else if (camNum == 3)
		cam3.begin();
	else if (camNum == 4)
		cam4.begin();

	ofPushMatrix();
	
	if (bWireframe) {                    // wireframe mode  (include axis)
		ofDisableLighting();
		ofSetColor(ofColor::slateGray);
		moon.drawWireframe();
		lander.drawWireframe();
		if (bRoverLoaded) {
			lander.drawWireframe();
			if (!bTerrainSelected) drawAxis(lander.getPosition());
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}
	else {
		ofEnableLighting();              // shaded mode
		moon.drawFaces();
		lander.drawFaces();
		if (bRoverLoaded) {
			lander.drawFaces();
			if (!bTerrainSelected) drawAxis(lander.getPosition());
		}
		//if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}


	if (bDisplayPoints) {                // display points as an option    
		glPointSize(3);
		ofSetColor(ofColor::green);
		moon.drawVertices();
		lander.drawVertices();
	}

	// highlight selected point (draw sphere around selected point)
	//
	if (bPointSelected) {
		ofSetColor(ofColor::blue);
		ofDrawSphere(selectedPoint, .1);
	}
	
	ofNoFill();
	//ofSetColor(ofColor::white);
	//drawBox(boundingBox);

	// draw all the lights 
	//
	/*
	ofSetColor(ofColor::aqua);
	keyLight.draw();
	fillLight.draw();
	rimLight.draw();

	ofSetColor(ofColor::red);
	for (int i=0; i < level1.size(); i++)
		drawBox(level1[i]);

	ofSetColor(ofColor::blue);
	for (int i = 0; i < level2.size(); i++)
		drawBox(level2[i]);

	ofSetColor(ofColor::yellow);
	for (int i = 0; i < level3.size(); i++)
		drawBox(level3[i]);
	*/
	
	//drawOct(root, 5, 0);
	
	//This was what I(Justin) originally used to test the drawing while I was debugging the recursion draw
	/*for (int i = 0; i < 8; i++) {
		drawBox(root.children[i].Box);
		for (int j = 0; j < root.children[i].children.size(); j++) {
			ofSetColor(ofColor::blue);
			drawBox(root.children[i].children[j].Box);
			for (int k = 0; k < root.children[i].children[j].children.size(); k++) {
				ofSetColor(ofColor::green);
				drawBox(root.children[i].children[j].children[k].Box);
			}
		}
	}*/

	//emitter.draw();//draw emitter

	//sys.draw();
	engine.draw();
	ofPopMatrix();

	//gui.draw();
	if (camNum == 1)
		cam.end();
	else if (camNum == 2)
		cam2.end();
	else if (camNum == 3)
		cam3.end();
	else if (camNum == 4)
		cam4.end();
	
}

// 

// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {

	ofPushMatrix();
	ofTranslate(location);

	ofSetLineWidth(1.0);

	// X Axis
	ofSetColor(ofColor(255, 0, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));
	

	// Y Axis
	ofSetColor(ofColor(0, 255, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));

	// Z Axis
	ofSetColor(ofColor(0, 0, 255));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));

	ofPopMatrix();
}


void ofApp::keyPressed(int key) {

	switch (key) {
		//directional controls by Justin Armijo
	case OF_KEY_UP:
		//playSound();
		thruster.add(ofVec3f(0, 0, .5));
		engine.setVelocity(ofVec3f(0, 0, -5));
		engine.start();
		break;
	case OF_KEY_DOWN:
		//playSound();
		thruster.add(ofVec3f(0, 0, -.5));
		engine.setVelocity(ofVec3f(0, 0, 5));
		engine.start();
		break;
	case OF_KEY_LEFT:
		//playSound();
		thruster.add(ofVec3f(-.5, 0, 0));
		engine.setVelocity(ofVec3f(5, 0, 0));
		engine.start();
		break;
	case OF_KEY_RIGHT:
		//playSound();
		thruster.add(ofVec3f(.3, 0, 0));
		engine.setVelocity(ofVec3f(-2, 0, 0));
		engine.start();
		break;
	case '1':
		camNum = 1; // Pressing number key changes value of camNum
		break;
	case '2':
		camNum = 2;
		break;
	case '3':
		camNum = 3;
		break;
	case '4':
		camNum = 4;
		break;
	case 'C':
	case 'c':
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
		else cam.enableMouseInput();
		break;
	case 'F':
	case 'f':
		ofToggleFullscreen();
		break;
	case 'H':
	case 'h':
		bHide = !bHide;
		break;
	case 'r':
		cam.reset();
		break;
	case 's':
		savePicture();
		break;
	case 't':
		cam.lookAt(lander.getPosition()); // Free camera will look at 3D model
		break;
	case 'u':
		break;
	case 'i':
	{
		ofVec3f vel = sys.particles[0].velocity; //Justin Armijo
		impulseForce.apply(-60 * vel);
		break;
	}
	case 'v':
		togglePointsDisplay();
		break;
	case 'V':
		break;
	case 'w':
		toggleWireframeMode();
		break;
	case ' ':
		//lander.setPosition(0, 0, 0);
		//playSound();
		thruster.add(ofVec3f(0, .5, 0));            //Justin Armijo
		engine.setVelocity(ofVec3f(0, -5, 0));
		engine.start();
		break;
		//emitter.start();
	case OF_KEY_ALT:
		cam.enableMouseInput();
		bAltKeyDown = true;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = true;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_DEL:
		break;
	default:
		break;
	}
}

void ofApp::toggleWireframeMode() {
	bWireframe = !bWireframe;
}

void ofApp::toggleSelectTerrain() {
	bTerrainSelected = !bTerrainSelected;
}

void ofApp::togglePointsDisplay() {
	bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {
	engine.stop();
	thruster.set(ofVec3f(0, 0, 0));
	switch (key) {
	
	case OF_KEY_ALT:
		cam.disableMouseInput();
		bAltKeyDown = false;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = false;
		break;
	case OF_KEY_SHIFT:
		break;
	default:
		break;

	}
}



//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
}


//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
    ofVec3f mouse(mouseX, mouseY);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z),
		Vector3(rayDir.x, rayDir.y, rayDir.z));
	float startTime = ofGetElapsedTimeMicros();
	recSearch(ray, root, selectedPoint);
	float endTime = ofGetElapsedTimeMicros();
	responseTime = endTime - startTime;
	string rt;
	rt += "Response Time: " + std::to_string(responseTime);
	cout << rt << endl;
}


//draw a box from a "Box" class  
//
void ofApp::drawBox(const Box &box) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	ofVec3f p = ofVec3f(center.x(), center.y(), center.z());
	float w = size.x();
	float h = size.y();
	float d = size.z();
	ofDrawBox(p, w, h, d);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box ofApp::meshBounds(const ofMesh & mesh) {
	int n = mesh.getNumVertices();
	ofVec3f v = mesh.getVertex(0);
	ofVec3f max = v;
	ofVec3f min = v;
	for (int i = 1; i < n; i++) {
		ofVec3f v = mesh.getVertex(i);

		if (v.x > max.x) max.x = v.x;
		else if (v.x < min.x) min.x = v.x;

		if (v.y > max.y) max.y = v.y;
		else if (v.y < min.y) min.y = v.y;

		if (v.z > max.z) max.z = v.z;
		else if (v.z < min.z) min.z = v.z;
	}
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}

//  Subdivide a Box into eight(8) equal size boxes, return them in boxList;
//
void ofApp::subDivideBox8(const Box &box, vector<Box> & boxList) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	float xdist = (max.x() - min.x()) / 2;
	float ydist = (max.y() - min.y()) / 2;
	float zdist = (max.z() - min.z()) / 2;
	Vector3 h = Vector3(0, ydist, 0);

	//  generate ground floor
	//
	Box b[8];
	b[0] = Box(min, center);
	b[1] = Box(b[0].min() + Vector3(xdist, 0, 0), b[0].max() + Vector3(xdist, 0, 0));
	b[2] = Box(b[1].min() + Vector3(0, 0, zdist), b[1].max() + Vector3(0, 0, zdist));
	b[3] = Box(b[2].min() + Vector3(-xdist, 0, 0), b[2].max() + Vector3(-xdist, 0, 0));

	boxList.clear();
	for (int i = 0; i < 4; i++)
		boxList.push_back(b[i]);

	// generate second story
	//
	for (int i = 4; i < 8; i++) {
		b[i] = Box(b[i - 4].min() + h, b[i - 4].max() + h);
		boxList.push_back(b[i]);
	}
}

//By Justin Armijo
void ofApp::buildTree(TreeNode & root, int interator)
{
	vector<Box> divisions;
	subDivideBox8(root.Box, divisions);
	vector<TreeNode> tempLeaves;
	for (int k = 0; k < 8; k++) {
		TreeNode temp;
		temp.Box = divisions[k];
		tempLeaves.push_back(temp);
	}
	for (int i = 0; i < root.points.size(); i++) {
		for (int j = 0; j < 8; j++) {
			ofVec3f p = mesh.getVertex(root.points[i]);
			Vector3 v = Vector3(p.x, p.y, p.z);
			if (tempLeaves[j].Box.inside(v)) {
				tempLeaves[j].points.push_back(root.points[i]);
			}
		}
	}
	for (int i = 0; i < 8; i++) {
		if (tempLeaves[i].points.size() > 1) {			
			root.children.push_back(tempLeaves[i]);
			//buildTree(root.children.back(), (interator));

			//if (interator > 0 || interator <= -1) {
				//interator--;
			buildTree(root.children.back(), (interator));
				
			//}
		}
		if (tempLeaves[i].points.size() == 1) {
			root.children.push_back(tempLeaves[i]);
			
		}
	}
		
}
void ofApp::drawOct(TreeNode & node, int numLevels, int level) {
	if (level >= numLevels) return;
	drawBox(node.Box);
	
	level++;
	for (int i = 0; i < node.children.size(); i++) {
		drawOct(node.children[i], numLevels, level);
		
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {


}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}


//
//  Select Target Point on Terrain by comparing distance of mouse to 
//  vertice points projected onto screenspace.
//  if a point is selected, return true, else return false;
//
bool ofApp::doPointSelection() {

	ofMesh mesh = moon.getMesh(0);
	int n = mesh.getNumVertices();
	float nearestDistance = 0;
	int nearestIndex = 0;

	bPointSelected = false;

	ofVec2f mouse(mouseX, mouseY);
	vector<ofVec3f> selection;

	// We check through the mesh vertices to see which ones
	// are "close" to the mouse point in screen space.  If we find 
	// points that are close, we store them in a vector (dynamic array)
	//
	for (int i = 0; i < n; i++) {
		ofVec3f vert = mesh.getVertex(i);
		ofVec3f posScreen = cam.worldToScreen(vert);
		float distance = posScreen.distance(mouse);
		if (distance < selectionRange) {
			selection.push_back(vert);
			bPointSelected = true;
		}
	}

	//  if we found selected points, we need to determine which
	//  one is closest to the eye (camera). That one is our selected target.
	//
	if (bPointSelected) {
		float distance = 0;
		for (int i = 0; i < selection.size(); i++) {
			ofVec3f point =  cam.worldToCamera(selection[i]);

			// In camera space, the camera is at (0,0,0), so distance from 
			// the camera is simply the length of the point vector
			//
			float curDist = point.length(); 

			if (i == 0 || curDist < distance) {
				distance = curDist;
				selectedPoint = selection[i];
			}
		}
	}
	return bPointSelected;
}

// Set the camera to use the selected point as it's new target
//  
void ofApp::setCameraTarget() {

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
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//
void ofApp::initLightingAndMaterials() {

	static float ambient[] =
	{ .5f, .5f, .5, 1.0f };
	static float diffuse[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float position[] =
	{5.0, 5.0, 5.0, 0.0 };

	static float lmodel_ambient[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float lmodel_twoside[] =
	{ GL_TRUE };


	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position);


	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
//	glEnable(GL_LIGHT1);
	glShadeModel(GL_SMOOTH);
} 

void ofApp::savePicture() {
	ofImage picture;
	picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
	picture.save("screenshot.png");
	cout << "picture saved" << endl;
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent(ofDragInfo dragInfo) {

	ofVec3f point;
	mouseIntersectPlane(ofVec3f(0, 0, 0), cam.getZAxis(), point);

	if (lander.loadModel(dragInfo.files[0])) {
		lander.setScaleNormalization(false);
		lander.setScale(.005, .005, .005);
		lander.setPosition(point.x, point.y, point.z);
		bRoverLoaded = true;
	}
	else cout << "Error: Can't load model" << dragInfo.files[0] << endl;
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
	ofVec2f mouse(mouseX, mouseY);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}

//by Justin Armijo
void ofApp::recSearch(Ray ray, TreeNode &root, ofVec3f &selected) {
	
	for (int i = 0; i < root.children.size(); i++) {
		
		if (root.children[i].Box.intersect(ray, -100, 100)) {
			if (root.children[i].children.size() == 0) {
				int index = root.children[i].points[0];
				selected = mesh.getVertex(index);
				bPointSelected = true;
			}
			else {
				recSearch(ray, root.children[i], selected);
			}
		}
	}

}

//by Justin Armijo
bool ofApp::intersect(const ofVec3f &point, TreeNode & node, ofVec3f & selected)
{
	bool result = false;
	Vector3 p = Vector3(point.x, point.y, point.z);
	for (int i = 0; i < node.children.size(); i++) {
		if (node.children[i].Box.inside(p)) {
			if (node.children[i].children.size() == 0) {
				result = true;
				//cout << "collision Intersect" << endl;
				return result;
			}
			else {
				result = intersect(point, node.children[i], selected);
				return result;
			}
		}
	}
	return result;
}

//by Justin Armijo
void ofApp::collisionDetect()
{
	ofVec3f vel = sys.particles[0].velocity;
	if (vel.y > 0) return;
	ofVec3f contactPt = sys.particles[0].position;
	ofVec3f node;
	//cout << intersect(contactPt, root, node) << endl;
	if (intersect(contactPt, root, node)) {
		cout << "collision" << endl;
		bCollision = true;
		ofVec3f norm = ofVec3f(0, 1, 0);
		ofVec3f f = (restitution + 1.0) * ((-vel.dot(norm))*norm);
		impulseForce.apply(60 * f);
	}
}