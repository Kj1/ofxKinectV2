#include "ofxKinectCommonBridge.h"

//================================================================================================================
// common bridge
//================================================================================================================


ofxKinectCommonBridge::ofxKinectCommonBridge(){
	hKinect = NULL;

	pDepthFrame = NULL;
	pColorFrame = NULL;

	bUsingBodyIndex = false;
	bUsingIRVideo = false;
	bUsingColorVideo = false;
	bInited = false;
	bStarted = false;
	bUsingCalibratedColorStream = false;
	bUsingWorldStream = false;

	bUsingSkeletons = false;
	bUsingDepth = false;

	updateDepth = false;
	updateBodyIndex = false;
	updateColorVideo = false;
	updateIRVideo = false;
	updateCalibratedColorStream = false;
	updateWorldStream = false;

	setDepthClipping();
	newFrame = false;
	skeletonDrawOrder[0] = make_pair<JointType, JointType>(JointType_Head, JointType_Neck);
	skeletonDrawOrder[1] = make_pair<JointType, JointType>(JointType_Neck, JointType_SpineShoulder);
	skeletonDrawOrder[2] = make_pair<JointType, JointType>(JointType_SpineShoulder, JointType_SpineMid);
	skeletonDrawOrder[3] = make_pair<JointType, JointType>(JointType_SpineMid, JointType_SpineBase);
	skeletonDrawOrder[4] = make_pair<JointType, JointType>(JointType_SpineShoulder, JointType_ShoulderRight);
	skeletonDrawOrder[5] = make_pair<JointType, JointType>(JointType_SpineShoulder, JointType_ShoulderLeft);
	skeletonDrawOrder[6] = make_pair<JointType, JointType>(JointType_SpineBase, JointType_HipRight);
	skeletonDrawOrder[7] = make_pair<JointType, JointType>(JointType_SpineBase, JointType_HipLeft);

	// Right Arm    
	skeletonDrawOrder[8] = make_pair<JointType, JointType>(JointType_ShoulderRight, JointType_ElbowRight);
	skeletonDrawOrder[9] = make_pair<JointType, JointType>(JointType_ElbowRight, JointType_WristRight);
	skeletonDrawOrder[10] = make_pair<JointType, JointType>(JointType_WristRight, JointType_HandRight);
	skeletonDrawOrder[11] = make_pair<JointType, JointType>(JointType_HandRight, JointType_HandTipRight);
	skeletonDrawOrder[12] = make_pair<JointType, JointType>(JointType_WristRight, JointType_ThumbRight);

	// Left Arm
	skeletonDrawOrder[13] = make_pair<JointType, JointType>(JointType_ShoulderLeft, JointType_ElbowLeft);
	skeletonDrawOrder[14] = make_pair<JointType, JointType>(JointType_ElbowLeft, JointType_WristLeft);
	skeletonDrawOrder[15] = make_pair<JointType, JointType>(JointType_WristLeft, JointType_HandLeft);
	skeletonDrawOrder[16] = make_pair<JointType, JointType>(JointType_HandLeft, JointType_HandTipLeft);
	skeletonDrawOrder[17] = make_pair<JointType, JointType>(JointType_WristLeft, JointType_ThumbLeft);

	// Right Leg
	skeletonDrawOrder[18] = make_pair<JointType, JointType>(JointType_HipRight, JointType_KneeRight);
	skeletonDrawOrder[19] = make_pair<JointType, JointType>(JointType_KneeRight, JointType_AnkleRight);
	skeletonDrawOrder[20] = make_pair<JointType, JointType>(JointType_AnkleRight, JointType_FootRight);

	// Left Leg
	skeletonDrawOrder[21] = make_pair<JointType, JointType>(JointType_HipLeft, JointType_KneeLeft);
	skeletonDrawOrder[22] = make_pair<JointType, JointType>(JointType_KneeLeft, JointType_AnkleLeft);
	skeletonDrawOrder[23] = make_pair<JointType, JointType>(JointType_AnkleLeft, JointType_FootLeft);

}

ofxKinectCommonBridge::~ofxKinectCommonBridge(){

	bStarted = false;
	KCBCloseSensor(&hKinect);
	KCBReleaseBodyIndexFrame(&pBodyIndexFrame);
	KCBReleaseColorFrame(&pColorFrame);
	KCBReleaseDepthFrame(&pDepthFrame);
	KCBReleaseInfraredFrame(&pInfraredFrame);
}

void ofxKinectCommonBridge::start() {
	bStarted = true;	
}

//---------------------------------------------------------------------------
void ofxKinectCommonBridge::setDepthClipping(float nearClip, float farClip){
	nearClipping = nearClip;
	farClipping = farClip;
	updateDepthLookupTable();
}
bool ofxKinectCommonBridge::isNewFrame() {
	bool b = newFrame;
	newFrame = false;
	return b;
}
//---------------------------------------------------------------------------
void ofxKinectCommonBridge::updateDepthLookupTable()
{
	bNearWhite = false;
	unsigned char nearColor = bNearWhite ? 255 : 0;
	unsigned char farColor = bNearWhite ? 0 : 255;
	unsigned int maxDepthLevels = 10001;
	depthLookupTable.resize(maxDepthLevels);
	depthLookupTable[0] = 0;
	for(unsigned int i = 1; i < maxDepthLevels; i++)
	{
		depthLookupTable[i] = ofMap(i, nearClipping, farClipping, nearColor, farColor, true);
	}
}

void ofxKinectCommonBridge::update()
{
	if(!bStarted)
	{
		ofLogError("ofxKinectCommonBridge::update") << "Kinect not started";

		return;
	}

	long s1 = ofGetElapsedTimeMillis();

	//Kj1: We want to have all depth related frames arrived in sync.
	//The color cam is 15-30 fps so thats checked only when depth is updated
	if (!KCBIsFrameReady(hKinect, FrameSourceTypes_Depth) || 
		!KCBIsFrameReady(hKinect, FrameSourceTypes_Body) || 
		!KCBIsFrameReady(hKinect, FrameSourceTypes_BodyIndex)){
			newFrame = false;
			return;

	}

	if (bUsingBodyIndex)
	{
		if (SUCCEEDED(KCBGetBodyIndexFrame(hKinect, pBodyIndexFrame)))
		{
			updateBodyIndex = true;
		}
	}

	if (bUsingIRVideo)
	{
		if (SUCCEEDED(KCBGetInfraredFrame(hKinect, pInfraredFrame)))
		{
			updateIRVideo = true;
		}
	}

	if(bUsingColorVideo)
	{
		if (SUCCEEDED(KCBGetColorFrame(hKinect, pColorFrame)))
		{
			updateColorVideo = true;
		}
	}

	if (bUsingDepth)
	{
		if (SUCCEEDED(KCBGetDepthFrame(hKinect, pDepthFrame))) 
		{
			for(int i = 0; i < depthPixels.getWidth()*depthPixels.getHeight(); i++) {
				depthPixelsRaw.getPixels()[i] = pDepthFrame->Buffer[i];
				depthPixels.getPixels()[i]    = depthLookupTable[ofClamp(depthPixelsRaw.getPixels()[i], 0, depthLookupTable.size() - 1)];
			}
			updateDepth = true;			

			if (bUsingCalibratedColorStream){
				generateCalibratedColorFrame();
				updateCalibratedColorStream = true;
			}
			if (bUsingWorldStream) {
				generateCalibratedWorldFrame();
				updateWorldStream= true;
			}
		}
	}

	if(bUsingSkeletons) 
	{
		LONGLONG timestamp;
		IBodyFrame* pBodyFrame = NULL;
		IBody* ppBodies[BODY_COUNT] = { 0 };
		if (SUCCEEDED(KCBGetIBodyFrame(hKinect, &pBodyFrame)))
		{
			HRESULT hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);

			// buffer for later
			for (int i = 0; i < BODY_COUNT; ++i)
			{
				skeletons[i].joints.clear();
				skeletons[i].tracked = false;

				IBody *pBody = ppBodies[i];
				BOOLEAN isTracked = false;

				if (pBody == NULL)
				{
					continue;
				}

				HRESULT hr = pBody->get_IsTracked(&isTracked);
				if (isTracked)
				{
					HRESULT hrJoints = ppBodies[i]->GetJoints(JointType_Count, joints);
					HRESULT hrOrient = ppBodies[i]->GetJointOrientations(JointType_Count, jointOrients);
					if (FAILED(hrJoints))
					{
						ofLogError("ofxKinectCommonBridge::Function") << "Failed to get joints";
					}

					if (FAILED(hrOrient))
					{
						ofLogError("ofxKinectCommonBridge::Function") << "Failed to get orientations";
					}

					if (SUCCEEDED(hrJoints) && SUCCEEDED(hrOrient))
					{
						for (int j = 0; j < JointType_Count; ++j)
						{
							skeletons[i].joints[joints[j].JointType] = Kv2Joint(joints[j], jointOrients[j]);
						}
					}
					skeletons[i].tracked = true;
				}

				pBody->Release();
			}

			// all done clean up
			pBodyFrame->Release();
		}
	}
	newFrame = true;
}


//------------------------------------
ofPixels& ofxKinectCommonBridge::getBodyIndexPixelsRef(){	
	if(!bUsingBodyIndex){
		ofLogWarning("ofxKinectCommonBridge::bUsingBodyIndex") << "Getting body Pixels when stream unitialized";
	}
	return bodyIndexPixels;
}
//------------------------------------
ofPixels& ofxKinectCommonBridge::getColorPixelsRef(){
	if(!bUsingColorVideo){
		ofLogWarning("ofxKinectCommonBridge::getColorPixelsRef") << "Getting Color Pixels when color stream unitialized";
	}
	return videoPixels;
}

//------------------------------------
ofPixels & ofxKinectCommonBridge::getDepthPixelsRef(){       	///< grayscale values
	if(!bUsingDepth){
		ofLogWarning("ofxKinectCommonBridge::bUsingDepth") << "Getting depth Pixels when stream unitialized";
	}
	return depthPixels;
}

//------------------------------------
ofPixels & ofxKinectCommonBridge::getCalibratedColorPixelsRef(){       	///< grayscale values
	if(!bUsingDepth || !bUsingCalibratedColorStream){
		ofLogWarning("ofxKinectCommonBridge::calibratedColorPixels") << "Getting depth Pixels when stream unitialized";
	}
	return calibratedColorPixels;
}
//------------------------------------
ofFloatPixels & ofxKinectCommonBridge::getWorldPixelsRef(){       	///< grayscale values
	if(!bUsingDepth ||!bUsingWorldStream){
		ofLogWarning("ofxKinectCommonBridge::worldPixels") << "Getting depth Pixels when stream unitialized";
	}
	return worldPixels;
}

//------------------------------------
ofShortPixels& ofxKinectCommonBridge::getIRPixelsRef(){
	if(!bUsingIRVideo){
		ofLogWarning("ofxKinectCommonBridge::getIRPixelsRef") << "Getting IR Pixels with IR stream unitialized";
	}
	return irPixels;
}

//------------------------------------
vector<Kv2Skeleton> ofxKinectCommonBridge::getSkeletons(){	
	if(!bUsingSkeletons){
		ofLogWarning("ofxKinectCommonBridge::bUsingSkeletons") << "Getting skeletons when stream unitialized";
	}
	return skeletons;
}


bool ofxKinectCommonBridge::initSensor()
{
	if(bStarted){
		ofLogError("ofxKinectCommonBridge::initSensor") << "Cannot configure once the sensor has already started" << endl;
		return false;
	}

	if (ofGetCurrentRenderer()->getType() == ofGLProgrammableRenderer::TYPE){
		ofLogError("ofxKinectCommonBridge::initSensor") << "Using programmable renbder, unknow results!" << endl;
	}

	hKinect = KCBOpenDefaultSensor();

	return true;
}

bool ofxKinectCommonBridge::initDepthStream()
{

	if(hKinect == NULL){
		ofLogError("ofxKinectCommonBridge::initDepthStream") << "Cannot init depth stream until initSensor() is called";
		return false;
	}

	if(bStarted){
		ofLogError("ofxKinectCommonBridge::initDepthStream") << "Cannot configure once the sensor has already started";
		return false;
	}


	HRESULT hr;
	hr = KCBGetDepthFrameDescription(hKinect, &depthFrameDescription);

	//hr = KCBCreateDepthFrame(depthFrameDescription, &pDepthFrame);

	depthPixels.allocate(depthFrameDescription.width, depthFrameDescription.height, OF_IMAGE_GRAYSCALE);
	worldPixels.allocate(depthFrameDescription.width, depthFrameDescription.height, OF_IMAGE_COLOR);
	calibratedColorPixels.allocate(depthFrameDescription.width, depthFrameDescription.height, OF_IMAGE_COLOR);
	depthPixelsRaw.allocate(depthFrameDescription.width, depthFrameDescription.height, OF_IMAGE_GRAYSCALE);

	depthTex.allocate(depthFrameDescription.width, depthFrameDescription.height, GL_LUMINANCE);
	calibratedColorTex.allocate(depthFrameDescription.width, depthFrameDescription.height, GL_RGB);

	pDepthFrame = new KCBDepthFrame();
	pDepthFrame->Buffer = depthPixelsRaw.getPixels();
	pDepthFrame->Size = depthFrameDescription.lengthInPixels;

	bUsingDepth = true;
	return bInited;
}

bool ofxKinectCommonBridge::initColorStream(ColorImageFormat format)
{

	if(hKinect == NULL){
		ofLogError("ofxKinectCommonBridge::initDepthStream") << "Cannot init depth stream until initSensor() is called";
		return false;
	}

	KCBGetColorFrameDescription(hKinect, ColorImageFormat_Rgba, &colorFrameDescription);

	if (format != ColorImageFormat_Rgba)
	{
		videoPixels.allocate(colorFrameDescription.width, colorFrameDescription.height, 2);
	}
	else
	{
		videoPixels.allocate(colorFrameDescription.width, colorFrameDescription.height, OF_IMAGE_COLOR_ALPHA);
	}

	videoTex.allocate(colorFrameDescription.width, colorFrameDescription.height, GL_RGBA);
	pColorFrame = new KCBColorFrame();
	pColorFrame->Buffer = videoPixels.getPixels();
	pColorFrame->Size = colorFrameDescription.lengthInPixels * colorFrameDescription.bytesPerPixel;
	pColorFrame->Format = format;


	bUsingColorVideo = true;

	return true;
}

bool ofxKinectCommonBridge::initCalibratedStream(){
	bUsingCalibratedColorStream = true;
	return true;
}
bool ofxKinectCommonBridge::initWorldStream(){
	bUsingWorldStream = true;
	return true;
}

bool ofxKinectCommonBridge::initIRStream()
{
	if(bStarted){
		ofLogError("ofxKinectCommonBridge::startIRStream") << " Cannot configure when the sensor has already started";
		return false;
	}

	bUsingIRVideo = true;

	KCBGetInfraredFrameDescription(hKinect, &irFrameDescription);

	irTex.allocate(irFrameDescription.width, irFrameDescription.height, GL_LUMINANCE);
	irPixels.allocate(irFrameDescription.width, irFrameDescription.height, OF_IMAGE_GRAYSCALE);

	pInfraredFrame = new KCBInfraredFrame();
	pInfraredFrame->Buffer = irPixels.getPixels();
	pInfraredFrame->Size = irFrameDescription.lengthInPixels;

	bInited = true;
	return true;
}

bool ofxKinectCommonBridge::initBodyIndexStream()
{
	if (bStarted){
		ofLogError("ofxKinectCommonBridge::initBodyIndexStream") << "Cannot configure once the sensor has already started";
		return false;
	}
	HRESULT hr = KCBGetBodyIndexFrameDescription(hKinect, &bodyIndexFrameDescription);

	if (!SUCCEEDED(hr))
	{
		ofLogError("ofxKinectCommonBridge::initBodyIndexStream") << "cannot initialize stream";
		return false;
	}

	bodyIndexPixels.allocate(bodyIndexFrameDescription.width, bodyIndexFrameDescription.height, OF_IMAGE_GRAYSCALE);

	bodyIndexTex.allocate(bodyIndexFrameDescription.width, bodyIndexFrameDescription.height, GL_LUMINANCE);
	pBodyIndexFrame = new KCBBodyIndexFrame();
	pBodyIndexFrame->Buffer = bodyIndexPixels.getPixels();
	pBodyIndexFrame->Size = bodyIndexFrameDescription.lengthInPixels;

	bUsingBodyIndex = true;

	return true; //??
}

bool ofxKinectCommonBridge::initSkeletonStream()
{
	if(bStarted){
		ofLogError("ofxKinectCommonBridge::initSkeletonStream") << "Cannot configure once the sensor has already started";
		return false;
	}

	skeletons.resize(6);

	bUsingSkeletons = true;
	return true;
}


//edit KJ1
ofVec3f ofxKinectCommonBridge::mapDepthPointToWorldPoint(ofPoint depthPoint){
	
	int loc = depthPoint.y*depthFrameDescription.width+depthPoint.x;
	//we should test if the point is inside the depth freaem
	if (loc < 0 || loc > depthFrameDescription.width * depthFrameDescription.height) {
		return ofVec3f(0,0,0);
	}
	ofFloatColor c = worldPixels.getColor(depthPoint.x, depthPoint.y);
	ofVec3f o (c.r, c.g, c.b);
	return o;
}

//----------------------------------------------------------
ofVec2f ofxKinectCommonBridge::mapDepthPointToColorCoint(ofPoint depthPoint){


	//KJ1 edit
	vector<DepthSpacePoint> depthPixels;
	vector<UINT16> depths;
	vector<ColorSpacePoint> colorPoints;
	vector<ofVec2f> colorPointsOut;

	int depthArraySize = 1; 
	int depthArrayMax = depthFrameDescription.width * depthFrameDescription.height;

	depthPixels.resize(depthArraySize);
	depthPixels[0].X = depthPoint.x;
	depthPixels[0].Y = depthPoint.y;

	depths.resize(depthArraySize);
	int loc = depthPoint.y*depthFrameDescription.width+depthPoint.x;
	//we should test if the point is inside the depth freaem
	if (loc < 0 || loc > depthArrayMax) {
		return ofVec2f();
	}

	depths[0] = (UINT16)depthPixelsRaw.getPixels()[loc];

	colorPoints.resize(depthArraySize);
	colorPointsOut.resize(depthArraySize);

	HRESULT mapResult;
	mapResult = KCBMapDepthPointsToColorSpace(hKinect, 
		depthArraySize, &depthPixels[0],
		depthArraySize, &depths[0],
		depthArraySize, &colorPoints[0]);



	ColorSpacePoint& p = colorPoints[0]; 
	colorPointsOut[0].set(ofVec2f(ofClamp(p.X,0,colorFrameDescription.width-1),
		ofClamp(p.Y,0,colorFrameDescription.height-1)) );

	return colorPointsOut[0];

}

ofPoint ofxKinectCommonBridge::mapWorldPointToDepthPoint(ofPoint worldPoint) {
	
	//KJ1 edit
	vector<CameraSpacePoint> cameraPixels;
	vector<DepthSpacePoint> depthPoints;
	vector<ofVec2f> depthPointsOut;

	int depthArraySize = 1; 
	int depthArrayMax = depthFrameDescription.width * depthFrameDescription.height;

	cameraPixels.resize(depthArraySize);
	cameraPixels[0].X = worldPoint.x;
	cameraPixels[0].Y = worldPoint.y;
	cameraPixels[0].Z = worldPoint.z;

	depthPoints.resize(depthArraySize);
	depthPointsOut.resize(depthArraySize);

	HRESULT mapResult;
	mapResult = KCBMapCameraPointsToDepthSpace(hKinect, 
		depthArraySize, &cameraPixels[0], //camerapoint
		depthArraySize, &depthPoints[0]);	//depthPoint
	
	DepthSpacePoint& p = depthPoints[0]; 
	depthPointsOut[0].set(p.X, p.Y);

	return depthPointsOut[0];
}
ofPoint ofxKinectCommonBridge::mapWorldPointToColorPoint(ofPoint worldPoint) {
	
	//KJ1 edit
	vector<CameraSpacePoint> cameraPixels;
	vector<ColorSpacePoint> colorPoints;
	vector<ofVec2f> colorPointsOut;

	int depthArraySize = 1; 
	int depthArrayMax = depthFrameDescription.width * depthFrameDescription.height;

	cameraPixels.resize(depthArraySize);
	cameraPixels[0].X = worldPoint.x;
	cameraPixels[0].Y = worldPoint.y;
	cameraPixels[0].Z = worldPoint.z;

	colorPoints.resize(depthArraySize);
	colorPointsOut.resize(depthArraySize);

	HRESULT mapResult;
	mapResult = KCBMapCameraPointsToColorSpace(hKinect, 
		depthArraySize, &cameraPixels[0], //camerapoint
		depthArraySize, &colorPoints[0]);	//depthPoint
	
	ColorSpacePoint& p = colorPoints[0]; 
	colorPointsOut[0].set(p.X, p.Y);

	return colorPointsOut[0];
}

//new github method, twice as fast
void ofxKinectCommonBridge::generateCalibratedWorldFrame() {

	int depthArraySize = depthFrameDescription.width * depthFrameDescription.height;
	vector<UINT16> depths; //input
	depths.resize(depthArraySize);
	for(int y = 0; y < depthFrameDescription.height; y++){
		for(int x = 0; x < depthFrameDescription.width; x++) {
			int i = y*depthFrameDescription.width+x;
			depths[i] = (UINT16) depthPixelsRaw.getPixels()[i];
		}
	}

	//output
	vector<CameraSpacePoint> cameraSpacePoints;
	cameraSpacePoints.resize(depthArraySize);

	HRESULT mapResult;
	mapResult = KCBMapDepthFrameToCameraSpace(
		hKinect,
		depthArraySize, &depths[0],
		depthArraySize, &cameraSpacePoints[0]);


	if(!worldPixels.isAllocated() || 
		worldPixels.getWidth() != depthFrameDescription.width || worldPixels.getWidth() != depthFrameDescription.height)
	{
		worldPixels.allocate(depthFrameDescription.width,depthFrameDescription.height, OF_IMAGE_COLOR);
	}

	memset(worldPixels.getPixels(), 0, worldPixels.getWidth()*worldPixels.getHeight()*worldPixels.getBytesPerPixel());

	for(int y = 0; y < depthFrameDescription.height; y++){
		for(int x = 0; x < depthFrameDescription.width; x++) {		
			int depthFrameIndex = y * depthFrameDescription.width + x;
			CameraSpacePoint& p = cameraSpacePoints[depthFrameIndex]; 
			worldPixels.setColor(x,y, ofFloatColor(p.X, p.Y, p.Z));
		}
	}
}

//new github method, twice as fast
void ofxKinectCommonBridge::generateCalibratedColorFrame() {

	int depthArraySize = depthFrameDescription.width * depthFrameDescription.height;
	vector<UINT16> depths; //input
	depths.resize(depthArraySize);
	for(int y = 0; y < depthFrameDescription.height; y++){
		for(int x = 0; x < depthFrameDescription.width; x++) {
			int i = y*depthFrameDescription.width+x;
			depths[i] = (UINT16)depthPixelsRaw.getPixels()[i];
		}
	}

	//output
	vector<ColorSpacePoint> colorPoints;
	colorPoints.resize(depthArraySize);

	HRESULT mapResult;
	mapResult = KCBMapDepthFrameToColorSpace(
		hKinect,
		depthArraySize, &depths[0],
		depthArraySize, &colorPoints[0]);

	if(!calibratedColorPixels.isAllocated() || 
		calibratedColorPixels.getWidth() != depthFrameDescription.width || calibratedColorPixels.getWidth() != depthFrameDescription.height)
	{
		calibratedColorPixels.allocate(depthFrameDescription.width,depthFrameDescription.height, OF_IMAGE_COLOR);
	}

	memset(calibratedColorPixels.getPixels(), 0, calibratedColorPixels.getWidth()*calibratedColorPixels.getHeight()*calibratedColorPixels.getBytesPerPixel());

	for(int y = 0; y < depthFrameDescription.height; y++){
		for(int x = 0; x < depthFrameDescription.width; x++) {		
			int depthFrameIndex = y * depthFrameDescription.width + x;
			ColorSpacePoint& p = colorPoints[depthFrameIndex]; 
			if(p.X >= 0 && p.X < colorFrameDescription.width && p.Y >= 0 && p.Y < colorFrameDescription.height)
			{
				calibratedColorPixels.setColor(x,y, videoPixels.getColor(p.X,p.Y));
			}
		}
	}
}



ofTexture&				ofxKinectCommonBridge::getBodyIndex(){
	if(updateBodyIndex) bodyIndexTex.loadData(getBodyIndexPixelsRef(), GL_LUMINANCE);
	updateBodyIndex = false;
	return bodyIndexTex;
}
ofTexture&				ofxKinectCommonBridge::getColor(){
	if (updateColorVideo) 	videoTex.loadData(getColorPixelsRef(), GL_RGBA);
	updateColorVideo = false;
	return videoTex;
}
ofTexture&				ofxKinectCommonBridge::getDepth(){
	if (updateDepth) depthTex.loadData(getDepthPixelsRef(),  GL_LUMINANCE);
	updateDepth = false;
	return depthTex;

}
ofTexture&				ofxKinectCommonBridge::getIR(){
	if (updateIRVideo) irTex.loadData(getIRPixelsRef(), GL_LUMINANCE);
	updateIRVideo = false;
	return irTex;
}
ofTexture&				ofxKinectCommonBridge::getCalibratedColor(){
	if (updateCalibratedColorStream) calibratedColorTex.loadData(getCalibratedColorPixelsRef(), GL_RGB);
	updateCalibratedColorStream = false;
	return calibratedColorTex;
}


void ofxKinectCommonBridge::drawSkeleton( int index, bool depthTrueColorFalse )
{
	if(index >= skeletons.size())
	{
		return;
	}

	if (!skeletons[index].tracked)
	{
		return;
	}

	for (int i = 0; i < JointType_Count; i++)
	{
		ofSetLineWidth(2);
		ofPoint lineBegin ;
		ofPoint lineEnd ;
		
		if (depthTrueColorFalse) {
			lineBegin = mapWorldPointToDepthPoint(skeletons[index].joints[skeletonDrawOrder[i].first].getPosition());
			lineEnd = mapWorldPointToDepthPoint(skeletons[index].joints[skeletonDrawOrder[i].second].getPosition());
		}	else {
			lineBegin = mapWorldPointToColorPoint(skeletons[index].joints[skeletonDrawOrder[i].first].getPosition());
			lineEnd = mapWorldPointToColorPoint(skeletons[index].joints[skeletonDrawOrder[i].second].getPosition());
		}	
		ofSetColor(0, 255, 0);
		ofLine(lineBegin, lineEnd);
		ofSetColor(255, 0, 0);
		ofCircle(lineEnd, 10);
	}

	ofSetColor(255, 255, 255);
}