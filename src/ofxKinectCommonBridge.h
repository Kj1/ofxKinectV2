#pragma once

#include "ofMain.h"

#include "KCBv2LIB.h"
#pragma comment (lib, "KCBv2.lib") // add path to lib additional dependency dir $(TargetDir)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// not sure this is right
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Kv2Joint
{
  public:
	Kv2Joint(){}
	Kv2Joint(const _Joint& kcbPosition, const _JointOrientation& kcbOrientation)
	{
		jointOrientation.set(kcbOrientation.Orientation.x, kcbOrientation.Orientation.y, kcbOrientation.Orientation.z, kcbOrientation.Orientation.w);
		jointPosition.set(kcbPosition.Position.X, kcbPosition.Position.Y, kcbPosition.Position.Z);
		type = kcbPosition.JointType;
	}
	
	ofVec3f getPosition()
	{
		return jointPosition;
	}

	ofQuaternion getOrientation()
	{
		return jointOrientation;
	}

	TrackingState getTrackingState()
	{
		return trackingState;
	}

  protected:
	ofVec3f jointPosition;
	ofQuaternion jointOrientation;
	JointType type;
	TrackingState trackingState;
};

class Kv2Skeleton
{
  public:
	bool tracked;
	map<JointType, Kv2Joint> joints;
};

class ofxKinectCommonBridge {
  public:
	
	ofxKinectCommonBridge();
	~ofxKinectCommonBridge();

	// new API
	bool initSensor( );
	bool initDepthStream();
	bool initColorStream(ColorImageFormat format = ColorImageFormat_Rgba);
	bool initIRStream();
	bool initSkeletonStream();
	bool initBodyIndexStream();
	bool initCalibratedStream();
	bool initWorldStream();
	
	void start();


	void setDepthClipping(float nearClip=500, float farClip=8000);
	
	void update();
	
	//basic streams
	ofPixels&				getBodyIndexPixelsRef();
	ofPixels&				getColorPixelsRef();
	ofPixels&				getDepthPixelsRef();
	ofShortPixels&			getIRPixelsRef();
	
	//mapped streams
	ofFloatPixels&			getWorldPixelsRef();			//the 'world' camera space (x y z in meters)
	ofPixels&				getCalibratedColorPixelsRef();  //the calibrated RGB->depth image
	
	//basic streams
	ofTexture&				getBodyIndex();
	ofTexture&				getColor();
	ofTexture&				getDepth();
	ofTexture&				getIR();
	ofTexture&				getCalibratedColor();  //the calibrated RGB->depth image
	
	vector<Kv2Skeleton> getSkeletons();

	//Point functions
	//these might induce threading errors!
	ofVec3f mapDepthPointToWorldPoint(ofPoint depthPoint);
	ofVec2f mapDepthPointToColorCoint(ofPoint depthPoint);
	
	ofPoint mapWorldPointToDepthPoint(ofPoint worldPoint);
	ofPoint mapWorldPointToColorPoint(ofPoint worldPoint);
	
	//vector functions

		
	void drawSkeleton( int index, bool depthTrueColorFalse = true);

	bool isNewFrame();

  protected:

    KCBHANDLE hKinect;
	ColorImageFormat colorFormat;
	bool newFrame;
  	bool bInited;
	bool bStarted;
	vector<Kv2Skeleton> skeletons;

	//quantize depth buffer to 8 bit range
	vector<unsigned char> depthLookupTable;
	void updateDepthLookupTable();
	bool bNearWhite;
	float nearClipping, farClipping;
	
	//basic streams
	ofPixels bodyIndexPixels;
	ofPixels videoPixels;
	ofPixels depthPixels;
	ofShortPixels depthPixelsRaw;
	ofShortPixels irPixels;	
	//calibrated streams
	ofFloatPixels worldPixels;
	ofPixels	calibratedColorPixels;

	
	ofTexture videoTex;
	ofTexture depthTex;
	ofTexture bodyIndexTex;
	ofTexture calibratedColorTex;
	ofTexture irTex;

	

	//frame functions
	void generateCalibratedWorldFrame();
	void generateCalibratedColorFrame();
	
	bool bUsingSkeletons;
	
	bool bUsingDepth;
	bool bUsingBodyIndex;
	bool bUsingColorVideo;
	bool bUsingIRVideo;
	bool bUsingCalibratedColorStream;
	bool bUsingWorldStream;

	
	bool updateDepth;
	bool updateBodyIndex;
	bool updateColorVideo;
	bool updateIRVideo;
	bool updateCalibratedColorStream;
	bool updateWorldStream;

	BYTE *irPixelByteArray;

	void Function();

	KCBDepthFrame *pDepthFrame;
	KCBColorFrame *pColorFrame;
	KCBInfraredFrame *pInfraredFrame;
	KCBBodyIndexFrame *pBodyIndexFrame;
	
	KCBFrameDescription colorFrameDescription;
	KCBFrameDescription depthFrameDescription;
	KCBFrameDescription irFrameDescription;
	KCBFrameDescription bodyIndexFrameDescription;

	JointOrientation jointOrients[JointType_Count];
	Joint joints[JointType_Count];

	pair<JointType, JointType> skeletonDrawOrder[JointType_Count];
		
};
