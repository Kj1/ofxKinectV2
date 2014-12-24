ofxKinectV2
===========

This is an openFrameworks addon for working with the Microsoft SDK for the Kinect V2. There's a few caveats: you need a fairly specific setup to get this working. You'll need

* A computer with a USB 3 port, preferably intel
* Windows 8.1 (bootcamp ok but VM, not)
* [Visual Studio 2012 or 2013](http://www.visualstudio.com/)
* [A Kinect V2 developer edition](http://www.microsoft.com/en-us/kinectforwindows/Purchase/developer-sku.aspx)
* latest public Kinect V2 SDK 
* OF 084

Forked from https://github.com/joshuajnoble/ofxKinectV2
features added:
* singlethreaded
* single buffered (frames align better)
* more mapping functions
* world stream (rgb float colors denote coordinates)
* different wait for new frames logic 

made specifically for kinect-projector calibration v2, a calibrated stream is available.
this stream overlaps the RGB & depth image so you know of each pixel the depth & RGB color.
This forms the world image; see tutorial here: 
https://www.youtube.com/watch?feature=player_embedded&v=llQM-OGsETQ


Just to answer a few of the initial questions:

Can I use this on OSX?

Nope.

Can I use this on Windows 7?

Nope.

Can I use this on Linux?

Nope.

Why not?

Because we're just wrapping the functionality provided by the Microsoft Kinect team.

Where's speech recognition and face tracking?

Coming soon.
