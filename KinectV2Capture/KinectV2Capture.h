#pragma once
#include <opencv2\core.hpp>

#include <Kinect.h>
#include <memory>

class KinectV2Capture {
private:
	IKinectSensor* m_pKinectSensor;
	IMultiSourceFrameReader* m_pMultiSourceFrameReader;
	ICoordinateMapper* m_pMapper;

	ColorSpacePoint* m_pColorSpacePoints;
	CameraSpacePoint* m_pCameraSpacePoints;
	
	cv::Mat m_DepthImg;
	cv::Mat m_ColorImg;
	cv::Mat m_IrImg;

	RGBQUAD* m_pColorRGBX;// Color Frame buf
	UINT16* m_pDepthBuffer;

	static const int m_ColorWidth = 1920;
	static const int m_ColorHeight = 1080;
	static const int m_DepthWidth = 512;
	static const int m_DepthHeight = 424;

	cv::Size m_DepthSize;
	cv::Size m_ColorSize;

	bool m_bCamInited;

	bool m_DepthImgValid;
	bool m_ColorImgValid;

protected:
	HANDLE m_heventThreadDone;
	LPDWORD m_ThreadId;
	
	WAITABLE_HANDLE m_hMSEvent;


	static DWORD WINAPI threadGrabImage(LPVOID);

	void FrameArrived(IMultiSourceFrameArrivedEventArgs*);

	void DepthFrameArrived(IDepthFrameReference*);
	void ColorFrameArrived(IColorFrameReference*);

public:
	KinectV2Capture();

	void CreateCapture();
	void Run();
	void Close();
	void DestoryCapture();

};
