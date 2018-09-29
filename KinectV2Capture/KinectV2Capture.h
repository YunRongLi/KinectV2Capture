#pragma once
#include <opencv2\core.hpp>

#include <Kinect.h>
#include <memory>

#include <boost\signals2.hpp>

class KinectV2Capture {
	typedef void (ColorImg_Callback)(const boost::shared_ptr<cv::Mat> &);

private:
	IKinectSensor* m_pKinectSensor;
	IMultiSourceFrameReader* m_pMultiSourceFrameReader;
	ICoordinateMapper* m_pMapper;

	ColorSpacePoint* m_pColorSpacePoints;
	CameraSpacePoint* m_pCameraSpacePoints;

	RGBQUAD* m_pColorRGBX;// Color Frame buf
	UINT16* m_pDepthBuffer;

	static const int m_ColorWidth = 1920;
	static const int m_ColorHeight = 1080;
	static const int m_DepthWidth = 512;
	static const int m_DepthHeight = 424;

	cv::Mat m_DepthImg;

	cv::Size m_DepthSize;
	cv::Size m_ColorSize;

	bool m_bCamInited;

protected:
	HANDLE m_heventThreadDone;
	LPDWORD m_ThreadId;
	
	WAITABLE_HANDLE m_hMSEvent;

	boost::signals2::signal<ColorImg_Callback> m_Signal_Color;

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

	void registerCallback(const ColorImg_Callback& callback);

	bool GetPointCloud(cv::Mat&);
	cv::Mat GetROIImg(cv::Rect);
};
