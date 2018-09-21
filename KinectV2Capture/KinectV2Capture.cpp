#include <iostream>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>

#include "KinectV2Capture.h"

int main()
{
	KinectV2Capture Cap;

	Cap.CreateCapture();


	//std::cin.get();
	/*char ch = 'w';
	bool quit = false;*/
	while (1) {
		/*std::cin >> ch;
		switch (ch)
		{
		case 'q':
			Cap.DestoryCapture();
			quit = true;
			break;
		default:
			break;
		}*/
	}

	return 0;
}

KinectV2Capture::KinectV2Capture() :m_pKinectSensor(nullptr), m_ColorImgValid(false), m_DepthImgValid(false), m_bCamInited(false),
									m_hMSEvent(NULL)
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);

	if (FAILED(hr)) {
		std::cerr << "Could not get default kinect sensor" << std::endl;
		return;
	}

	if (m_pKinectSensor) {
		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pMapper);
		}

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Depth |
															 FrameSourceTypes::FrameSourceTypes_Color |
														     FrameSourceTypes::FrameSourceTypes_Infrared,
														     &m_pMultiSourceFrameReader);
		}

		if (SUCCEEDED(hr)) {
			m_pMultiSourceFrameReader->SubscribeMultiSourceFrameArrived(&m_hMSEvent);
		}
	}

	if (!m_pKinectSensor || FAILED(hr)) {
		std::cerr << "No ready Kinect." << std::endl;
	}

	m_bCamInited = true;

	m_DepthSize = cv::Size(m_DepthWidth, m_DepthHeight);
	m_ColorSize = cv::Size(m_ColorWidth, m_ColorHeight);

	m_pColorRGBX = new RGBQUAD[m_ColorWidth * m_ColorHeight];
	m_pColorSpacePoints = new ColorSpacePoint[m_DepthWidth * m_DepthHeight];
	m_pCameraSpacePoints = new CameraSpacePoint[m_ColorWidth * m_ColorHeight];

	cv::namedWindow("ColorFrame", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("DepthFrame", cv::WINDOW_AUTOSIZE);
}

DWORD WINAPI KinectV2Capture::threadGrabImage(LPVOID pParam) {
	KinectV2Capture* pCapture = ((KinectV2Capture*)pParam);

	pCapture->Run();

	return 0;
}

void KinectV2Capture::CreateCapture() {
	m_heventThreadDone = CreateThread(NULL, 0, KinectV2Capture::threadGrabImage, this, 0, m_ThreadId);
}

void KinectV2Capture::Run() {
	if (!m_pMultiSourceFrameReader) {
		return;
	}

	IMultiSourceFrame* pMultiSourceFrame = NULL;


	HANDLE handles[] = { reinterpret_cast<HANDLE>(m_hMSEvent) };

	int index;
	DWORD result;
	HRESULT hr;

	while (m_bCamInited) {
		result = MsgWaitForMultipleObjects(1, handles, FALSE, 100, QS_ALLINPUT);

		if (result == (WAIT_OBJECT_0)) {
			IMultiSourceFrameArrivedEventArgs* pFrameArgs = nullptr;
			hr = m_pMultiSourceFrameReader->GetMultiSourceFrameArrivedEventData(m_hMSEvent, &pFrameArgs);

			if (SUCCEEDED(hr)) {
				FrameArrived(pFrameArgs);
			}

			pFrameArgs->Release();
		}

		/*if (m_ColorImgValid) {
			cv::imshow("ColorFrame", m_ColorImg);
			std::cout << "Show Color Img " << std::endl;
		}

		if (m_DepthImgValid) {
			cv::imshow("DepthFrame", m_DepthImg);
		}*/

		//int key = cv::waitKey(10);
	}

	Close();
}

void KinectV2Capture::Close() {
	cv::destroyAllWindows();

	if (m_pColorRGBX) {
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	if (m_pColorSpacePoints != NULL) {
		delete[] m_pColorSpacePoints;
		m_pColorSpacePoints = NULL;
	}
	if (m_pCameraSpacePoints != NULL) {
		delete[] m_pCameraSpacePoints;
		m_pCameraSpacePoints = NULL;
	}

	if (m_pMultiSourceFrameReader) {
		m_pMultiSourceFrameReader->Release();
	}
	
	if (m_pMapper) {
		m_pMapper->Release();
	}

	if (m_pKinectSensor) {
		m_pKinectSensor->Close();
	}
	
	m_pKinectSensor->Release();
}

void KinectV2Capture::DestoryCapture() {
	if (m_bCamInited) {
		m_bCamInited = false;
		WaitForSingleObject(m_heventThreadDone, INFINITY);
	}
}

void KinectV2Capture::FrameArrived(IMultiSourceFrameArrivedEventArgs* pArgs) {
	HRESULT hr;

	//IDepthFrameReference* pDepthFrameReference = nullptr;
	//IColorFrameReference* pColorFrameReference = nullptr;

	IMultiSourceFrameReference* pFrameReference = nullptr;

	hr = pArgs->get_FrameReference(&pFrameReference);

	if (SUCCEEDED(hr)) {
		IMultiSourceFrame* pFrame = nullptr;

		hr = pFrameReference->AcquireFrame(&pFrame);

		if (SUCCEEDED(hr)) {
			IDepthFrameReference* pDepthFrameReference = nullptr;
			IColorFrameReference* pColorFrameReference = nullptr;

			hr = pFrame->get_DepthFrameReference(&pDepthFrameReference);
			if (SUCCEEDED(hr)) {
				DepthFrameArrived(pDepthFrameReference);
			}

			pDepthFrameReference->Release();
			pDepthFrameReference = nullptr;

			hr = pFrame->get_ColorFrameReference(&pColorFrameReference);
			if (SUCCEEDED(hr)){
				ColorFrameArrived(pColorFrameReference);
			}

			pColorFrameReference->Release();;
			pColorFrameReference = nullptr;
		}

		pFrameReference->Release();
	}

	/*hr = pSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
	if (SUCCEEDED(hr)) {
		DepthFrameArrived(pDepthFrameReference);
		std::cout << "Start Store Depth Img" << std::endl;
	}
		
	pDepthFrameReference->Release();
	pDepthFrameReference = NULL;

	hr = pSourceFrame->get_ColorFrameReference(&pColorFrameReference);
	if (SUCCEEDED(hr)) {
		ColorFrameArrived(pColorFrameReference);
	}

	pColorFrameReference->Release();
	pColorFrameReference = NULL;

	pSourceFrame->Release();*/
}

void KinectV2Capture::DepthFrameArrived(IDepthFrameReference* pDepthFrameReference) {
	IDepthFrame* pDepthFrame = NULL;
	HRESULT hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
	if (FAILED(hr)) {
		return;
	}

	IFrameDescription* pDepthFrameDesciption = NULL;
	int nDepthWidth = 0;
	int nDepthHeight = 0;
	UINT nDepthBufferSize = 0;

	hr = pDepthFrame->get_FrameDescription(&pDepthFrameDesciption);
	if (SUCCEEDED(hr)) {
		hr = pDepthFrameDesciption->get_Width(&nDepthWidth);
	}
	if (SUCCEEDED(hr)) {
		hr = pDepthFrameDesciption->get_Height(&nDepthHeight);
	}
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &m_pDepthBuffer);
		cv::Mat img = cv::Mat(m_DepthSize, CV_16UC1, m_pDepthBuffer, cv::Mat::AUTO_STEP);
		m_DepthImg = img.clone();
		m_DepthImgValid = true;
	}
	pDepthFrameDesciption->Release();
	pDepthFrameDesciption = NULL;
	pDepthFrame->Release();
	pDepthFrame = NULL;
}

void KinectV2Capture::ColorFrameArrived(IColorFrameReference* pColorFrameReference) {
	IColorFrame* pColorFrame = NULL;
	HRESULT hr = pColorFrameReference->AcquireFrame(&pColorFrame);
	if (FAILED(hr)) {
		return;
	}

	IFrameDescription* pColorFrameDescription = NULL;
	int nColorWidth = 0;
	int nColorHeight = 0;
	ColorImageFormat ImageFormat = ColorImageFormat_None;
	UINT nColorBufferSize = 0;
	RGBQUAD* pColorBuffer = NULL;

	hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
	if (SUCCEEDED(hr)) {
		hr = pColorFrameDescription->get_Width(&nColorWidth);
	}
	if (SUCCEEDED(hr)) {
		hr = pColorFrameDescription->get_Height(&nColorHeight);
	}
	if (SUCCEEDED(hr)) {
		hr = pColorFrame->get_RawColorImageFormat(&ImageFormat);
	}
	if (SUCCEEDED(hr)) {
		if (ImageFormat == ColorImageFormat_Bgra) {
			hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
		}
		else if (m_pColorRGBX) {
			pColorBuffer = m_pColorRGBX;
			nColorBufferSize = m_ColorWidth * m_ColorHeight * sizeof(RGBQUAD);
			hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
		}
		else {
			hr = E_FAIL;
		}
		if (SUCCEEDED(hr)) {
			cv::Mat img = cv::Mat(m_ColorSize, CV_8UC4, pColorBuffer, cv::Mat::AUTO_STEP);
			m_ColorImg = img.clone();
			m_ColorImgValid = true;
		}
	}
	pColorFrameDescription->Release();
	pColorFrameDescription = NULL;
	pColorFrame->Release();
	pColorFrame = NULL;
}
