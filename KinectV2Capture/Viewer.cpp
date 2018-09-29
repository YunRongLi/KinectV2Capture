#include <iostream>
#include <mutex>

#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>

#include <Eigen\Dense.h>

#include "TcpClient.h"

#include "KinectV2Capture.h"

#include "QRCodeScanner.h"

#define M_PI 3.1415926

cv::Mat colorImg;
cv::Mat roi;
cv::Mat PointCloud;
cv::Rect rect(960, 600, 250, 150);
cv::Rect QR_rect;

bool NewPose;
vector<string> Pose;

std::mutex gMutex;

const float Camera_X = 120.8; // 118.8
const float Camera_Y = -0.5; // 2
const float Camera_Z = 152; // 152.0
const float Pitch = -158 * M_PI / 180.0;

void DeterminePose(cv::Point RoiCenterPoint);
float DetermineNormalVector(std::vector<std::vector<Eigen::Vector4f>>);

void CombineString(string& dstStr, vector<string>& srcVec, const string& Separator) {
	dstStr = "";
	size_t vecSize = srcVec.size();

	for (size_t index = 0; index < vecSize; index++) {
		dstStr.append(srcVec.at(index));

		if (index < (vecSize - 1)) {
			dstStr.append(Separator);
		}
	}
}

static void onMouseCB(int event, int x, int y, int, void*) {
	if (event == CV_EVENT_LBUTTONDOWN) {
		cv::Point chance_point;
		chance_point.x = x;
		chance_point.y = y;

		DeterminePose(chance_point);
	}
}

void ColorImage_callback(const boost::shared_ptr<cv::Mat> &img) {
	std::lock_guard<std::mutex> lock(gMutex);
	colorImg = img->clone();
}

int main()
{
	KinectV2Capture Cap;
	TcpClient Client;
	QRCodeScanner Scanner;
	string msg;
	
	Client.InitSocket();
	Client.ConnectSocket("127.0.0.1", 4568);

	Cap.registerCallback(ColorImage_callback);

	Cap.CreateCapture();
	Scanner.StartScan();

	cv::namedWindow("ColorFrame", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("ROIFrame", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("PointCloud", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback("ROIFrame", onMouseCB, 0);

	bool quit = false;
	while (!quit){
		if (Client.isReceNewData()) {
			msg = Client.GetData();
			if (msg == "Start") {
				if (Cap.GetPointCloud(PointCloud)) {
					cv::imshow("PointCloud", PointCloud);
				}
			}
		}

		if (!colorImg.empty()) {
			std::lock_guard<std::mutex> lock(gMutex);
			cv::imshow("ColorFrame", colorImg);
			Scanner.SetLastFrame(colorImg);
		}		

		if (Scanner.isScanning()) {
			if (Scanner.isHaveNewRect()) {
				QR_rect = Scanner.GetRect();
				roi = Cap.GetROIImg(QR_rect).clone();
				cv::imshow("ROIFrame", roi);
			}
		}
		
		if (NewPose) {
			CombineString(msg, Pose, "[");
			Client.SendData(msg);
			Pose.clear();
			NewPose = false;
		}

		int key = cv::waitKey(1);
		switch (key) {
		case 'q':
			Cap.Close();
			quit = true;
			break;
		case 'p':
			if (Cap.GetPointCloud(PointCloud)) {
				cv::imshow("PointCloud", PointCloud);
			}
			break;
		default:
			break;
		}
	}

	Cap.DestoryCapture();

	return 0;
}

void DeterminePose(cv::Point Roi_DrillPoint) {
	Eigen::Matrix4f TransforMatrix;
	TransforMatrix << cos(Pitch), 0, sin(Pitch), Camera_X,
		                       0, 1,          0, Camera_Y,
		             -sin(Pitch), 0, cos(Pitch), Camera_Z,
		                       0, 0,          0, 1;

	cv::Point Color_DrillPoint = cv::Point(Roi_DrillPoint.x + QR_rect.x, Roi_DrillPoint.y + QR_rect.y);

	std::vector<std::vector<Eigen::Vector4f>> World_Points(5, std::vector<Eigen::Vector4f>(1));
	Eigen::Vector4f Camera_DrillPoint, World_Point;
	Camera_DrillPoint(0) =  PointCloud.at<cv::Vec3f>(Color_DrillPoint)[1];
	Camera_DrillPoint(1) = -PointCloud.at<cv::Vec3f>(Color_DrillPoint)[0];
	Camera_DrillPoint(2) =  PointCloud.at<cv::Vec3f>(Color_DrillPoint)[2];
	Camera_DrillPoint(3) =  1;

	World_Point = TransforMatrix * Camera_DrillPoint;

	for (int index = 0; index < 5; index++) {
		World_Points.at(index).at(0) = World_Point;
	}
	
	//World_Points.push_back(World_Point);

	for (int index = 0; index < 5; index++) {
		int distance = index + 1;
		for (int x = -1; x < 2; x++) {
			for (int y = -1; y < 2; y++) {
				if (x != 0 && y != 0) {
					Eigen::Vector4f Camera_NeighborPoint;
					Eigen::Vector4f World_NeighborPoint;
					cv::Point PointCloudPosition = cv::Point(Color_DrillPoint.x + distance* x, Color_DrillPoint.y + distance * y);
					Camera_NeighborPoint(0) =  PointCloud.at<cv::Vec3f>(PointCloudPosition)[1];
					Camera_NeighborPoint(1) = -PointCloud.at<cv::Vec3f>(PointCloudPosition)[0];
					Camera_NeighborPoint(2) =  PointCloud.at<cv::Vec3f>(PointCloudPosition)[2];
					Camera_NeighborPoint(3) = 1;
					World_NeighborPoint = TransforMatrix * Camera_NeighborPoint;
					World_Points.at(index).push_back(World_NeighborPoint);
				}
			}
		}
	}
	
	//float angle = DetermineNormalVector(World_Points);

	int A_angle = int(DetermineNormalVector(World_Points) * 1000);
	int X = int(World_Point(0) * 10000 + 90000);
	int Y = int(World_Point(1) * 10000);
	int Z = int(World_Point(2) * 10000 - 130000);

	Pose.push_back(std::to_string(X));
	Pose.push_back(std::to_string(Y));
	Pose.push_back(std::to_string(Z));
	Pose.push_back(std::to_string(A_angle));
	NewPose = true;
}

float DetermineNormalVector(std::vector<std::vector<Eigen::Vector4f>> WorldPoints) {
	Eigen::Vector3f VectorCA, VectorCB, NormalVector1, NormalVector2, NormalVector;

	std::vector<float> Angles;

	float a = 0.0;

	for (int index = 0; index < WorldPoints.size(); index++) {
		VectorCA(0) = WorldPoints.at(index).at(1)(0) - WorldPoints.at(index).at(0)(0);
		VectorCA(1) = WorldPoints.at(index).at(1)(1) - WorldPoints.at(index).at(0)(1);
		VectorCA(2) = WorldPoints.at(index).at(1)(2) - WorldPoints.at(index).at(0)(2);

		VectorCB(0) = WorldPoints.at(index).at(2)(0) - WorldPoints.at(index).at(0)(0);
		VectorCB(1) = WorldPoints.at(index).at(2)(1) - WorldPoints.at(index).at(0)(1);
		VectorCB(2) = WorldPoints.at(index).at(2)(2) - WorldPoints.at(index).at(0)(2);

		NormalVector1 = VectorCB.cross(VectorCA);

		VectorCA(0) = WorldPoints.at(index).at(3)(0) - WorldPoints.at(index).at(0)(0);
		VectorCA(1) = WorldPoints.at(index).at(3)(1) - WorldPoints.at(index).at(0)(1);
		VectorCA(2) = WorldPoints.at(index).at(3)(2) - WorldPoints.at(index).at(0)(2);

		VectorCB(0) = WorldPoints.at(index).at(4)(0) - WorldPoints.at(index).at(0)(0);
		VectorCB(1) = WorldPoints.at(index).at(4)(1) - WorldPoints.at(index).at(0)(1);
		VectorCB(2) = WorldPoints.at(index).at(4)(2) - WorldPoints.at(index).at(0)(2);

		NormalVector2 = VectorCA.cross(VectorCB);

		NormalVector(0) = (NormalVector1(0) + NormalVector2(0)) / 2;
		NormalVector(1) = (NormalVector1(1) + NormalVector2(1)) / 2;
		NormalVector(2) = (NormalVector1(2) + NormalVector2(2)) / 2;

		float Distance_Normal = abs(sqrtf(NormalVector(0) * NormalVector(0) + NormalVector(1) * NormalVector(1) + NormalVector(2) * NormalVector(2)));
		float Unit_X = NormalVector(0) / Distance_Normal;
		float Unit_Y = NormalVector(1) / Distance_Normal;
		float Unit_Z = NormalVector(2) / Distance_Normal;
		float Angle_A = 90.0 - atan2f(Unit_Z, Unit_Y) / M_PI * 180;
		Angles.push_back(Angle_A);
	}	

	return a;
}

