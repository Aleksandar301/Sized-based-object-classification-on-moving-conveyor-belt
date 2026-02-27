#include "stdafx.h"

#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <iomanip>
#include "Serial.h"
#include "MV_2022_SortingLine_Template.h"

#include <queue>

// Setting for using Basler GigE cameras.
#include <pylon/gige/BaslerGigEInstantCamera.h>
//typedef Pylon::CBaslerGigEInstantCamera Camera_t;
typedef Pylon::CInstantCamera Camera_t;

//using namespace Basler_GigECameraParams;
// Namespace for using pylon objects.
using namespace Pylon;
using namespace cv;
using namespace std;

// Namespace for using GenApi objects
using namespace GenApi;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 100;

struct SortingLineObject {
	int beginning_position;
	int end_position;
};

std::queue<SortingLineObject> objects_queue;

#define IN_OBJECT 1
#define OUT_OF_OBJECT 0
int scanning_status = OUT_OF_OBJECT;

#define WAITING_ON_OBJECT 1
#define IDLE 0
int acquisition_status = IDLE;

Mat image;
CPylonImage image2;
Mat cv_img, gray;
tstring com = _T("\\\\.\\COM20");
char stop[] = "q";
tstring commPortName(com);
Serial serial(commPortName, 57600);
//unsigned char comp[832*832];
unsigned char* iluminationCompR, * iluminationCompG, * iluminationCompB;
Mat grayBackgroundImage;
unsigned long mavis_position = 1000000;
unsigned long mavis_position_enc = 1500;
unsigned long mavis_obj_beginning_position = 0;
unsigned long mavis_obj_beginning_position_past = 1;
unsigned long mavis_obj_end_position = 0;
unsigned long mavis_obj_end_position_past = 1;
unsigned long mavis_obj_position_push = 0;
int mavis_inp = 0;
int in_processing = 0;
int pusher_selection = 0;

int mavis_position_enc_high = 0;
int mavis_position_enc_low = 0;
std::mutex com_lock_dummy; // useful if multiple threads need to communicate with hardware concurrently

unsigned long foto_camera_distance = 3000; // distance from photocell to camera

// ================== function for object classification ==================
string classifyObjectsOnTrack(Mat& img, int min_area, int area_thresh) {
	if (img.empty()) {
		return "Unknown";
	}

	// 1. grayscale
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);

	// 2. blur (reduces noise)
	Mat blur;
	GaussianBlur(gray, blur, Size(5, 5), 0);

	// 3. mask for the track (taking the lower 2/3 of the image as the track, can be adjusted)
	int x1 = 0;
	int y1 = img.rows/ 3; 
	int x2 = img.cols;
	int y2 = img.rows;

	Mat track_mask = Mat::zeros(gray.size(), CV_8U);
	rectangle(track_mask, Point(x1, y1), Point(x2, y2), Scalar(255), FILLED);

	Mat gray_track;
	blur.copyTo(gray_track, track_mask);

	// 4. Otsu threshold
	Mat mask;
	threshold(gray_track, mask, 0, 255, THRESH_BINARY | THRESH_OTSU);

	// 5. Morphological closing
	Mat closed;
	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(12, 12));
	morphologyEx(mask, closed, MORPH_CLOSE, kernel);

	// 6. Finding contours
	vector<vector<Point>> contours;
	findContours(closed, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	for (size_t i = 0; i < contours.size(); i++) {
		double area = contourArea(contours[i]);

		if (area < min_area) continue;

		string category = (area > area_thresh) ? "Big" : "Small";

		// centroid and drawing
		Moments M = moments(contours[i]);
		if (M.m00 > 0) {
			int cx = int(M.m10 / M.m00);
			int cy = int(M.m01 / M.m00);
			drawContours(img, contours, (int)i, Scalar(0, 0, 255), 2);
			putText(img, category, Point(cx - 40, cy),
				FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
		}

		return category;
	}

	return "Unknown";
}

// ================== MAIN PROGRAM ==================
int main(int argc, char** argv)
{
	if (argc != 2)
	{
		cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
		return -1;
	}

	image = imread(argv[1], IMREAD_COLOR); // Read the file

	if (!image.data) // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}
	namedWindow("Display window NASA", WINDOW_AUTOSIZE); // Create a window for display.
	imshow("Display window NASA", image); // Show our image inside it.

	namedWindow("Display window CURRENT", WINDOW_NORMAL); // Create a window for display.
	namedWindow("Display window FINAL", WINDOW_NORMAL); // Create a window for display.

	int exitCode = 0;

	// Before using any pylon methods, the pylon runtime must be initialized. 
	PylonInitialize();

	try
	{
		cout << "Creating Camera..." << endl;
		CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());
		cout << "Camera Created." << endl;
		// Print the model name of the camera.
		cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

		INodeMap& nodemap = camera.GetNodeMap();
		camera.Open();

		// Get camera device information.
		cout << "Camera Device Information" << endl
			<< "=========================" << endl;
		cout << "Vendor           : "
			<< CStringPtr(nodemap.GetNode("DeviceVendorName"))->GetValue() << endl;
		cout << "Model            : "
			<< CStringPtr(nodemap.GetNode("DeviceModelName"))->GetValue() << endl;
		cout << "Firmware version : "
			<< CStringPtr(nodemap.GetNode("DeviceFirmwareVersion"))->GetValue() << endl << endl;

		// Camera settings.
		cout << "Camera Device Settings" << endl
			<< "======================" << endl;

		// GENICAM standard - defining camera parameters
		CIntegerPtr width = nodemap.GetNode("Width");
		CIntegerPtr height = nodemap.GetNode("Height");
		CIntegerPtr offsetX(nodemap.GetNode("OffsetX"));
		CIntegerPtr offsetY(nodemap.GetNode("OffsetY"));
		CEnumerationPtr TriggerMode(nodemap.GetNode("TriggerMode"));
		CEnumerationPtr ExposureMode(nodemap.GetNode("ExposureMode"));
		CFloatPtr ExposureTimeAbs(nodemap.GetNode("ExposureTimeAbs"));

		// setting ROI
		int64_t newWidth = 1600;
		int64_t newHeight = 1200;
		offsetX->SetValue(0);
		offsetY->SetValue(0);
		width->SetValue(newWidth);
		height->SetValue(newHeight);

		TriggerMode->FromString("Off");// trigger is internal
		ExposureMode->FromString("Timed");// trigger is determined by internal time base
		ExposureTimeAbs->SetValue(300);// exposure time

		// parameters determining the acquisition mode in the camera
		camera.MaxNumBuffer = 1;
		camera.OutputQueueSize = 1;
		camera.StartGrabbing(GrabStrategy_UpcomingImage);

		// initialization of object beginning and end
		mavis_obj_beginning_position = SortingLineGetObjectBeginningPosition(&com_lock_dummy);
		mavis_obj_beginning_position_past = mavis_obj_beginning_position;
		mavis_obj_end_position = SortingLineGetObjectEndPosition(&com_lock_dummy);
		mavis_obj_end_position_past = mavis_obj_end_position;
		serial.MavisSendComData(&com_lock_dummy, 23, 1);// INIT - do not touch
		serial.MavisSendComData(&com_lock_dummy, 7, 40);// some sorting line setting - do not touch
		serial.MavisSendComData(&com_lock_dummy, 5, 0);// some sorting line setting - do not touch
		serial.MavisSendComData(&com_lock_dummy, 26, 1);// #0-encoder feedback, 1- servo_feedback - do not touch
		serial.MavisSendComData(&com_lock_dummy, 25, 0);// tolerance - do not touch
		serial.MavisSendComData(&com_lock_dummy, 46, 1);// light on - do not touch
		serial.MavisSendComData(&com_lock_dummy, 13, 600);// pusher time - do not touch
		serial.MavisSendComData(&com_lock_dummy, 23, 10);// GO - do not touch 

		while (camera.IsGrabbing())
		{
			mavis_position_enc = SortingLineGetCurrentPosition(&com_lock_dummy);// reading current belt position

			// processing photocell signals indicating object presence
			if (scanning_status == OUT_OF_OBJECT) {
				mavis_obj_beginning_position = SortingLineGetObjectBeginningPosition(&com_lock_dummy);
				if (mavis_obj_beginning_position != mavis_obj_beginning_position_past) {
					// beginning of a new object
					scanning_status = IN_OBJECT;
					mavis_obj_beginning_position_past = mavis_obj_beginning_position;
					cout << endl << "OBJECT BEGINNING AT " << mavis_obj_beginning_position << endl;
				}
			}
			else {// IN_OBJECT
				mavis_obj_end_position = SortingLineGetObjectEndPosition(&com_lock_dummy);
				if (mavis_obj_end_position != mavis_obj_end_position_past) {
					// end of a new object
					scanning_status = OUT_OF_OBJECT;
					mavis_obj_end_position_past = mavis_obj_end_position;
					SortingLineObject new_object;
					new_object.beginning_position = mavis_obj_beginning_position;
					new_object.end_position = mavis_obj_end_position;
					objects_queue.push(new_object);
					cout << endl << "OBJECT END AT " << mavis_obj_end_position << endl;
				}
			}

			// if an object exists, position the camera to wait
			if (objects_queue.size() > 0) {
				SortingLineObject new_object;
				new_object = objects_queue.front();
				mavis_position = ((new_object.beginning_position + new_object.end_position) / 2) + foto_camera_distance;
				acquisition_status = WAITING_ON_OBJECT;
				cout << "W" << mavis_position << endl;
			}
			else {
				mavis_position = mavis_position_enc + 500;
				cout << ".";
			}

			SortingLineSetPosition(&com_lock_dummy, mavis_position);

			if (acquisition_status == WAITING_ON_OBJECT) {
				if (SortingLineGetInPositionStatus(&com_lock_dummy) == 1) {
					cout << endl << "IMAGE ACQUISITION AT " << endl;
					AcquireImage(&camera);
					SortingLineObject new_object;
					new_object = objects_queue.front();
					mavis_obj_position_push = (new_object.beginning_position + new_object.end_position) / 2;

					// ================== pusher selection based on classification ==================
					// pusher_selection is already set inside AcquireImage based on object class

					switch (pusher_selection) {
					case 0:
						SortingLineSetPositionPusher1(mavis_obj_position_push);
						break;
					case 1:
						SortingLineSetPositionPusher2(mavis_obj_position_push);
						break;
					case 2:
						SortingLineSetPositionPusher3(mavis_obj_position_push);
						break;
					}
					objects_queue.pop();
					acquisition_status = IDLE;
				}
			}
			if (waitKey(30) >= 0) break;
		}
		camera.StopGrabbing();
	}
	catch (const GenericException& e)
	{
		cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
		exitCode = 1;
	}
	PylonTerminate();
	return exitCode;
}


// ================== ACQUISITION FUNCTION ==================
int AcquireImage(Camera_t* camera) {
	CGrabResultPtr ptrGrabResult;
	CPylonImage imagePylonTemp;
	static Mat rgbImage, grayImage;
	Mat rgbImageWithoutBackground;
	static Mat rgbFinalImage, grayFinalImage, absdifference, difference, rgbImageWithBackground, grayImageWithBackground;
	CImageFormatConverter fc;
	fc.OutputPixelFormat = PixelType_BGR8packed;
	double t = (double)getTickCount();
	camera->RetrieveResult(3000, ptrGrabResult, TimeoutHandling_ThrowException);

	if (ptrGrabResult->GrabSucceeded())
	{
		fc.Convert(imagePylonTemp, ptrGrabResult);
		rgbImageWithBackground = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)imagePylonTemp.GetBuffer());

		// ================== object classification ==================
		string result = classifyObjectsOnTrack(rgbImageWithBackground, 20000, 200000);

		if (result == "Big") {
			pusher_selection = 2; // big goes to pusher 3
		}
		else if (result == "Small") {
			pusher_selection = 1; // small goes to pusher 2
		}
		else {
			pusher_selection = 0; // unknown goes to pusher 1
		}

		cout << "Detected object: " << result << " -> pusher_selection = " << pusher_selection << endl;

		imshow("Display window FINAL", rgbImageWithBackground);
	}
	else
	{
		cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
	}
	return 0;
}


// ================== OTHER FUNCTIONS ==================
int SortingLineSetPosition(std::mutex* comm_lock, unsigned long position) {
	serial.MavisSendComData(comm_lock, 30, (mavis_position >> 16) & 0xFFFF);// command that sets the higher 16 bits of the 32-bit target position
	serial.MavisSendComData(comm_lock, 31, mavis_position & 0xFFFF);// command that sets the lower 16 bits of the 32-bit target position
	return 0;
}

unsigned long SortingLineGetCurrentPosition(std::mutex* comm_lock) {
	unsigned long mavis_position_enc;
	int mavis_position_enc_high, mavis_position_enc_low;
	int retval1, retval2;
	retval1 = serial.MavisGetComData(comm_lock, 30, &mavis_position_enc_high);// command that reads the higher 16 bits of the 32-bit current position
	retval2 = serial.MavisGetComData(comm_lock, 31, &mavis_position_enc_low);// command that reads the lower 16 bits of the 32-bit current position
	mavis_position_enc = (mavis_position_enc_high << 16) | mavis_position_enc_low;// current 32-bit belt position
	return mavis_position_enc;
}

unsigned long SortingLineGetObjectBeginningPosition(std::mutex* comm_lock) {
	unsigned long mavis_position_obj_start;
	int mavis_position_enc_high, mavis_position_enc_low;
	serial.MavisGetComData(comm_lock, 34, &mavis_position_enc_high);// command that reads the higher 16 bits of the 32-bit object beginning position detected by photocell
	serial.MavisGetComData(comm_lock, 35, &mavis_position_enc_low);// command that reads the lower 16 bits of the 32-bit object beginning position detected by photocell
	mavis_position_obj_start = (mavis_position_enc_high << 16) | mavis_position_enc_low;// current 32-bit object beginning position
	return mavis_position_obj_start;
}

unsigned long SortingLineGetObjectEndPosition(std::mutex* comm_lock) {
	unsigned long mavis_position_obj_stop;
	int mavis_position_enc_high, mavis_position_enc_low;
	serial.MavisGetComData(comm_lock, 36, &mavis_position_enc_high);// command that reads the higher 16 bits of the 32-bit object end position detected by photocell
	serial.MavisGetComData(comm_lock, 37, &mavis_position_enc_low);// command that reads the lower 16 bits of the 32-bit object end position detected by photocell
	mavis_position_obj_stop = (mavis_position_enc_high << 16) | mavis_position_enc_low;// current 32-bit object end position
	return mavis_position_obj_stop;
}

int SortingLineGetInPositionStatus(std::mutex* comm_lock) {
	int mavis_inp;
	serial.MavisGetComData(comm_lock, 32, &mavis_inp);
	return mavis_inp;
}

int SortingLineSetPositionPusher1(unsigned long mavis_position_obj_push) {
	mavis_position_obj_push = mavis_position_obj_push + 7900;
	serial.MavisSendComData(&com_lock_dummy, 40, (mavis_position_obj_push >> 16) & 0xFFFF);// 
	serial.MavisSendComData(&com_lock_dummy, 41, mavis_position_obj_push & 0xFFFF);// 
	return 0;
}

int SortingLineSetPositionPusher2(unsigned long mavis_position_obj_push) {
	mavis_position_obj_push = mavis_position_obj_push + 12000;
	serial.MavisSendComData(&com_lock_dummy, 42, (mavis_position_obj_push >> 16) & 0xFFFF);// 
	serial.MavisSendComData(&com_lock_dummy, 43, mavis_position_obj_push & 0xFFFF);// 
	return 0;
}

int SortingLineSetPositionPusher3(unsigned long mavis_position_obj_push) {
	mavis_position_obj_push = mavis_position_obj_push + 14500;
	serial.MavisSendComData(&com_lock_dummy, 44, (mavis_position_obj_push >> 16) & 0xFFFF);// 
	serial.MavisSendComData(&com_lock_dummy, 45, mavis_position_obj_push & 0xFFFF);// 
	return 0;
}
