#include"header_Final_V1.h"

cv::VideoCapture cap;

/*Matrix used for image analyzing*/
cv::Mat frame_original;
cv::Mat frame_display;

cv::Mat frame_ROI_LEFT;
cv::Mat frame_ROI_RIGHT;

cv::Mat frame_threshold_Left;
cv::Mat frame_threshold_Right;

cv::Mat frame_Acc_Masked_Left;
cv::Mat frame_Acc_Masked_Right;

cv::Mat mask_Left;
cv::Mat mask_Right;

cv::Rect ROI_LEFT; // ROI = Region of Interest
cv::Rect ROI_RIGHT;
/*Matrix used for image analyzing*/





int main() {

	cap.open("File Directory");
	//cap.open(1);

	if (!cap.isOpened()) {
		cout << "Error opening video stream or file" << endl;
		return -1;
	}

	/*
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	cap.set(CV_CAP_PROP_FPS, 30);
	cap.set(CV_CAP_PROP_BRIGHTNESS, 100);
	cap.set(CV_CAP_PROP_HUE, 75);
	*/

	/*Initialize Struct and Variables*/
	image_column = cap.get(CAP_PROP_FRAME_WIDTH);
	image_row = cap.get(CAP_PROP_FRAME_HEIGHT);
	image_midline = image_column / 2;

	ROI_LEFT = cv::Rect(0, 0, image_midline, image_row);
	mask_Left = cv::Mat(ROI_LEFT.size(), CV_8U, GrayScale_Channel_Black);
	frame_ROI_LEFT = cv::Mat(ROI_LEFT.size(), CV_8UC3, RGB_Channel_Black);
	frame_Acc_Masked_Left = cv::Mat(ROI_LEFT.size(), CV_8U, GrayScale_Channel_Black);
	Previous_Frames	Old_Frame_Data_Left = {
		cv::Mat(ROI_LEFT.size(), CV_8U, GrayScale_Channel_Black),
		cv::Mat(ROI_LEFT.size(), CV_8U, GrayScale_Channel_Black),
		cv::Mat(ROI_LEFT.size(), CV_8U, GrayScale_Channel_Black),
		cv::Mat(ROI_LEFT.size(), CV_8U, GrayScale_Channel_Black),
		cv::Mat(ROI_LEFT.size(), CV_8U, GrayScale_Channel_Black)
	};
	LinearRegression_Element_NotInitialized	Regression_Element_Left = { { Point2f(0, 0) }, 0, 0, false, true };
	LinearRegression_Element_OLS	Regression_Ele_OLS_Left = { 0, 0, 0, 0, 0 };
	LinearRegression_Element_Robust Regression_Ele_Robust_Left = { { Point(0, 0) }, 0, 0, 0, 0, 0, 0 };
	LinearRegression_Result_OneSide	Regression_Results_Left = { 0, 0, false, true };
	Elements_Lane_Coordinate		LaneDrawing_Ele_Left = { Point(0, 0), Point(0, 0) };
	Elements_Masking_Coordinate		MaskShaping_Ele_Left = { Point(0, 0), Point(0, 0), Point(0, 0), Point(0, 0) };

	ROI_RIGHT = cv::Rect(image_midline, 0, image_midline, image_row);
	mask_Right = cv::Mat(ROI_RIGHT.size(), CV_8U, GrayScale_Channel_Black);
	frame_ROI_RIGHT = cv::Mat(ROI_RIGHT.size(), CV_8UC3, RGB_Channel_Black);
	frame_Acc_Masked_Right = cv::Mat(ROI_RIGHT.size(), CV_8U, GrayScale_Channel_Black);
	Previous_Frames	Old_Frame_Data_Right = {
		cv::Mat(ROI_RIGHT.size(), CV_8U, GrayScale_Channel_Black),
		cv::Mat(ROI_RIGHT.size(), CV_8U, GrayScale_Channel_Black),
		cv::Mat(ROI_RIGHT.size(), CV_8U, GrayScale_Channel_Black),
		cv::Mat(ROI_RIGHT.size(), CV_8U, GrayScale_Channel_Black),
		cv::Mat(ROI_RIGHT.size(), CV_8U, GrayScale_Channel_Black)
	};
	LinearRegression_Element_NotInitialized	Regression_Element_Right = { { Point2f(0, 0) }, 0, 0, false, true };
	LinearRegression_Element_OLS	Regression_Ele_OLS_Right = { 0, 0, 0, 0, 0 };
	LinearRegression_Element_Robust Regression_Ele_Robust_Right = { { Point(0, 0) }, 0, 0, 0, 0, 0, 0 };
	LinearRegression_Result_OneSide	Regression_Results_Right = { 0, 0, false, false };
	Elements_Lane_Coordinate		LaneDrawing_Ele_Right = { Point(0, 0), Point(0, 0) };
	Elements_Masking_Coordinate		MaskShaping_Ele_Right = { Point(0, 0), Point(0, 0), Point(0, 0), Point(0, 0) };

	Elements_Trajectory_Coordinate Trajectory_Drawing_Ele = { Point(0, 0), Point(0, 0), 0 };
	Elements_Line_Specification Drawing_Specification = { 25, RGB_Channel_Yellow, 15,  RGB_Channel_Sky_Blue};
	/*Initialize Struct Variables*/


	/*INIT MODE*/
	init_Mode(
		&cap, &ROI_LEFT, &ROI_RIGHT,
		&Value_Threshold, &Value_Background,
		&Regression_Results_Left, &MaskShaping_Ele_Left,
		&Regression_Results_Right, &MaskShaping_Ele_Right
	);
	Mask_Shaper_Parallel(&MaskShaping_Ele_Left, &mask_Left);
	Mask_Shaper_Parallel(&MaskShaping_Ele_Right, &mask_Right);
	/*INIT MODE*/


	cv::namedWindow("Result_Image", CV_WINDOW_NORMAL);
	cv::resizeWindow("Result_Image", image_column, image_row);

	cv::namedWindow("Left_Mask", CV_WINDOW_NORMAL);
	cv::resizeWindow("Left_Mask", image_column / 4, image_row / 4);
	cv::namedWindow("Right_Mask", CV_WINDOW_NORMAL);
	cv::resizeWindow("Right_Mask", image_column / 4, image_row / 4);
	cv::namedWindow("Left", CV_WINDOW_NORMAL);
	cv::resizeWindow("Left", image_column / 4, image_row / 4);
	cv::namedWindow("Right", CV_WINDOW_NORMAL);
	cv::resizeWindow("Right", image_column / 4, image_row / 4);

	omp_set_num_threads(2);
	
	while (1){
		
		cap.read(frame_original);		
		
		#pragma omp parallel
		{
			int threadID = omp_get_thread_num();

			if (threadID == 0) {
				frame_ROI_LEFT = frame_original(ROI_LEFT);		
				image_Convertor(&frame_ROI_LEFT, &frame_threshold_Left);
				frame_Accumulator(&Old_Frame_Data_Left,	&frame_threshold_Left, &mask_Left, &frame_Acc_Masked_Left);
				Buoy_Detector_Parallel(&frame_Acc_Masked_Left, &Regression_Element_Left);
				Robust_Regression_Ele_Init_Parallel(&Regression_Element_Left, &Regression_Results_Left, &Regression_Ele_Robust_Left);
				Robust_Regression_Cal_Parallel(&Regression_Ele_Robust_Left, &Regression_Results_Left);

				# pragma omp barrier
				XY_Coordinate_Cal_Trajectory_Parallel(&Regression_Results_Left, &Regression_Results_Right, &curr_height, &Trajectory_Drawing_Ele);
				# pragma omp barrier
				
				XY_Coordinate_Cal_Buoy_Lane_Parallel(&Regression_Results_Left, &curr_height, &LaneDrawing_Ele_Left);
				XY_Coordinate_Cal_Mask_Parallel(&LaneDrawing_Ele_Left, &Regression_Element_Left, &Regression_Results_Left, &MaskShaping_Ele_Left);
				Mask_Shaper_Parallel(&MaskShaping_Ele_Left, &mask_Left);

			}else{
				frame_ROI_RIGHT = frame_original(ROI_RIGHT);			
				image_Convertor(&frame_ROI_RIGHT, &frame_threshold_Right);
				frame_Accumulator(&Old_Frame_Data_Right, &frame_threshold_Right, &mask_Right, &frame_Acc_Masked_Right);
				Buoy_Detector_Parallel(&frame_Acc_Masked_Right, &Regression_Element_Right);
				Robust_Regression_Ele_Init_Parallel(&Regression_Element_Right, &Regression_Results_Right, &Regression_Ele_Robust_Right);
				Robust_Regression_Cal_Parallel(&Regression_Ele_Robust_Right, &Regression_Results_Right);
			
				# pragma omp barrier
				
				# pragma omp barrier

				XY_Coordinate_Cal_Buoy_Lane_Parallel(&Regression_Results_Right, &curr_height, &LaneDrawing_Ele_Right);
				XY_Coordinate_Cal_Mask_Parallel(&LaneDrawing_Ele_Right, &Regression_Element_Right, &Regression_Results_Right, &MaskShaping_Ele_Right);
				Mask_Shaper_Parallel(&MaskShaping_Ele_Right, &mask_Right);
			}
		}

		/*
		wait for both section done
		set frameROI as golbal
		*/
		Line_Drawer(
			&Drawing_Specification, 
			&Regression_Element_Left, &Regression_Element_Right,
			&LaneDrawing_Ele_Left, &LaneDrawing_Ele_Right, 
			&Trajectory_Drawing_Ele, 
			&frame_ROI_LEFT, &frame_ROI_RIGHT, 
			&frame_display);
		
		cv::imshow("Result_Image", frame_display);

		cv::imshow("Left_Mask", mask_Left);
		cv::imshow("Right_Mask", mask_Right);
		cv::imshow("Left", frame_Acc_Masked_Left);
		cv::imshow("Right", frame_Acc_Masked_Right);

		char c = (char)waitKey(25);
		if (c == 27) {
			break;
		}
	}

	cap.release();

	cv::destroyAllWindows();

	return 0;
}