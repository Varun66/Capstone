#ifndef _HEADER_FINAL_V1_H_
#define _HEADER_FINAL_V1_H_

#include<opencv2/opencv.hpp>
#include<iostream>
#include<fstream>
#include <string>
#include <iomanip>
#include<algorithm> // added for sort function
#include<omp.h>




/*Defined for Indentiying the type of Variable used in fuction*/
#define INPUT // input for the function
#define OUTPUT // output for the function
#define InOut_Put // works as input and output at same time
/*Defined for Indentiying the type of Variable used in fuction*/

/*Pre-Defined constatns*/
#define Size_Array 20
/*Pre-Defined constatns*/

/*Pre-Defined Colour Set*/
#define GrayScale_Channel_Black Scalar(0)
#define GrayScale_Channel_White  Scalar(255)

#define RGB_Channel_Black Scalar(0, 0, 0)
#define RGB_Channel_White Scalar(255, 255, 255)

#define RGB_Channel_Red Scalar(0, 0, 255)
#define RGB_Channel_Green Scalar(0, 255, 0)
#define RGB_Channel_Blue Scalar(255, 0, 0)
#define RGB_Channel_Sky_Blue Scalar(235, 206, 135)

#define RGB_Channel_Yellow Scalar(0, 255, 255)
#define RGB_Channel_CherryRed Scalar(255, 0, 255)
#define RGB_Channel_BlueGreen Scalar(255, 255, 0)
/*Pre-Defined Colour Set*/



/*NameSpace required by the software*/
using namespace cv;
using namespace std;
/*NameSpace required by the software*/



/*Variable used for image analizing*/
extern int image_row;
extern int image_column;
extern int image_midline;
extern int curr_height;

extern int Value_Threshold;
extern int Value_Background;
/*Variable used for image analizing*/


/*Pre-Defined Struct Set*/
typedef struct LinearRegression_Element_NotInitialized {
	cv::Point2f XY_Coordinate[Size_Array];
	int Num_identified_Buoy;
	int Array_Position_Counter;
	bool Array_Full;
	bool Not_Enough_Buoy;
} LinearRegression_Element_NotInitialized;

typedef struct LinearRegression_Element_OLS {
	int Num_identified_Buoy;
	double x_Sum;
	double xx_Sum;
	double y_Sum;
	double xy_Sum;
} LinearRegression_Element_OLS;

typedef struct LinearRegression_Element_Robust {
	cv::Point XY_Coordinate_Data[Size_Array];
	int Num_of_Collected_Data;
	double x_mean;
	double init_weight[Size_Array];
	double init_X_weight;
	double init_Y_weight;
	double xx_Sum;
} LinearRegression_Element_Robust;

typedef struct LinearRegression_Result_OneSide {
	double m_slope;
	double b_intercept;
	bool e_error;
	bool LEFT_OR_RIGHT; // left for TRUE, right for FALSE
} LinearRegression_Result_OneSide;

typedef struct Elements_Line_Specification {
	int Line_Thickness_Buoy_Lane;
	Scalar Line_Colour_Buoy_Lane;
	int Line_Thickness_Trajectory_Line;	
	Scalar Line_Colour_Trajectory_Line;
}  Elements_Line_Specification;

typedef struct Elements_Lane_Coordinate {
	cv::Point Lane_Coordinate_TOP;
	cv::Point Lane_Coordinate_BOT;
} Elements_Lane_Coordinate;

typedef struct Elements_Trajectory_Coordinate {
	cv::Point Trajectory_Coordinate_TOP;
	cv::Point Trajectory_Coordinate_BOT;
	double AngleToTurn;
} Elements_Trajectory_Coordinate;

typedef struct Elements_Masking_Coordinate {
	cv::Point Masking_Coordinate_One;
	cv::Point Masking_Coordinate_Two;
	cv::Point Masking_Coordinate_Three;
	cv::Point Masking_Coordinate_Four;
} Elements_Masking_Coordinate;

typedef struct Previous_Frames {
	cv::Mat frame_Previous_1;
	cv::Mat frame_Previous_2;
	cv::Mat frame_Previous_3;
	cv::Mat frame_Previous_4;
	cv::Mat frame_Previous_5;
} Previous_Frames;

typedef struct INIT_MOD_ONLY {
	bool NewClick;
	int Picked_height;
	cv::Point XY_Coordinate_Clicked;
}  INIT_MOD_ONLY;
/*Pre-Defined Struct Set*/






/*----------------------------The functions of "process_image.cpp"----------------------------*/
void image_Convertor(
	INPUT	cv::Mat *inputFrame,
	OUTPUT	cv::Mat *outputFrame
);

void frame_Accumulator(
	INPUT Previous_Frames * Old_Frames,
	INPUT cv::Mat *input_Frame,
	INPUT cv::Mat *input_Mask,
	OUTPUT cv::Mat *outputFrame_Masked_Accumulated
);

void Line_Drawer(
	INPUT Elements_Line_Specification * Input_Specification,
	INPUT LinearRegression_Element_NotInitialized * Input_Buoy_Num_Left,
	INPUT LinearRegression_Element_NotInitialized * Input_Buoy_Num_Right,
	INPUT Elements_Lane_Coordinate * Input_Lane_Left,
	INPUT Elements_Lane_Coordinate * Input_Lane_Right,
	INPUT Elements_Trajectory_Coordinate * Input_Trajectory,
	INPUT cv::Mat * inputFrame_LEFT,
	INPUT cv::Mat * inputFrame_RIGHT,
	InOut_Put cv::Mat * OutputFrame_Display
);

/*----------------------------The functions of "process_image.cpp"----------------------------*/



/*----------------------------The functions of "process_regression.cpp"----------------------------*/
/*----------------------------Buoy Detecting Method----------------------------*/
void Buoy_Detector_Parallel(
	INPUT cv::Mat *inputFrame,
	OUTPUT LinearRegression_Element_NotInitialized * Output
);

/*----------------------------Buoy Detecting Method----------------------------*/

/*----------------------------OLS Method----------------------------*/
void OLS_Regression_Ele_Init_MomentBased_Parallel(
	INPUT LinearRegression_Element_NotInitialized * Input,
	OUTPUT LinearRegression_Element_OLS * Output
);

void OLS_Regression_Cal_Parallel(
	INPUT LinearRegression_Element_OLS * Input,
	OUTPUT LinearRegression_Result_OneSide * Output
);

/*----------------------------OLS Method----------------------------*/


/*----------------------------Robust Method----------------------------*/
void Robust_Regression_Ele_Init_Parallel(
	INPUT	LinearRegression_Element_NotInitialized	* Input_One,
	INPUT	LinearRegression_Result_OneSide			* Input_Two,
	OUTPUT	LinearRegression_Element_Robust			* Output
);

void Robust_Regression_Cal_Parallel(
	INPUT LinearRegression_Element_Robust * Input,
	OUTPUT LinearRegression_Result_OneSide * Output
);

/*----------------------------Robust Method----------------------------*/


/*----------------------------XY - Coordinate Calculation----------------------------*/
void XY_Coordinate_Cal_Trajectory_Parallel(
	INPUT LinearRegression_Result_OneSide * Input_Left,
	INPUT LinearRegression_Result_OneSide * Input_Right,
	OUTPUT int * calculated_height,
	OUTPUT Elements_Trajectory_Coordinate * tempOutput
);

void XY_Coordinate_Cal_Buoy_Lane_Parallel(
	INPUT LinearRegression_Result_OneSide * Input,
	INPUT int *currHeight,
	OUTPUT Elements_Lane_Coordinate * Output
);

/*----------------------------XY - Coordinate Calculation----------------------------*/





/*----------------------------The functions of "process_masking.cpp"----------------------------*/
void XY_Coordinate_Cal_Mask_Parallel(
	INPUT Elements_Lane_Coordinate * InOne,
	INPUT LinearRegression_Element_NotInitialized * InTwo,
	INPUT LinearRegression_Result_OneSide * In_Three,
	OUTPUT Elements_Masking_Coordinate * Output
);

void Mask_Shaper_Parallel(
	INPUT Elements_Masking_Coordinate * Input,
	InOut_Put cv::Mat * Mask
);

/*----------------------------The functions of "process_masking.cpp"----------------------------*/



/*----------------------------The functions of "function_initialize_Parallel_V0.cpp"----------------------------*/
void CallBackFunc(
	int event,
	int x,
	int y,
	int flags,
	void * userData
);

void initializer_IMG_CVT_VALUE(
	INPUT VideoCapture * inputVideo,
	OUTPUT int * Value_BackGround,
	OUTPUT int * Value_Threshold
);

void initializer_Masking_and_Regression(
	INPUT Mat * inFrame,
	OUTPUT LinearRegression_Result_OneSide * tempRegression,
	OUTPUT Elements_Masking_Coordinate * tempMask
);

void initializer_initialized_Vision(
	INPUT Elements_Masking_Coordinate * Init_Mask,
	INPUT Elements_Lane_Coordinate * Init_Lane,
	InOut_Put Mat * InOut_Frame
);

void XY_Coordinate_Cal_Mask_Parallel_INIT_MOD(
	INPUT Elements_Lane_Coordinate * InOne,
	INPUT int * InTWO_NUM_IDENTIFIED,
	INPUT LinearRegression_Result_OneSide * In_Three,
	OUTPUT Elements_Masking_Coordinate * Output
);

void init_image_Processing_Val_Saving(
	INPUT int * Val_Threshold,
	INPUT int * Val_Backgrond
);

void init_image_Processing_Val_Reading(
	OUTPUT int * Val_Threshold,
	OUTPUT int * Val_Backgrond
);

void init_Regression_Result_Saving(
	INPUT LinearRegression_Result_OneSide * regression_Left,
	INPUT LinearRegression_Result_OneSide * regression_Right
);

void init_Regression_Result_Reading(
	OUTPUT LinearRegression_Result_OneSide * regression_Left,
	OUTPUT LinearRegression_Result_OneSide * regression_Right
);

void init_Masking_Coordinate_Saving(
	INPUT Elements_Masking_Coordinate * masking_Left,
	INPUT Elements_Masking_Coordinate * masking_Right
);

void init_Masking_Coordinate_Reading(
	OUTPUT Elements_Masking_Coordinate * masking_Left,
	OUTPUT Elements_Masking_Coordinate * masking_Right
);

void init_Mode(
	INPUT VideoCapture * inputVideo,
	INPUT Rect * region_of_Interest_LEFT,
	INPUT Rect * region_of_Interest_Right,
	OUTPUT int * Val_Threshold,
	OUTPUT int * Val_Backgounrd,
	OUTPUT LinearRegression_Result_OneSide * regression_RLeft,
	OUTPUT Elements_Masking_Coordinate * XY_Coordinate_MLeft,
	OUTPUT LinearRegression_Result_OneSide * regression_RRIGHT,
	OUTPUT Elements_Masking_Coordinate * XY_Coordinate_MRIght
);
#endif