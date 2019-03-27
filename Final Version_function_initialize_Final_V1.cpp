#include"header_Final_V1.h"

/*
This file includes functions which allows the user to initialize pre-set value.
The functions in this file are built for demo, that the system is designed to be used under specific condtion;
however, demo cannot be done at the specific condition.
Therefore, the functions in this file allows pre-set valuse can be dynamically assigned mannually.
*/

void CallBackFunc(int event, int x, int y, int flags, void*userData) {

	INIT_MOD_ONLY * dataP = (INIT_MOD_ONLY *)userData;

	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		dataP->XY_Coordinate_Clicked.x = x;
		dataP->XY_Coordinate_Clicked.y = y;
		dataP->NewClick = true;
		cout << "NewClick?: " << dataP->NewClick << endl;
	}
	else if (event == EVENT_RBUTTONDOWN)
	{
		cout << "Right button of the mouse is clicked - Picked Height: " << y << endl;
		dataP->Picked_height = y;
	}
}



/*
This function allow the user can modifies the following values
- Theshold Value
=> This value filter out outlier based on their colour value
- Background Value
=> This value filter out outlier by substracting  average background value.
*/
void initializer_IMG_CVT_VALUE(
	INPUT VideoCapture * inputVideo,
	OUTPUT int * Value_BackGround,
	OUTPUT int * Value_Threshold
) {

	cv::Mat tempVdieo;
	cv::Mat tempGray;
	cv::Mat tempGray_BGR;
	cv::Mat tempDisplay;
	cv::Mat tempRed;
	cv::Mat tempBlue;
	cv::Mat tempBackGround(image_row, image_column, CV_8UC1, GrayScale_Channel_Black);

	cv::namedWindow("Control_Pannel", CV_WINDOW_NORMAL);
	cv::resizeWindow("Control_Pannel", 600, 200);

	int control_Threshold;
	cvCreateTrackbar("Threshold", "Control_Pannel", &control_Threshold, 255); //Value (0 - 255)
	setTrackbarPos("Threshold", "Control_Pannel", 20);

	int control_Background;
	cvCreateTrackbar("Background", "Control_Pannel", &control_Background, 255); //Value (0 - 255)
	setTrackbarPos("Background", "Control_Pannel", 100);

	cv::namedWindow("Filtered_Condition", CV_WINDOW_NORMAL);
	cv::resizeWindow("Filtered_Condition", 1200, 450);

	while (1) {
		char c = (char)waitKey(25);
		if (c == 27) {
			*Value_Threshold = control_Threshold;
			*Value_BackGround = control_Background;

			cv::destroyAllWindows();

			c = 0;

			break;
		}

		inputVideo->read(tempVdieo);

		control_Threshold = getTrackbarPos("Threshold", "Control_Pannel");
		control_Background = getTrackbarPos("Background", "Control_Pannel");
		tempBackGround.setTo(Scalar(control_Background));

		if (!tempVdieo.empty()) {

			cv::extractChannel(tempVdieo, tempRed, 2);
			cv::extractChannel(tempVdieo, tempBlue, 0);

			tempGray = tempRed - tempBlue;

			tempGray = tempGray - tempBackGround;

			cv::threshold(tempGray, tempGray, control_Threshold, 255, THRESH_BINARY);

			cv::cvtColor(tempGray, tempGray_BGR, COLOR_GRAY2BGR);

			cv::hconcat(tempVdieo, tempGray_BGR, tempDisplay);

			cv::imshow("Filtered_Condition", tempDisplay);
		}
	}
}


/*
initializer_Masking_and_Regression() function
This function used to initialize data required to perform robust regression at the real-time phase
OLS Regression method is used, since the user picks the input data
*/
void initializer_Masking_and_Regression(
	INPUT Mat * inFrame,
	OUTPUT LinearRegression_Result_OneSide * tempRegression,
	OUTPUT Elements_Masking_Coordinate * tempMask
) {
	cv::Mat tempFrame_Ori;	// For original input image
	cv::Mat tempFrame_Modi; // For displaying image with masking area and picked lane

	INIT_MOD_ONLY					init_MOD_ELE = { false, 0, Point(0, 0) };
	LinearRegression_Element_OLS	tempRegression_ELE = { 0, 0, 0, 0, 0 };
	LinearRegression_Result_OneSide	tempRegression_RESULT = *tempRegression;
	Elements_Lane_Coordinate		tempLaneDrawing_ELE = { Point(0, 0), Point(0, 0) };
	Elements_Masking_Coordinate		tempMasking_Coordinate = { Point(0, 0), Point(0, 0), Point(0, 0), Point(0, 0) };

	namedWindow("Mask_Initializer", CV_WINDOW_NORMAL);
	resizeWindow("Mask_Initializer", inFrame->rows, inFrame->cols);
	setMouseCallback("Mask_Initializer", CallBackFunc, &init_MOD_ELE);

	inFrame->copyTo(tempFrame_Modi);

	while (1) {
		char c = (char)waitKey(25);
		if (c == 27) {
			*tempRegression = tempRegression_RESULT;
			*tempMask = tempMasking_Coordinate;

			tempFrame_Ori.release();
			tempFrame_Modi.release();
			cv::destroyAllWindows();

			c = 0;

			break;
		}

		imshow("Mask_Initializer", tempFrame_Modi);

		if (init_MOD_ELE.NewClick == true) {
			init_MOD_ELE.NewClick = false;
			tempRegression_ELE.Num_identified_Buoy = tempRegression_ELE.Num_identified_Buoy + 1;
			tempRegression_ELE.x_Sum = tempRegression_ELE.x_Sum + init_MOD_ELE.XY_Coordinate_Clicked.x;
			tempRegression_ELE.y_Sum = tempRegression_ELE.y_Sum + init_MOD_ELE.XY_Coordinate_Clicked.y;
			tempRegression_ELE.xx_Sum = tempRegression_ELE.xx_Sum + pow(init_MOD_ELE.XY_Coordinate_Clicked.x, 2);
			tempRegression_ELE.xy_Sum = tempRegression_ELE.xy_Sum + (init_MOD_ELE.XY_Coordinate_Clicked.x * init_MOD_ELE.XY_Coordinate_Clicked.y);


			if (tempRegression_ELE.Num_identified_Buoy > 1) {
				OLS_Regression_Cal_Parallel(&tempRegression_ELE, &tempRegression_RESULT);
				XY_Coordinate_Cal_Buoy_Lane_Parallel(&tempRegression_RESULT, &init_MOD_ELE.Picked_height, &tempLaneDrawing_ELE);
				XY_Coordinate_Cal_Mask_Parallel_INIT_MOD(&tempLaneDrawing_ELE, &tempRegression_ELE.Num_identified_Buoy, &tempRegression_RESULT, &tempMasking_Coordinate);

				inFrame->copyTo(tempFrame_Modi);

				cout << tempRegression_RESULT.m_slope << endl;
				cout << tempRegression_RESULT.b_intercept << endl;
			}
		}
		initializer_initialized_Vision(&tempMasking_Coordinate, &tempLaneDrawing_ELE, &tempFrame_Modi);
	}
}


/*
initializer_initialized_Vision() function
This function provides visual output of current initialzation state.
It is possible to see regression result based on the current input, and the masking area generated by it.
*/
void initializer_initialized_Vision(
	INPUT Elements_Masking_Coordinate * Init_Mask,
	INPUT Elements_Lane_Coordinate * Init_Lane,
	InOut_Put Mat * InOut_Frame
) {
	/*
	For initialized Lane
	*/
	cv::line(*InOut_Frame, Init_Lane->Lane_Coordinate_TOP, Init_Lane->Lane_Coordinate_BOT, RGB_Channel_Red, 4, LINE_AA);

	/*
	For initialized Masking Area
	*/
	vector< Point> Masking_Cooridinate;
	Masking_Cooridinate.push_back(Init_Mask->Masking_Coordinate_One);
	Masking_Cooridinate.push_back(Init_Mask->Masking_Coordinate_Two);
	Masking_Cooridinate.push_back(Init_Mask->Masking_Coordinate_Three);
	Masking_Cooridinate.push_back(Init_Mask->Masking_Coordinate_Four);

	cv::polylines(*InOut_Frame, Masking_Cooridinate, true, RGB_Channel_Yellow, 5, LINE_AA);
}



/*
SPECIAL FOR DEMO
XY_Coordinate_Cal_Mask_Parallel_INIT_MOD() fucntion
This function is special for the initialization mode, but its function is same as 'XY_Coordinate_Cal_Mask_Parallel()' function.
There is minor parameter differences.
*/
void XY_Coordinate_Cal_Mask_Parallel_INIT_MOD(
	INPUT Elements_Lane_Coordinate * InOne,
	INPUT int * InTWO_NUM_IDENTIFIED,
	INPUT LinearRegression_Result_OneSide * In_Three,
	OUTPUT Elements_Masking_Coordinate * Output
) {
	/*
	The following section checks the number of identified buoy, and define radius to build masking area
	This allows the masking size of masking area can be vary based on number of defined buoy.
	*/
	int radius_Top;
	int radius_Bottom;

	if (*InTWO_NUM_IDENTIFIED > 2) {
		radius_Top = 20;
		radius_Bottom = 40;
	}
	else if (*InTWO_NUM_IDENTIFIED == 2) {
		radius_Top = 25;
		radius_Bottom = 50;
	}
	else if (*InTWO_NUM_IDENTIFIED == 1) {
		radius_Top = 30;
		radius_Bottom = 60;
	}
	else {
		radius_Top = 35;
		radius_Bottom = 70;
	}


	/*
	The following section performs required calculation to build the masking area for left side.

	m_slope_Modi = - 1.0 / m_slope_Ori;
	b_intercept_Modi_Top = y_Top - m_slope_Modi * x_Top;
	b_intercept_Modi_Bottom = y_Bottom - m_slope_Modi * x_Bottom;

	quadratic_formula_a_Top = (pow(m_slope_Modi, 2) + 1);
	quadratic_formula_b_Top = 2 * (b_intercept_Modi_Top * m_slope_Modi -m_slope_Modi * y_Top - x_Top);
	quadratic_formula_c_Top = pow(b_intercept_Modi_Top, 2) - 2 * b_intercept_Modi_Top*y_Top - pow(r_radius, 2) + pow(x_Top, 2) + pow(y_Top, 2);

	x_One_Top = (-quadratic_formula_b_Top
	+ sqrt(pow(quadratic_formula_b_Top, 2) - 4 * quadratic_formula_a_Top * quadratic_formula_c_Top))
	/ (2 * quadratic_formula_a_Top);
	x_Two_Top = (-quadratic_formula_b_Top
	- sqrt(pow(quadratic_formula_b_Top, 2) - 4 * quadratic_formula_a_Top * quadratic_formula_c_Top))
	/ (2 * quadratic_formula_a_Top);

	y_One_Top = x_One_Top * m_slope_Modi + b_intercept_Modi_Top;
	y_Two_Top = x_Two_Top * m_slope_Modi + b_intercept_Modi_Top;
	*/
	/*
	In this section, 'double' is used to keep the value accurate.
	However, because in OpenCv, 'floating values' cannot be used as 'XY-coordinate', these value needs to be
	changed to 'int' type.
	This process will be automatically done by assigning the values into 'int' type value.

	This can makes the results more accurate, but slows down the system.
	If the system is not powerful enough, change some 'double' format variables into 'int' format variables.
	*/

	double orthogonal_m_slope;

	double orthogonal_b_intercept_Top;
	double quadratic_formula_a_Top;
	double quadratic_formula_b_Top;
	double quadratic_formula_c_Top;
	double discriminant_Top;
	double quadratic_formula_2a_Top;

	double orthogonal_b_intercept_Bottom;
	double quadratic_formula_a_Bottom;
	double quadratic_formula_b_Bottom;
	double quadratic_formula_c_Bottom;
	double discriminant_Bottom;
	double quadratic_formula_2a_Bottom;

	Point XY_One;
	Point XY_Two;
	Point XY_Three;
	Point XY_Four;

	if (In_Three->e_error == false) {
		orthogonal_m_slope = -1 / In_Three->m_slope;

		orthogonal_b_intercept_Top = InOne->Lane_Coordinate_TOP.y - orthogonal_m_slope * InOne->Lane_Coordinate_TOP.x;
		orthogonal_b_intercept_Bottom = InOne->Lane_Coordinate_BOT.y - orthogonal_m_slope * InOne->Lane_Coordinate_BOT.x;



		quadratic_formula_a_Top
			= (pow(orthogonal_m_slope, 2) + 1);
		quadratic_formula_b_Top
			= 2 * (orthogonal_b_intercept_Top * orthogonal_m_slope - orthogonal_m_slope * InOne->Lane_Coordinate_TOP.y - InOne->Lane_Coordinate_TOP.x);
		quadratic_formula_c_Top
			= pow(orthogonal_b_intercept_Top, 2) - 2 * orthogonal_b_intercept_Top * InOne->Lane_Coordinate_TOP.y
			- pow(radius_Top, 2)
			+ pow(InOne->Lane_Coordinate_TOP.x, 2)
			+ pow(InOne->Lane_Coordinate_TOP.y, 2);



		quadratic_formula_a_Bottom
			= (pow(orthogonal_m_slope, 2) + 1);
		quadratic_formula_b_Bottom
			= 2 * (orthogonal_b_intercept_Bottom * orthogonal_m_slope - orthogonal_m_slope * InOne->Lane_Coordinate_BOT.y - InOne->Lane_Coordinate_BOT.x);
		quadratic_formula_c_Bottom
			= pow(orthogonal_b_intercept_Bottom, 2)
			- 2 * orthogonal_b_intercept_Bottom * InOne->Lane_Coordinate_BOT.y
			- pow(radius_Bottom, 2)
			+ pow(InOne->Lane_Coordinate_BOT.x, 2)
			+ pow(InOne->Lane_Coordinate_BOT.y, 2);


		discriminant_Top = sqrt(pow(quadratic_formula_b_Top, 2) - 4 * quadratic_formula_a_Top * quadratic_formula_c_Top);
		discriminant_Bottom = sqrt(pow(quadratic_formula_b_Bottom, 2) - 4 * quadratic_formula_a_Bottom * quadratic_formula_c_Bottom);


		quadratic_formula_2a_Top = 2 * quadratic_formula_a_Top;
		quadratic_formula_2a_Bottom = 2 * quadratic_formula_a_Bottom;


		XY_One.x = (-quadratic_formula_b_Top - discriminant_Top) / quadratic_formula_2a_Top;
		XY_One.y = XY_One.x * orthogonal_m_slope + orthogonal_b_intercept_Top;

		XY_Two.x = (-quadratic_formula_b_Top + discriminant_Top) / quadratic_formula_2a_Top;
		XY_Two.y = XY_Two.x * orthogonal_m_slope + orthogonal_b_intercept_Top;

		XY_Three.x = (-quadratic_formula_b_Bottom + discriminant_Bottom) / quadratic_formula_2a_Bottom;
		XY_Three.y = XY_Three.x * orthogonal_m_slope + orthogonal_b_intercept_Bottom;

		XY_Four.x = (-quadratic_formula_b_Bottom - discriminant_Bottom) / quadratic_formula_2a_Bottom;
		XY_Four.y = XY_Four.x * orthogonal_m_slope + orthogonal_b_intercept_Bottom;

		Output->Masking_Coordinate_One = XY_One;
		Output->Masking_Coordinate_Two = XY_Two;
		Output->Masking_Coordinate_Three = XY_Three;
		Output->Masking_Coordinate_Four = XY_Four;
	}
}



/*
This function stores defined image processing values at the initialization mode,
and allows the system to reuse the value without the initialization mode.
The defined values are same as the followings.
- Threshold Value: the varialbe which decides value to perform threshold
- Background Value: the variable which allows system to filter out the backgound

Process
1. Opens the pre-defined file
-> If the file exists, it opens the files as input mode.
-> If the file does not exist, it creates the file and opens the file as input mode.
2. Stores the received data.
3. Closes the opened file
*/
void init_image_Processing_Val_Saving(
	INPUT int * Val_Threshold,
	INPUT int * Val_Backgrond
) {

	std::ofstream File_imagePData("imagePData.txt", std::ofstream::out);

	if (File_imagePData.is_open() == true) {
		File_imagePData << "Threshold_Val: " << endl;
		File_imagePData << setw(3) << *Val_Threshold << endl;
		File_imagePData << "Background_Val: " << endl;
		File_imagePData << setw(3) << *Val_Backgrond << endl;
	}
	else {
		cout << "File is not exist in the working directory." << endl;
		cout << "Generating the file in the working directory." << endl;

		File_imagePData.open("imagePData.txt");
		File_imagePData << "Threshold_Val: " << endl;
		File_imagePData << setw(3) << *Val_Threshold << endl;
		File_imagePData << "Background_Val: " << endl;
		File_imagePData << setw(3) << *Val_Backgrond << endl;

	}
	File_imagePData.close();
}



/*
This function reads data from the pre-specified file which is inside the working directory.
The data read by this function are the same as followings
- Value Threshold
- Value Background
The values are defined by the initialization mode,
and this function allows the user can skip the initialization phase
if the system is use under same or similar condition.

Process
1. Opens the pre-defined file
-> If the file exists, it opens the files as output mode.
-> If the file does not exist, it terminates the program.
2. Reads the data from the file
3. Store the read data to pre-defined array and return the values.
4. Closes the opened file
*/
void init_image_Processing_Val_Reading(
	OUTPUT int * Val_Threshold,
	OUTPUT int * Val_Backgrond
) {
	string value_Tag[2];
	unsigned int value_ImageP[2];

	std::ifstream File_imagePData("imagePData.txt", std::ifstream::in);

	if (File_imagePData.is_open() == false) {
		cout << "File does not exist" << endl;
		cout << "Terminating the system" << endl;
		system("pause");
		exit(EXIT_FAILURE);
	}
	else {
		File_imagePData >> value_Tag[0];
		File_imagePData >> value_ImageP[0];
		File_imagePData >> value_Tag[1];
		File_imagePData >> value_ImageP[1];

		cout << value_Tag[0] << value_ImageP[0] << endl;
		cout << value_Tag[1] << value_ImageP[1] << endl;
		cout << '\n' << endl;

		*Val_Threshold = value_ImageP[0];
		*Val_Backgrond = value_ImageP[1];

		File_imagePData.close();
	}
}


/*
This fucntion save the result of regression at the initialization phase and store the result to the working directory.
It behaviour is same as other saving function.
*/
void init_Regression_Result_Saving(
	INPUT LinearRegression_Result_OneSide * regression_Left,
	INPUT LinearRegression_Result_OneSide * regression_Right
) {

	std::ofstream File_regressionRData("regressionRData.txt", std::ofstream::out);

	if (File_regressionRData.is_open() == true) {
		File_regressionRData << "Left_Side_Regression_Result: " << endl;
		File_regressionRData << "m_Slope: " << setw(10) << regression_Left->m_slope << endl;
		File_regressionRData << "b_Intercept: " << setw(10) << regression_Left->b_intercept << endl;
		File_regressionRData << "e_Error: " << setw(1) << regression_Left->e_error << endl;
		File_regressionRData << "Left_OR_Right: " << setw(1) << regression_Left->LEFT_OR_RIGHT << endl;

		File_regressionRData << "Right_Side_Regression_Result: " << endl;
		File_regressionRData << "m_Slope: " << setw(10) << regression_Right->m_slope << endl;
		File_regressionRData << "b_Intercept: " << setw(10) << regression_Right->b_intercept << endl;
		File_regressionRData << "e_Error: " << setw(1) << regression_Right->e_error << endl;
		File_regressionRData << "Left_OR_Right: " << setw(1) << regression_Right->LEFT_OR_RIGHT << endl;
	}
	else {
		cout << "File is not exist in the working directory." << endl;
		cout << "Generating the file in the working directory." << endl;

		File_regressionRData.open("imagePData.txt");
		File_regressionRData << "Left_Side_Regression_Result: " << endl;
		File_regressionRData << "m_Slope: " << setw(10) << regression_Left->m_slope << endl;
		File_regressionRData << "b_Intercept: " << setw(10) << regression_Left->b_intercept << endl;
		File_regressionRData << "e_Error: " << setw(1) << regression_Left->e_error << endl;
		File_regressionRData << "Left_OR_Right: " << setw(1) << regression_Left->LEFT_OR_RIGHT << endl;

		File_regressionRData << "Right_Side_Regression_Result: " << endl;
		File_regressionRData << "m_Slope: " << setw(10) << regression_Right->m_slope << endl;
		File_regressionRData << "b_Intercept: " << setw(10) << regression_Right->b_intercept << endl;
		File_regressionRData << "e_Error: " << setw(1) << regression_Right->e_error << endl;
		File_regressionRData << "Left_OR_Right: " << setw(1) << regression_Right->LEFT_OR_RIGHT << endl;

	}
	File_regressionRData.close();
}


/*
This function reads regression result data from the file which is located at the working direcotry.
Its behaviour is same are other reading function.
*/
void init_Regression_Result_Reading(
	OUTPUT LinearRegression_Result_OneSide * regression_Left,
	OUTPUT LinearRegression_Result_OneSide * regression_Right
) {
	std::string masking_Coordinate_Tag_Left[5];
	std::string masking_Coordinate_Tag_Right[5];

	std::ifstream File_regressionRData("regressionRData.txt", std::ifstream::in);

	if (File_regressionRData.is_open() == false) {
		cout << "File does not exist" << endl;
		cout << "Terminating the system" << endl;
		system("pause");
		exit(EXIT_FAILURE);
	}
	else {
		File_regressionRData >> masking_Coordinate_Tag_Left[0];
		File_regressionRData >> masking_Coordinate_Tag_Left[1];
		File_regressionRData >> regression_Left->m_slope;
		File_regressionRData >> masking_Coordinate_Tag_Left[2];
		File_regressionRData >> regression_Left->b_intercept;
		File_regressionRData >> masking_Coordinate_Tag_Left[3];
		File_regressionRData >> regression_Left->e_error;
		File_regressionRData >> masking_Coordinate_Tag_Left[4];
		File_regressionRData >> regression_Left->LEFT_OR_RIGHT;

		cout << masking_Coordinate_Tag_Left[0] << endl;
		cout << masking_Coordinate_Tag_Left[1] << regression_Left->m_slope << endl;
		cout << masking_Coordinate_Tag_Left[2] << regression_Left->b_intercept << endl;
		cout << masking_Coordinate_Tag_Left[3] << regression_Left->e_error << endl;
		cout << masking_Coordinate_Tag_Left[4] << regression_Left->LEFT_OR_RIGHT << endl;



		File_regressionRData >> masking_Coordinate_Tag_Right[0];
		File_regressionRData >> masking_Coordinate_Tag_Right[1];
		File_regressionRData >> regression_Right->m_slope;
		File_regressionRData >> masking_Coordinate_Tag_Right[2];
		File_regressionRData >> regression_Right->b_intercept;
		File_regressionRData >> masking_Coordinate_Tag_Right[3];
		File_regressionRData >> regression_Right->e_error;
		File_regressionRData >> masking_Coordinate_Tag_Right[4];
		File_regressionRData >> regression_Right->LEFT_OR_RIGHT;

		cout << masking_Coordinate_Tag_Right[0] << endl;
		cout << masking_Coordinate_Tag_Right[1] << regression_Right->m_slope << endl;
		cout << masking_Coordinate_Tag_Right[2] << regression_Right->b_intercept << endl;
		cout << masking_Coordinate_Tag_Right[3] << regression_Right->e_error << endl;
		cout << masking_Coordinate_Tag_Right[4] << regression_Right->LEFT_OR_RIGHT << endl;
		cout << '\n' << endl;

		File_regressionRData.close();
	}
}



/*
This function stores defined masking XY-coordinate data at the initialization mode,
and allows the system to reuse the value without the initialization mode.

Process
1. Opens the pre-defined file
-> If the file exists, it opens the files as input mode.
-> If the file does not exist, it creates the file and opens the file as input mode.
2. Stores the received data.
3. Closes the opened file
*/
void init_Masking_Coordinate_Saving(
	INPUT Elements_Masking_Coordinate * masking_Left,
	INPUT Elements_Masking_Coordinate * masking_Right
) {
	std::ofstream File_maskCData("maskCData.txt", std::ofstream::out);

	if (File_maskCData.is_open() == true) {
		File_maskCData << "Left_Masking_Coordinate: " << endl;
		File_maskCData << "One: " << setw(4) << masking_Left->Masking_Coordinate_One.x << " " << setw(4) << masking_Left->Masking_Coordinate_One.y << endl;
		File_maskCData << "Two: " << setw(4) << masking_Left->Masking_Coordinate_Two.x << " " << setw(4) << masking_Left->Masking_Coordinate_Two.y << endl;
		File_maskCData << "Three: " << setw(4) << masking_Left->Masking_Coordinate_Three.x << " " << setw(4) << masking_Left->Masking_Coordinate_Three.y << endl;
		File_maskCData << "Four: " << setw(4) << masking_Left->Masking_Coordinate_Four.x << " " << setw(4) << masking_Left->Masking_Coordinate_Four.y << endl;

		File_maskCData << "Right_Masking_Coordinate: " << endl;
		File_maskCData << "One: " << setw(4) << masking_Right->Masking_Coordinate_One.x << " " << setw(4) << masking_Right->Masking_Coordinate_One.y << endl;
		File_maskCData << "Two: " << setw(4) << masking_Right->Masking_Coordinate_Two.x << " " << setw(4) << masking_Right->Masking_Coordinate_Two.y << endl;
		File_maskCData << "Three: " << setw(4) << masking_Right->Masking_Coordinate_Three.x << " " << setw(4) << masking_Right->Masking_Coordinate_Three.y << endl;
		File_maskCData << "Four: " << setw(4) << masking_Right->Masking_Coordinate_Four.x << " " << setw(4) << masking_Right->Masking_Coordinate_Four.y << endl;
	}
	else {
		cout << "File is not exist in the working directory." << endl;
		cout << "Generating the file in the working directory." << endl;

		File_maskCData.open("maskCData.txt", std::ofstream::out);
		File_maskCData << "Left_Masking_Cooridata: " << endl;
		File_maskCData << "One: " << setw(4) << masking_Left->Masking_Coordinate_One.x << " " << setw(4) << masking_Left->Masking_Coordinate_One.y << endl;
		File_maskCData << "Two: " << setw(4) << masking_Left->Masking_Coordinate_Two.x << " " << setw(4) << masking_Left->Masking_Coordinate_Two.y << endl;
		File_maskCData << "Three: " << setw(4) << masking_Left->Masking_Coordinate_Three.x << " " << setw(4) << masking_Left->Masking_Coordinate_Three.y << endl;
		File_maskCData << "Four: " << setw(4) << masking_Left->Masking_Coordinate_Four.x << " " << setw(4) << masking_Left->Masking_Coordinate_Four.y << endl;

		File_maskCData << "Right_Masking_Cooridata: " << endl;
		File_maskCData << "One: " << setw(4) << masking_Right->Masking_Coordinate_One.x << " " << setw(4) << masking_Right->Masking_Coordinate_One.y << endl;
		File_maskCData << "Two: " << setw(4) << masking_Right->Masking_Coordinate_Two.x << " " << setw(4) << masking_Right->Masking_Coordinate_Two.y << endl;
		File_maskCData << "Three: " << setw(4) << masking_Right->Masking_Coordinate_Three.x << " " << setw(4) << masking_Right->Masking_Coordinate_Three.y << endl;
		File_maskCData << "Four: " << setw(4) << masking_Right->Masking_Coordinate_Four.x << " " << setw(4) << masking_Right->Masking_Coordinate_Four.y << endl;

	}
	File_maskCData.close();
}



/*
This function reads data from the pre-specified file which is inside the working directory.
The data read by this function are the same as followings
- Left Initial Masking XY-Coordinate
- Right Initial Masking XY-Coordinate
The coordinate data are defined by the initialization mode,
and this function allows the user can skip the initialization phase
if the system is use under same or similar condition.

Process
1. Opens the pre-defined file
-> If the file exists, it opens the files as output mode.
-> If the file does not exist, it terminates the program.
2. Reads the data from the file
3. Store the read data to pre-defined array and return the values.
4. Closes the opened file
*/
void init_Masking_Coordinate_Reading(
	OUTPUT Elements_Masking_Coordinate * masking_Left,
	OUTPUT Elements_Masking_Coordinate * masking_Right
) {
	std::string masking_Coordinate_Tag_Left[5];
	Elements_Masking_Coordinate temp_masking_Left;
	std::string masking_Coordinate_Tag_Right[5];
	Elements_Masking_Coordinate temp_masking_Right;

	std::ifstream File_maskCData("maskCData.txt", std::ifstream::in);

	if (File_maskCData.is_open() == false) {
		cout << "File does not exist" << endl;
		cout << "Terminating the system" << endl;
		system("pause");
		exit(EXIT_FAILURE);
	}
	else {
		File_maskCData >> masking_Coordinate_Tag_Left[0];
		File_maskCData >> masking_Coordinate_Tag_Left[1];
		File_maskCData >> temp_masking_Left.Masking_Coordinate_One.x;
		File_maskCData >> temp_masking_Left.Masking_Coordinate_One.y;
		File_maskCData >> masking_Coordinate_Tag_Left[2];
		File_maskCData >> temp_masking_Left.Masking_Coordinate_Two.x;
		File_maskCData >> temp_masking_Left.Masking_Coordinate_Two.y;
		File_maskCData >> masking_Coordinate_Tag_Left[3];
		File_maskCData >> temp_masking_Left.Masking_Coordinate_Three.x;
		File_maskCData >> temp_masking_Left.Masking_Coordinate_Three.y;
		File_maskCData >> masking_Coordinate_Tag_Left[4];
		File_maskCData >> temp_masking_Left.Masking_Coordinate_Four.x;
		File_maskCData >> temp_masking_Left.Masking_Coordinate_Four.y;

		cout << masking_Coordinate_Tag_Left[0] << endl;
		cout << masking_Coordinate_Tag_Left[1] << temp_masking_Left.Masking_Coordinate_One.x << " " << temp_masking_Left.Masking_Coordinate_One.y << endl;
		cout << masking_Coordinate_Tag_Left[2] << temp_masking_Left.Masking_Coordinate_Two.x << " " << temp_masking_Left.Masking_Coordinate_Two.y << endl;
		cout << masking_Coordinate_Tag_Left[3] << temp_masking_Left.Masking_Coordinate_Three.x << " " << temp_masking_Left.Masking_Coordinate_Three.y << endl;
		cout << masking_Coordinate_Tag_Left[4] << temp_masking_Left.Masking_Coordinate_Four.x << " " << temp_masking_Left.Masking_Coordinate_Four.y << endl;

		*masking_Left = temp_masking_Left;



		File_maskCData >> masking_Coordinate_Tag_Right[0];
		File_maskCData >> masking_Coordinate_Tag_Right[1];
		File_maskCData >> temp_masking_Right.Masking_Coordinate_One.x;
		File_maskCData >> temp_masking_Right.Masking_Coordinate_One.y;
		File_maskCData >> masking_Coordinate_Tag_Right[2];
		File_maskCData >> temp_masking_Right.Masking_Coordinate_Two.x;
		File_maskCData >> temp_masking_Right.Masking_Coordinate_Two.y;
		File_maskCData >> masking_Coordinate_Tag_Right[3];
		File_maskCData >> temp_masking_Right.Masking_Coordinate_Three.x;
		File_maskCData >> temp_masking_Right.Masking_Coordinate_Three.y;
		File_maskCData >> masking_Coordinate_Tag_Right[4];
		File_maskCData >> temp_masking_Right.Masking_Coordinate_Four.x;
		File_maskCData >> temp_masking_Right.Masking_Coordinate_Four.y;

		cout << masking_Coordinate_Tag_Right[0] << endl;
		cout << masking_Coordinate_Tag_Right[1] << temp_masking_Right.Masking_Coordinate_One.x << " " << temp_masking_Right.Masking_Coordinate_One.y << endl;
		cout << masking_Coordinate_Tag_Right[2] << temp_masking_Right.Masking_Coordinate_Two.x << " " << temp_masking_Right.Masking_Coordinate_Two.y << endl;
		cout << masking_Coordinate_Tag_Right[3] << temp_masking_Right.Masking_Coordinate_Three.x << " " << temp_masking_Right.Masking_Coordinate_Three.y << endl;
		cout << masking_Coordinate_Tag_Right[4] << temp_masking_Right.Masking_Coordinate_Four.x << " " << temp_masking_Right.Masking_Coordinate_Four.y << endl;
		cout << '\n' << endl;

		*masking_Right = temp_masking_Right;

		File_maskCData.close();
	}
}



/*
init_Mode() function
This function merged every initialization function to make user friendly condtion.
*/
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
) {
	cv::Mat instFrame;
	cv::Mat instROI_Left;
	cv::Mat instROI_Right;
	int init_mode_Checker;

	std::cout
		<< "Initialization Mode" << '\n'
		<< "0 - Skip to Enter" << '\n'
		<< "1 - Initialize All" << '\n'
		<< "2 - Initialize Image Processing Value Only" << '\n'
		<< "3 - Initialize Masking Cooridnate Only" << '\n'
		<< endl;

	std::cin >> init_mode_Checker;

	switch (init_mode_Checker) {
	case 0:
		cout << "Initialization Phase Skipped" << endl;
		init_image_Processing_Val_Reading(Val_Threshold, Val_Backgounrd);
		init_Regression_Result_Reading(regression_RLeft, regression_RRIGHT);
		init_Masking_Coordinate_Reading(XY_Coordinate_MLeft, XY_Coordinate_MRIght);
		break;

	case 1:
		cout << "Proceeding Initialization Phase" << endl;
		initializer_IMG_CVT_VALUE(inputVideo, Val_Backgounrd, Val_Threshold);
		init_image_Processing_Val_Saving(Val_Threshold, Val_Backgounrd);

		inputVideo->read(instFrame);
		instROI_Left = instFrame(*region_of_Interest_LEFT);
		initializer_Masking_and_Regression(&instROI_Left, regression_RLeft, XY_Coordinate_MLeft);
		instROI_Right = instFrame(*region_of_Interest_Right);
		initializer_Masking_and_Regression(&instROI_Right, regression_RRIGHT, XY_Coordinate_MRIght);

		init_Regression_Result_Saving(regression_RLeft, regression_RRIGHT);
		init_Masking_Coordinate_Saving(XY_Coordinate_MLeft, XY_Coordinate_MRIght);
		break;

	case 2:
		cout << "Proceeding Image Value Initialization Phase" << endl;
		initializer_IMG_CVT_VALUE(inputVideo, Val_Backgounrd, Val_Threshold);
		init_image_Processing_Val_Saving(Val_Threshold, Val_Backgounrd);

		init_Regression_Result_Reading(regression_RLeft, regression_RRIGHT);
		init_Masking_Coordinate_Reading(XY_Coordinate_MLeft, XY_Coordinate_MRIght);
		break;

	case 3:
		cout << "Proceeding Masking Coordinate Initialization Phase" << endl;
		init_image_Processing_Val_Reading(&Value_Threshold, &Value_Background);

		inputVideo->read(instFrame);
		instROI_Left = instFrame(*region_of_Interest_LEFT);
		initializer_Masking_and_Regression(&instROI_Left, regression_RLeft, XY_Coordinate_MLeft);
		instROI_Right = instFrame(*region_of_Interest_Right);
		initializer_Masking_and_Regression(&instROI_Right, regression_RRIGHT, XY_Coordinate_MRIght);

		init_Regression_Result_Saving(regression_RLeft, regression_RRIGHT);
		init_Masking_Coordinate_Saving(XY_Coordinate_MLeft, XY_Coordinate_MRIght);
		break;

	default:
		cout << "Initialization Phase Skipped" << endl;
		init_image_Processing_Val_Reading(Val_Threshold, Val_Backgounrd);
		init_Regression_Result_Reading(regression_RLeft, regression_RRIGHT);
		init_Masking_Coordinate_Reading(XY_Coordinate_MLeft, XY_Coordinate_MRIght);
		break;
	}
	system("pause");
}