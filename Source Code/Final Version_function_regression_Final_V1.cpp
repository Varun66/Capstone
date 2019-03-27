#include"header_Final_V1.h"

/*Variable used for image analizing*/
int curr_height = 0;
int image_midline = 0;
int image_row = 0;
int image_column = 0;
/*Variable used for image analizing*/



/*-----------------------------Buoy Detecting Method-----------------------------*/
/*
Buoy_Detector_Parallel() fucntion
The function receives one input and generates one output.
The function receives one colour channel image and 
it uses 'findContours()' function to find contour of remained objects in the image.
Based on the retrieved data from 'findContours()' function, it finds moments of the found contour.
If the calculated moment data passes the following error check, 
it assigns stores the data into array as an buoy position data, and updates the number of identified buoy.
-> Check the value is infinite
-> Check the value is 'not-a-number'

Parameters
INPUT cv::Mat *inputFrame
=> Mat type input with one colour channel

OUTPUT LinearRegression_Element_NotInitialized * Output
=> Struct type output which includes data of identify buoys.
-> Array - Identified buoy position
-> Int - Number of Identified buoy.
-> Bool - Is enough number of the buoy identified
*/
void Buoy_Detector_Parallel(
	INPUT cv::Mat *inputFrame,
	OUTPUT LinearRegression_Element_NotInitialized * Output
) {
	Point temp_XY_Coordinate[Size_Array];
	int temp_Contours_Size;

	/*
	Assigns the previous data points to the tempory array.
	It allows the system can update the data point one by one, 
	and skip updating data points if a specific error case is detected.
	*/
	for (int i = 0; i < Size_Array; i++) {
		temp_XY_Coordinate[i] = Output->XY_Coordinate[i];
	}
	Output->Num_identified_Buoy = 0;

	/*
	Finds the contour of the remained objected from filtered image.
	*/
	vector<vector<Point>>contours;
	vector<Vec4i>hierarchy;
	findContours(*inputFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/*
	Declare space for moment calculation
	*/
	temp_Contours_Size = contours.size();
	vector<Moments> mu(temp_Contours_Size);
	vector<Point2f> mc(temp_Contours_Size);

	/*
	Calculates moment for the found contour	
	*/
	for (int i = 0; i < temp_Contours_Size; i++) {
		mu[i] = moments(contours[i], false);
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

		/*
		Checks the calculation result is 'infinite' or 'not-a-number'
		*/
		if (cvIsNaN(mc[i].x) != 1 && cvIsNaN(mc[i].y) != 1) {
			if (Output->Array_Full != true) {
				if (Output->Array_Position_Counter > Size_Array - 1) {
					Output->Array_Position_Counter = 0;
					Output->Array_Full = true;
				}
				else {
					Output->Array_Position_Counter = Output->Array_Position_Counter + 1;
				}
			}

			Output->Num_identified_Buoy = Output->Num_identified_Buoy + 1;

			for (int arrayPos = Size_Array - 1; arrayPos > 0; arrayPos--) {
				temp_XY_Coordinate[arrayPos] = temp_XY_Coordinate[arrayPos - 1];
			}
			temp_XY_Coordinate[0] = Point2f(mc[i].x, mc[i].y);
		}
	}

	/*
	Check enough number of buoys are detected and if not do not update buoy position data.
	*/
	if (Output->Num_identified_Buoy > 1) {
		Output->Not_Enough_Buoy = false;
		for (int i = 0; i < Size_Array; i++) {
			Output->XY_Coordinate[i] = temp_XY_Coordinate[i];

		}
	}
	else {
		Output->Not_Enough_Buoy = true;
	}

}



/*-----------------------------OLS Method-----------------------------*/
/*
OLS_Regression_Ele_Init_MomentBased_Parallel() function
The function receives the data about the identified buoy, and generates an output which is required to perform least square regression.

Parameters
INPUT LinearRegression_Element_NotInitialized * Input
=> Struct type input which includes data of the identified buoys.
-> Array - Identified buoy position
-> Int - Number of the identified buoy.
-> Bool - Is enough number of the buoy identified

OUTPUT LinearRegression_Element_OLS * Output
=> Struct type output which includes data required to perform least sqaure regression method
-> Number of the identified buoy
-> Sum of X coordinate data
-> Sum of X^2 of coordinate data
-> Sum of Y coordinate data
-> Sum of X*Y of X and Y coordinate data
*/
void OLS_Regression_Ele_Init_MomentBased_Parallel(
	INPUT LinearRegression_Element_NotInitialized * Input,
	OUTPUT LinearRegression_Element_OLS * Output
) {
	if (Input->Array_Full == true) {
		Output->Num_identified_Buoy = Size_Array;

		for (int Array_Pos = 0; Array_Pos < Size_Array; Array_Pos++) {
			Output->x_Sum = Output->x_Sum + Input->XY_Coordinate[Array_Pos].x;
			Output->xx_Sum = Output->xx_Sum + pow(Input->XY_Coordinate[Array_Pos].x, 2);
			Output->y_Sum = Output->y_Sum + Input->XY_Coordinate[Array_Pos].y;
			Output->xy_Sum = Output->xy_Sum + Input->XY_Coordinate[Array_Pos].x * Input->XY_Coordinate[Array_Pos].y;
		}
	}
	else {
		Output->Num_identified_Buoy = Input->Array_Position_Counter;

		for (int Array_Pos = 0; Array_Pos < Input->Array_Position_Counter; Array_Pos++) {
			Output->x_Sum = Output->x_Sum + Input->XY_Coordinate[Array_Pos].x;
			Output->xx_Sum = Output->xx_Sum + pow(Input->XY_Coordinate[Array_Pos].x, 2);
			Output->y_Sum = Output->y_Sum + Input->XY_Coordinate[Array_Pos].y;
			Output->xy_Sum = Output->xy_Sum + Input->XY_Coordinate[Array_Pos].x * Input->XY_Coordinate[Array_Pos].y;
		}
	}
}


/*
OLS_Regression_Cal_Parallel()
This function receives one input and generate one output.
Based on the initialized elements, this performs Least Square Regression to find slope and intercept of the input data.
Also, it performs error checks to avoid 'infinite' or 'not-a-number' results

Parameters
INPUT LinearRegression_Element_OLS * Input
=> Struct type input which includes data required to perform least sqaure regression method
-> Number of the identified buoy
-> Sum of X coordinate data
-> Sum of X^2 of coordinate data
-> Sum of Y coordinate data
-> Sum of X*Y of X and Y coordinate data

OUTPUT LinearRegression_Result_OneSide * Output
=> Struct type output which includes regression results
=> Slope of the regression
=> Intercept of the regression
=> Error Flag that allows the system to know the result of current input data caused an error.
*/
void OLS_Regression_Cal_Parallel(
	INPUT LinearRegression_Element_OLS * Input,
	OUTPUT LinearRegression_Result_OneSide * Output
) {
	double temp_m_Slope;
	double temp_b_Intercept;

	/*
	Calculation Phase
	*/
	temp_m_Slope =
		(Input->Num_identified_Buoy * Input->xy_Sum - Input->x_Sum * Input->y_Sum) /
		(Input->Num_identified_Buoy * Input->xx_Sum - pow(Input->x_Sum, 2));
	temp_b_Intercept =
		(Input->y_Sum - temp_m_Slope * Input->x_Sum) /
		Input->Num_identified_Buoy;

	/*
	Error Check Phase
	If the results are failed to pass the error check, the calculation results will not be updated
	*/
	if (cvIsNaN(temp_m_Slope) == 1 ||
		cvIsInf(temp_m_Slope) == 1 ||
		cvIsNaN(temp_b_Intercept) == 1 ||
		cvIsInf(temp_b_Intercept) == 1) {
		Output->e_error = true;
	}
	else {
		if (Output->LEFT_OR_RIGHT == true && temp_m_Slope > 0) {
			Output->e_error = true;
		}
		else if (Output->LEFT_OR_RIGHT == false && temp_m_Slope < 0) {
			Output->e_error = true;
		}
		else {
			Output->m_slope = temp_m_Slope;
			Output->b_intercept = temp_b_Intercept;
			Output->e_error = false;
		}
	}
}



/*-----------------------------Robust Method-----------------------------*/
/*
Robust_Regression_Ele_Init_Parallel() function
The function receives the data about the identified buoy and previous regression results, and generates an output which is required to perform robust regression with Tukey's method.

Parameters
INPUT	LinearRegression_Element_NotInitialized	* Input_One
=> Struct type input which includes data of identify buoys.
-> Array - Identified buoy position
-> Int - Number of Identified buoy.
-> Bool - Is enough number of the buoy identified

INPUT	LinearRegression_Result_OneSide			* Input_Two
=> Struct type input which includes data of previous regression results
-> Double - slope
-> Double - intercept

OUTPUT	LinearRegression_Element_Robust			* Output
=> Struct type output which includes data to perform robust regression.
-> Point of array - Identified buoys' XY-Coordinate data
-> Int - Number of indentified buoy
-> Double - Mean value of X-coordinate data
-> Double type array - weights value of identified buoys
-> Double - Initial weight for X-coordinate data
-> Double - Initial weight for Y-coordinate data
-> Double - Sum of X^2 of X-coordinate data.
*/
void Robust_Regression_Ele_Init_Parallel(
	INPUT	LinearRegression_Element_NotInitialized	* Input_One,
	INPUT	LinearRegression_Result_OneSide			* Input_Two,
	OUTPUT	LinearRegression_Element_Robust			* Output
) {
	/*
	Initiailzes requried variables.
	*/
	int arraySize = 0;
	int inst_delta_Y;
	double inst_x_Sum = 0;
	double inst_xx_Sum = 0;
	double inst_w_Sum = 0;
	double inst_Result_wX_Sum = 0;
	double inst_Result_wY_Sum = 0;

	/*
	If Array is full, the number of identified buoy or stored data is same as the size of array.
	If not, it uses current 'end' point of array as number of idnetified buoy.
	*/
	if (Input_One->Array_Full == true) {
		arraySize = Size_Array;
	}
	else {
		arraySize = Input_One->Array_Position_Counter;
	}


	for (int i = 0; i < arraySize; i++) {
		/*
		Assigns XY-Coordinate of identified buoys.
		*/
		Output->XY_Coordinate_Data[i] = Input_One->XY_Coordinate[i];

		/*
		Calculates and assignes weight of input data based on the previouse regression result and current input data.
		Also, if 'inst_delta_Y' equals to zero, the weight of the input considers as 1.
		*/
		inst_delta_Y
			= Input_One->XY_Coordinate[i].y - (Input_Two->m_slope * (Input_One->XY_Coordinate[i].x) + Input_Two->b_intercept);

		if (inst_delta_Y == 0) {
			Output->init_weight[i] = 1;
		}
		else {
			Output->init_weight[i] = 1 / pow(inst_delta_Y, 2);
		}

		inst_x_Sum = inst_x_Sum + Input_One->XY_Coordinate[i].x;
		inst_xx_Sum = inst_xx_Sum + pow(Input_One->XY_Coordinate[i].x, 2);
		inst_Result_wX_Sum = inst_Result_wX_Sum + Output->init_weight[i] * Input_One->XY_Coordinate[i].x;
		inst_Result_wY_Sum = inst_Result_wY_Sum + Output->init_weight[i] * Input_One->XY_Coordinate[i].y;
		inst_w_Sum = inst_w_Sum + Output->init_weight[i];

	}

	Output->xx_Sum = inst_xx_Sum;
	Output->init_X_weight = inst_Result_wX_Sum / inst_w_Sum;
	Output->init_Y_weight = inst_Result_wY_Sum / inst_w_Sum;
	Output->x_mean = inst_x_Sum / arraySize;
	Output->Num_of_Collected_Data = arraySize;

}


/*
Robust_Regression_Cal_Parallel() function
Based on the initialized elements, this performs Robust Regression with Tukey's method to find slope and intercept of the input data.
In addtion, it performs error checks to prevent 'infinite' and 'not-a-number' results.

To get basic information about Robust Regression, please read the following link.
https://onlinecourses.science.psu.edu/stat501/node/352
https://onlinecourses.science.psu.edu/stat501/node/353
https://www.mathworks.com/help/curvefit/least-squares-fitting.html#bq_5kr9-3
https://www.mathworks.com/help/curvefit/least-squares-fitting.html#bq_5kr9-4

Parameters
INPUT LinearRegression_Element_Robust * Input
=> Struct type input which includes data to perform robust regression.
-> Point of the array - Identified buoys' XY-Coordinate data
-> Int - Number of the identified buoy
-> Double - Mean value of X-coordinate data
-> Double type array - weights value of identified buoys
-> Double - Initial weight for X-coordinate data
-> Double - Initial weight for Y-coordinate data
-> Double - Sum of X^2 of X-coordinate data.

OUTPUT LinearRegression_Result_OneSide * Output
=> Struct type output which includes regression results
-> Double - Slope of the regression
-> Double - Intercept of the regression
-> Bool -Error Flag that allows the system to know the result of current input data caused an error.
*/
void Robust_Regression_Cal_Parallel(
	INPUT LinearRegression_Element_Robust * Input,
	OUTPUT LinearRegression_Result_OneSide * Output
) {
	int K_tuning_const = 4.685;
	int loop_control = 0;
	int DataCount = 0;

	int NumCollected = Input->Num_of_Collected_Data;
	double x_weight = Input->init_X_weight;
	double y_weight = Input->init_Y_weight;

	double instResult_One = 0;
	double instResult_Two = 0;
	double m_slope_weight = 0;
	double b_intercept_weight = 0;

	double r_residuals[Size_Array];
	double r_residuals_MAD[Size_Array];
	double h_leverage = 0;
	double r_adjust = 0;
	double u_standarized_r_adj = 0;
	double median = 0;
	double MAD = 0;
	double inst_Result_wX_Sum = 0;
	double inst_Result_wY_Sum = 0;
	double inst_Result_w_Sum = 0;


	while (loop_control < 5) {
		//m_slope_weight =
		//		sum(w_i*(x_i - x_weight)*(y_i - y_weight)) / sum(w_i*(x_i - x_weight) ^ 2)
		//	b_intercept_weight =
		//		y_weight - m_slope * x_weight
		for (DataCount = 0; DataCount < NumCollected; DataCount++) {
			instResult_One = instResult_One +
				(Input->init_weight[DataCount] * (Input->XY_Coordinate_Data[DataCount].x - x_weight) * (Input->XY_Coordinate_Data[DataCount].y - y_weight));
			instResult_Two = instResult_Two +
				(Input->init_weight[DataCount] * pow((Input->XY_Coordinate_Data[DataCount].x - x_weight), 2));
		}
		m_slope_weight = instResult_One / instResult_Two;
		b_intercept_weight = y_weight - m_slope_weight * x_weight;


		for (DataCount = 0; DataCount < NumCollected; DataCount++) {
			r_residuals[DataCount] =
				Input->XY_Coordinate_Data[DataCount].y -
				(m_slope_weight * Input->XY_Coordinate_Data[DataCount].x + b_intercept_weight);
			r_residuals_MAD[DataCount] = r_residuals[DataCount];
		}

		std::sort(r_residuals_MAD, r_residuals_MAD + NumCollected);
		if (NumCollected % 2 == 0) {
			median = (r_residuals_MAD[NumCollected / 2 + 1] + r_residuals_MAD[NumCollected / 2]) / 2;
		}
		else {
			median = r_residuals_MAD[NumCollected / 2 + 1];
		}

		for (DataCount = 0; DataCount < NumCollected; DataCount++) {
			r_residuals_MAD[DataCount] = abs(r_residuals[DataCount] - median);
		}

		std::sort(r_residuals_MAD, r_residuals_MAD + NumCollected);
		if (NumCollected % 2 == 0) {
			MAD = (r_residuals_MAD[NumCollected / 2 + 1] + r_residuals_MAD[NumCollected / 2]) / 2;
		}
		else {
			MAD = r_residuals_MAD[NumCollected / 2 + 1];
		}
		

		for (DataCount = 0; DataCount < NumCollected; DataCount++) {

			h_leverage =
				1 / NumCollected + pow((Input->XY_Coordinate_Data[DataCount].x - Input->x_mean), 2) / Input->xx_Sum;
			r_adjust =
				r_residuals[DataCount] / sqrt(1 - h_leverage);
			u_standarized_r_adj
				= r_adjust / (K_tuning_const*MAD);

			if (abs(u_standarized_r_adj) < 1) {
				Input->init_weight[DataCount] = pow((1 - pow(u_standarized_r_adj, 2)), 2);
			}
			else {
				Input->init_weight[DataCount] = 0;
			}

			inst_Result_wX_Sum = inst_Result_wX_Sum + Input->init_weight[DataCount] * Input->XY_Coordinate_Data[DataCount].x;
			inst_Result_wY_Sum = inst_Result_wY_Sum + Input->init_weight[DataCount] * Input->XY_Coordinate_Data[DataCount].y;
			inst_Result_w_Sum = inst_Result_w_Sum + Input->init_weight[DataCount];

		}

		if (inst_Result_wX_Sum == 0 || inst_Result_wY_Sum == 0 || inst_Result_w_Sum == 0) {
			break;
		}

		x_weight = inst_Result_wX_Sum / inst_Result_w_Sum;
		y_weight = inst_Result_wY_Sum / inst_Result_w_Sum;
		loop_control = loop_control + 1;
		
	}
	

	if (cvIsNaN(m_slope_weight) == 1 || cvIsNaN(b_intercept_weight) == 1 ||
		cvIsInf(m_slope_weight) == 1 ||	cvIsInf(b_intercept_weight) == 1) {
		Output->e_error = true;
	}
	else {
		if (Output->LEFT_OR_RIGHT == true && m_slope_weight > 0) {
			Output->e_error = true;
		}
		else if (Output->LEFT_OR_RIGHT == false && m_slope_weight < 0) {
			Output->e_error = true;
		} else {
			Output->m_slope = m_slope_weight;
			Output->b_intercept = b_intercept_weight;
			Output->e_error = false;
		}
	}
}



/*-----------------------------Lane Position Cal-----------------------------*/
/*
XY_Coordinate_Cal_Trajectory_Parallel() function
This function calculates XY-Coordinate of the recommended trajectory of the boat.
The Top XY-Coordinate is same as the intersection point of two line given by linear regression.
The Bottom XY-Coordinate is bottom mid-point where the camera is located.
In addition, it calculates the expected height of horizon.

Parameters
INPUT LinearRegression_Result_OneSide * Input_Left
=> Struct type input which includes the result of regression for the left side.
-> Double - Slope of the regression
-> Double - Intercept of the regression
-> Bool - Error Flag, that allows the system to know the result of current input data caused an error.

INPUT LinearRegression_Result_OneSide * Input_Right
=> Struct type input which includes the result of regression for the right side.
-> Double - Slope of the regression
-> Double - Intercept of the regression
-> Bool - Error Flag, that allows the system to know the result of current input data caused an error.

OUTPUT int * calculated_height
=> Int type output that defines the height of the horizon.
=> It is 'int' type because the image coordinate is made of 'int' type values
=> The is not actual height of horizon that this is expected to value.
=> Horizon tracking method required too many resources and long execution time, 
the method could not be used for the real-time system.

OUTPUT Elements_Trajectory_Coordinate * tempOutput
=> Struct type output that includes the starting and ending coordinate of the recommanded trajectory line.
-> Point - Trajectory Top(= Start)
-> Point - Trajectory Bot(= End)
*/
void XY_Coordinate_Cal_Trajectory_Parallel(
	INPUT LinearRegression_Result_OneSide * Input_Left,
	INPUT LinearRegression_Result_OneSide * Input_Right,
	OUTPUT int * calculated_height,
	OUTPUT Elements_Trajectory_Coordinate * tempOutput
) {
	static cv::Point intersection_Point;
	static cv::Point2d data_DeltaXY;
	static int boatAngle;
	static double rad2deg = 180 / CV_PI;

	/*
	Method One
	- Find a intersection coordinate of two lines without shifting.
	- Shift the intersection cooridinate by midline
	-> If 'LEFT' is used, shift to right (Addition)
	-> If 'RIGHT' is used, shift to left (Substraction)

	intersection_Point.x = image_midline/2 +(Input_Right->b_intercept - Input_Left->b_intercept) / (Input_Left->m_slope - Input_Right->m_slope);
	intersection_Point.y = Input_Left->m_slope * (intersection_Point.x) + Input_Left->b_intercept;
	*/

	/*
	Method Two
	- Find a intersection coordinate of two lines based on shifted line.
	- Shifted by 'mid-line'
	-> If 'LEFT' is shifted, shift to left (Addition)
	-> If 'RIGHT' is sifhed, shift to right (Substraction)
	*/

	intersection_Point.x = (Input_Right->b_intercept - Input_Left->b_intercept - Input_Right->m_slope * image_midline) / (Input_Left->m_slope - Input_Right->m_slope);
	intersection_Point.y = Input_Left->m_slope * intersection_Point.x + Input_Left->b_intercept;

	data_DeltaXY.x	= intersection_Point.x - image_midline;
	data_DeltaXY.y	= image_row - intersection_Point.y;
	boatAngle		= data_DeltaXY.x / data_DeltaXY.y * rad2deg;

	*calculated_height = intersection_Point.y + 15;

	tempOutput->Trajectory_Coordinate_TOP	= intersection_Point;
	tempOutput->Trajectory_Coordinate_BOT	= Point(image_midline, image_row);
	tempOutput->AngleToTurn					= boatAngle;	
}



/*
XY_Coordinate_Cal_Buoy_Lane_Parallel() function
This function calculates starting and ending of XY-Coordinate of buoy lanes.

Parameters
INPUT LinearRegression_Result_OneSide * Input,
=> Struct type input which includes the result of regression.
-> Double - Slope of the regression
-> Double - Intercept of the regression
-> Bool - Error Flag, that allows the system to know the result of current input data caused an error.

INPUT int *currHeight,
=> Position of the horizon, given by XY_Coordinate_Cal_Trajectory() function.

OUTPUT Elements_Lane_Coordinate * Output
=> Struct that includes the starting and ending points of the buoy lanes.
-> Point - Lane Coordinate Top(= Start)
-> Point - Lane Coordinate Bot(= End)
*/
void XY_Coordinate_Cal_Buoy_Lane_Parallel(
	INPUT LinearRegression_Result_OneSide * Input,
	INPUT int *currHeight,
	OUTPUT Elements_Lane_Coordinate * Output
) {
	cv::Point tempPoint_Col_Zero;	// where x-coordinate is 0
	cv::Point tempPoint_Col_Mid;	// where x-coordinate is image_Midline
	cv::Point tempPoint_Col_Y;

	if (Input->e_error != 1) {
		tempPoint_Col_Zero = cv::Point(0, Input->m_slope * (0) + Input->b_intercept);
		tempPoint_Col_Mid = cv::Point(image_midline, Input->m_slope * (image_midline)+Input->b_intercept);
		tempPoint_Col_Y = cv::Point((*currHeight - Input->b_intercept) / Input->m_slope, *currHeight);
		
		if (tempPoint_Col_Mid.y > tempPoint_Col_Zero.y) {
			// Case RIGHT
			// Start X-coodinate = image_Mid
			// End X-coodinate = 0
			Output->Lane_Coordinate_TOP = tempPoint_Col_Y;
			Output->Lane_Coordinate_BOT = tempPoint_Col_Mid;
		}
		else {
			// Case LEFT
			// Start X-coodinate = 0
			// End X-coodinate = image_Mid
			Output->Lane_Coordinate_TOP = tempPoint_Col_Y;
			Output->Lane_Coordinate_BOT = tempPoint_Col_Zero;
		}
	}
}



/*-----------------------------Lane Position Cal-----------------------------*/