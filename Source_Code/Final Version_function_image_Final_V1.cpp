#include"header_Final_V1.h"

/*
external value.
*/
int Value_Threshold;
int Value_Background;

/*
Defines the name of pre-defined struct in 'std::ostringstream' and 'std::string stringPrint'
to prints message on the screen.
*/
std::ostringstream stringSave;
std::string stringPrint;


/*
'image_Convertor()' function 
The function receives an original image data as an input and 
generates a threshold applied image as an output.
The function extracts 'red' and 'blue' colour channel from the input and 
finds the difference between this two to remove white colour.
After finding the difference, it subtracts background to minimize the effect of noises.
After subtraction phase, it applies 'threshold' method to remove most outliers.

Parameters
INPUT	cv::Mat *inputFrame
=> Mat type input image data with three colour channel.

OUTPUT	cv::Mat *outputFrame
=> Mat type output image data with one colour channel.
*/
void image_Convertor(
	INPUT	cv::Mat *inputFrame,
	OUTPUT	cv::Mat *outputFrame
) {
	cv::Mat temp_Frame_splitColour_RED;
	cv::Mat temp_Frame_splitColour_BLUE;
	cv::Mat temp_Frame_Threshold;
	/*
	Defines, Mat backgound which is required to minimize the effect of noise.
	'Value_Background' value is defined at the initialization phase.
	*/
	static cv::Mat temp_Frame_avgBackground(image_row, image_midline, CV_8UC1, Scalar(Value_Background));

	/*
	Extracting Colour channels from BGR image data.
	*/
	cv::extractChannel(*inputFrame, temp_Frame_splitColour_RED, 2);
	cv::extractChannel(*inputFrame, temp_Frame_splitColour_BLUE, 0);

	/*
	Finds the difference between the channels
	*/
	temp_Frame_splitColour_RED = temp_Frame_splitColour_RED - temp_Frame_splitColour_BLUE;
	temp_Frame_splitColour_RED = temp_Frame_splitColour_RED - temp_Frame_avgBackground;

	/*
	Applying threshold method.
	'Value_Threshold' value is defined at the initialization phase.
	*/
	cv::threshold(temp_Frame_splitColour_RED, *outputFrame, Value_Threshold, 255, THRESH_BINARY);
}


/*
'frame_accumulator()' function
The function receives three inputs and generates one output.
The following processes are done to generate output.
1. Generates a tempory based on a current frame and five old frames which allows the system can track unexpected movement.
2. Applies 'dilate()' method to allow the system can provide smooth lane movement on the display.
3. Applies 'bitwise_and()' method to apply the mask to filter out the outliers and then generates the output.

After generating the output, the system updates old image data to use the data with the next cycle.

Parameters
INPUT Previous_Frames * Old_Frames
=> Struct type which includes five old frames data
INPUT cv::Mat *input_Frame
=> Mat type input image data with one colour channel generates by 'image_Convertor' fucntion.
INPUT cv::Mat *input_Mask
=> Mat type input image data with one colour channel generates by 'Mask_Shaper' fucntion.
OUTPUT cv::Mat *outputFrame_Masked_Accumulated
=> Mat type output image data with one colour channel, threshold and mask applied.
*/
void frame_Accumulator(
	INPUT Previous_Frames * Old_Frames,
	INPUT cv::Mat *input_Frame,
	INPUT cv::Mat *input_Mask,
	OUTPUT cv::Mat *outputFrame_Masked_Accumulated
) {
	cv::Mat frame_temp;
	/*
	Defines Mat which is required to proceed 'dilate' method.
	The constants were defined based on several executions, 
	which could provide enough execution speed for the real-time process.
	*/
	static cv::Mat mask_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(1, 1));
	
	/*
	Generates the accumulated frame.
	To provide better execution speed, '|(or)' operation is used instead of '+(addition)'
	*/
	frame_temp =
		*input_Frame |
		Old_Frames->frame_Previous_1 |
		Old_Frames->frame_Previous_2 |
		Old_Frames->frame_Previous_3 |
		Old_Frames->frame_Previous_4 |
		Old_Frames->frame_Previous_5;

	/*
	Applies 'dilate()' and 'bitwise_and()' method 
	*/
	cv::dilate(frame_temp, frame_temp, mask_dilate, cv::Point(-1, -1), 3);
	cv::bitwise_and(frame_temp, *input_Mask, *outputFrame_Masked_Accumulated);

	/*
	Updates old image data.
	'copyTo()' method is used instead of '=' operation. 
	Because the function receives 'address', instead of 'copied data', 
	if '=' is used, every frame will be filled with the same image data.
	*/
	Old_Frames->frame_Previous_4.copyTo(Old_Frames->frame_Previous_5);
	Old_Frames->frame_Previous_3.copyTo(Old_Frames->frame_Previous_4);
	Old_Frames->frame_Previous_2.copyTo(Old_Frames->frame_Previous_3);
	Old_Frames->frame_Previous_1.copyTo(Old_Frames->frame_Previous_2);
	input_Frame->copyTo(Old_Frames->frame_Previous_1);
}


/*
Line_Drawer() function
This fucntion receives eight inputs and generates one output.
The function draws the following lines based on the input data.
-> Left buoy lane for the left image
-> Right buoy lane for the right image
-> Trajectory line for the merged image
Also, based on the input data, the function prints out the following message.
-> Angle required to turn to the boat direction to center.
-> Warning Sign that the buoy is not detected.

Parameters
INPUT Elements_Line_Specification * Input_Specification
=> Struct type input which includes line specification as following.
-> Buoy line thickness and colour
-> Trajectory line thickness and colour

INPUT LinearRegression_Element_NotInitialized * Input_Buoy_Num_Left
=> Struct type input which includes data about buoy detection for the left side.
INPUT LinearRegression_Element_NotInitialized * Input_Buoy_Num_Right
=> Struct type input which includes data about buoy detection for right side.

INPUT Elements_Lane_Coordinate * Input_Lane_Left
=> Struct type input which includes two points data 
which define the buoy lane starting and ending coordinate for the left side
INPUT Elements_Lane_Coordinate * Input_Lane_Right
=> Struct type input which includes two points data 
which define the buoy lane starting and ending coordinate for the right side
INPUT Elements_Trajectory_Coordinate * Input_Trajectory,
=> Struct type input which includes two points data 
which define the trajectory line starting and ending coordinate.

INPUT cv::Mat * inputFrame_LEFT
=> Mat type input image data with three colour channel for left side.
INPUT cv::Mat * inputFrame_RIGHT
=> Mat type input image data with three colour channel for right side.
InOut_Put cv::Mat * OutputFrame_Display
=> Mat type output image data with three colour channel that left and right sides are merged and includes required lines.
*/
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
) {
	static Size stringSize;
	static Point stringCenter;
	static int fontFace = FONT_HERSHEY_PLAIN;
	static double fontScale = 4;
	static double fontThickness = 4;
	static int fontBaseline;
	static int boatAngle;
		

	/*
	Draws the buoy lane based on the calculated buoy coordinate, and the buoy lane specification.
	*/
	cv::line(*inputFrame_LEFT,
		Input_Lane_Left->Lane_Coordinate_TOP,
		Input_Lane_Left->Lane_Coordinate_BOT,
		Input_Specification->Line_Colour_Buoy_Lane,
		Input_Specification->Line_Thickness_Buoy_Lane,
		LINE_AA,
		0);

	/*
	Prints a warning message if the system fails to detect enough buoy.
	*/
	if (Input_Buoy_Num_Left->Not_Enough_Buoy == true) {
		stringSave.str("");
		stringSave.clear();

		stringSave << "Not Enough Buoy Detected";

		stringPrint = stringSave.str();
		stringSize = getTextSize(stringPrint, fontFace, 2, 3, &fontBaseline);
		stringCenter = Point((image_midline - stringSize.width) / 2, image_row - image_row / 6);
		putText(*inputFrame_LEFT, stringPrint, stringCenter, fontFace, 2, cvScalar(0, 0, 255), 3, CV_AA);
	}
	
	cv::line(*inputFrame_RIGHT,
		Input_Lane_Right->Lane_Coordinate_TOP,
		Input_Lane_Right->Lane_Coordinate_BOT,
		Input_Specification->Line_Colour_Buoy_Lane,
		Input_Specification->Line_Thickness_Buoy_Lane,
		LINE_AA,
		0);

	if (Input_Buoy_Num_Right->Not_Enough_Buoy == true) {
		stringSave.str("");
		stringSave.clear();

		stringSave << "Not Enough Buoy Detected";

		stringPrint = stringSave.str();
		stringSize = getTextSize(stringPrint, fontFace, 2, 3, &fontBaseline);
		stringCenter = Point((image_midline - stringSize.width) / 2, image_row - image_row / 6);
		putText(*inputFrame_RIGHT, stringPrint, stringCenter, fontFace, 2, cvScalar(0, 0, 255), 3, CV_AA);
	}
	
	/*
	Merges Left and Right side to generate one big image.
	*/
	cv::hconcat(*inputFrame_LEFT, *inputFrame_RIGHT, *OutputFrame_Display);

	/*
	Draws the trajectory line based on the calculated buoy coordinate, and the trajectory line specification.
	*/
	cv::line(*OutputFrame_Display,
		Input_Trajectory->Trajectory_Coordinate_BOT,
		Input_Trajectory->Trajectory_Coordinate_TOP,
		Input_Specification->Line_Colour_Trajectory_Line,
		Input_Specification->Line_Thickness_Trajectory_Line,
		LINE_AA,
		0);


	/*
	Prints the message about the required rudder turn.
	*/
	boatAngle = Input_Trajectory->AngleToTurn;

	/*
	Clears the old message
	*/
	stringSave.str("");
	stringSave.clear();

	/*
	Assigns the message based on the calculated angle.
	*/
	if (boatAngle < 0) {
		stringSave << "Turn Left: " << abs(boatAngle) << " deg";
	}
	else if (boatAngle > 0) {
		stringSave << "Turn Right: " << boatAngle << " deg";
	}
	else {
		stringSave << "Heading Center";
	}

	/*
	Stores the message which will be displayed.
	*/
	stringPrint = stringSave.str();
	/*
	Retrieves the data which are required to positioning the message on the image.
	*/
	stringSize = getTextSize(stringPrint, fontFace, fontScale, fontThickness, &fontBaseline);
	stringCenter = Point((image_column - stringSize.width) / 2, image_row / 6);
	/*
	Prints the message on the image.
	*/
	putText(*OutputFrame_Display, stringPrint, stringCenter, fontFace, fontScale, cvScalar(0, 0, 255), fontThickness, CV_AA);
}
