#include"header_Final_V1.h"


/*
XY_Coordinate_Cal_Mask_Parallel() function.
This function receives three inputs and generates one output.
Based on the number of identifed buoys, the function defines size of masking area.
Based on the regression data and defined size of masking area(= radius), the system calculates the followings
-> Orthogonal slope of the input slope.
-> Two intercepts which allow the orthogonal slope can be intersected at the starting and ending coordinates of the buoy line.
-> Four coordinates which are laid on the calculated intersecting line.

After finding the four coordinate, the system checks the calculated value are not infinite number or not-a-number.
If the values pass the error check, the function updates the masking coordinates.

Parameters
INPUT Elements_Lane_Coordinate * InOne
Struct type input which includes the starting and ending coordinate of buoy lane.

INPUT LinearRegression_Element_NotInitialized * InTwo
=> Struct type input which includes data about buoy detection.

INPUT LinearRegression_Result_OneSide * In_Three
=> Struct type input which includes data about linear regression results

OUTPUT Elements_Masking_Coordinate * Output
=> Struct type output which includes four masking coordinate.
*/
void XY_Coordinate_Cal_Mask_Parallel(
	INPUT Elements_Lane_Coordinate * InOne,
	INPUT LinearRegression_Element_NotInitialized * InTwo,
	INPUT LinearRegression_Result_OneSide * In_Three,
	OUTPUT Elements_Masking_Coordinate * Output
) {
	/*
	The following section checks the number of identified buoy, and define radius to build masking area
	This allows the masking size of masking area can be vary based on number of defined buoy.
	*/
	int radius_Top;
	int radius_Bottom;

	if (InTwo->Num_identified_Buoy > 2) {
		radius_Top = 10;
		radius_Bottom = 30;
	}
	else if (InTwo->Num_identified_Buoy == 2) {
		radius_Top = 15;
		radius_Bottom = 35;
	}
	else if (InTwo->Num_identified_Buoy == 1) {
		radius_Top = 20;
		radius_Bottom = 40;
	}
	else {
		radius_Top = 25;
		radius_Bottom = 45;
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
		/*
		Finds the orthogonal slope.
		*/
		orthogonal_m_slope = -1 / In_Three->m_slope;

		/*
		Finds the two intercepts at the starting and ending coordinates of buoy lane.
		*/
		orthogonal_b_intercept_Top = InOne->Lane_Coordinate_TOP.y - orthogonal_m_slope * InOne->Lane_Coordinate_TOP.x;
		orthogonal_b_intercept_Bottom = InOne->Lane_Coordinate_BOT.y - orthogonal_m_slope * InOne->Lane_Coordinate_BOT.x;
		
		/*
		Finds the four coordinates which are laid on the calculated line.
		*/
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

		/*
		Checks the error cases
		'static_cast<double>' is used to convert 'int' type to 'double' type becase 
		'cvIsInf()' and 'cvIsNaN()' methods can be used with 'double' or 'float' type.
		'double' type is not used directly because when 'double(=Point2d)' type is used to define the coordinate, 
		the system showed unexpected behaviour.
		*/
		if (XY_One.x < 0 ||
			cvIsInf(static_cast<double>(XY_One.x)) == 1 || cvIsInf(static_cast<double>(XY_One.y)) == 1 ||
			cvIsNaN(static_cast<double>(XY_One.x)) == 1 || cvIsNaN(static_cast<double>(XY_One.y)) == 1) {
		}
		else {
			Output->Masking_Coordinate_One = XY_One;
		}

		if (XY_Two.x < 0 ||
			cvIsInf(static_cast<double>(XY_Two.x)) == 1 || cvIsInf(static_cast<double>(XY_Two.y)) == 1 ||
			cvIsNaN(static_cast<double>(XY_Two.x)) == 1 || cvIsNaN(static_cast<double>(XY_Two.y)) == 1) {
		}
		else {
			Output->Masking_Coordinate_Two = XY_Two;
		}

		if (cvIsInf(static_cast<double>(XY_Three.x)) == 1 || cvIsInf(static_cast<double>(XY_Three.y)) == 1 ||
			cvIsNaN(static_cast<double>(XY_Three.x)) == 1 || cvIsNaN(static_cast<double>(XY_Three.y)) == 1) {
		}
		else {
			Output->Masking_Coordinate_Three = XY_Three;
		}

		if (cvIsInf(static_cast<double>(XY_Four.x)) == 1 || cvIsInf(static_cast<double>(XY_Four.y)) == 1 ||
			cvIsNaN(static_cast<double>(XY_Four.x)) == 1 || cvIsNaN(static_cast<double>(XY_Four.y)) == 1) {
		}
		else {
			Output->Masking_Coordinate_Four = XY_Four;
		}
	}
}


/*
Mask_Shaper_Parallel() function
The function performs three operations.
1. Resets the old masking area.
2. Assigns the received masking coordinates with the required format to use 'fillConvexPoly()' function.
3. Uses 'fillConvexPoly()' to fill the masking area.

Parameters
INPUT Elements_Masking_Coordinate * Input
=> Struct type input which includes four masking coordinate.
InOut_Put cv::Mat *Mask
=> Mat type output with one colour channel.
*/
void Mask_Shaper_Parallel(
	INPUT Elements_Masking_Coordinate * Input,
	InOut_Put cv::Mat * Mask
) {
	/*
	Resets the old masking area.
	*/
	Mask->setTo(Scalar(0));

	/*
	Assigns the updates masking coordinates with required format.
	*/
	Point Masking_Coordinate[1][4];
	Masking_Coordinate[0][0] = Input->Masking_Coordinate_One;
	Masking_Coordinate[0][1] = Input->Masking_Coordinate_Two;
	Masking_Coordinate[0][2] = Input->Masking_Coordinate_Three;
	Masking_Coordinate[0][3] = Input->Masking_Coordinate_Four;
	const Point* Masking_Coordinate_list[2] = { Masking_Coordinate[0] };

	/*
	Uses 'fillConvexPoly()' to fill the masking area based on the updated masking coordinates.
	*/
	fillConvexPoly(*Mask, Masking_Coordinate[0], 4, Scalar(255), LINE_AA);
}