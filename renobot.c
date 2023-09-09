/*
Group 4-22
Authors: Vishesh Garg, Victor Kalenda, Vishan Muralikaran, Connor Switzer
Version 42.0

Description: This is the final product code of group 4-22 for the MTE 100/121
final project. This code begins with a menu screen which allows the user to
select a task for the Renobot to complete, this will then run the selected task,
and then return to the main menu.

Tasks include:
Calculate Area / Perimeter
Measure and draw an angle
Calculate expected number of various materials (Paint, drywall etc...)


Assumptions:

The Renobot drives counter clockwise around a space, you should orient the
ultrasonic sensor so that it faces an exterior wall of the room you want to
measure.

The wall must be opaque, relatively smooth and continuous.

There must not be any obstructions in the path of the Renobot.

Any space with width smaller than the width of the Renobot will not be measured.

The robot cannot be started facing a corner.

Failure to adhere to these suggestions may result in an inaccurate measurment.
*/

#include "UW_sensorMux.c"


typedef struct {
  float right;
  float left;
  float time;
  float proportional;
  float IR;
} past;

typedef struct {
  float initial_ang;
  float final_ang;
  float right;
  float left;
  float proportional;
  float integral;
  float derivative;
  float origin_tol;
  float IR;
} present;

typedef struct {
  float x;
  float y;
  int encoder;
} coordinate;

// Conversion Constants
const float DEG_TO_RAD = PI / 180;
const float RAD_TO_DEG = 180 / PI;
const float ENC_TO_DIST = 9 * PI / 1000;
const float DIST_TO_ENC = 1000 / (9 * PI);
const float CM2_TO_M2 = 1 / 10000.0;
const float CM_TO_M = 1 / 100.0;

// PID Constants
const float PID_TARGET = 10;
const float CONST_PROPORTIONAL = 2;
const float CONST_INTEGERAL = 10;
const float CONST_DERIVATIVE = 10;
const float ERROR_TOL = 10;
const int MUX_TOL = 5;

// Drive Constants
const float DRIVE = 40;
const float TRACE_DRIVE = 10;
const float TURN_DRIVE = 25;
const int ENCODER_OFFSET = 5;
const float RENO_WIDTH = 22.5;
const float MARGINAL_DIST = -5;
const float MARGINAL_ANG = -15;
const float DIST_TOL = 7;
const float TIME_GAP = 500;
const float ACUTE_MARGINAL_ANG = -5;
const float ACUTE_MARGINAL_DIST = -5;
const float ANG_TOL = 0.4;

// Origin Constants
const float ORIGIN_FACTOR = 1 / 5.5;
const int ORIGIN_SHIFT = 4;

// IR Constants
const int IR_DRIVE_MOTOR_POWER = 70;
const float IR_CONST_PROPORTIONAL = 0.5;
const float IR_CONST_INTEGERAL = 1.5;
const float IR_CONST_DERIVATIVE = 0.1;
const float IR_ERROR_TOL = 0;
const int IR_TIME_TOL = 5;
const int IR_POWER_MAX = 50;
const int IR_RESET_ANGLE = 360;
const int IR_RESET_TOL = 5;
const int IR_TURN_MODE = 0;
const float IR_AREA_OFFSET_FACTOR = 1;
const float IR_PERIM_OFFSET_FACTOR = 1;

// Coordinate Process Constants
const float TIRE_TO_WALL = 3;
const float AREA_OFFSET_FACTOR = 1;
const float PERIM_OFFSET_FACTOR = 1;


// Pencil Constants
const int PENCIL_INCREMENT = 55;
const int PENCIL_POWER = 20;
const int MAX_SIZE = 120;

// Array for coordinate data collection (Used for Draw_corner function)
coordinate data[MAX_SIZE];


// Main Menu Function Prototypes
bool checkForArea(float closedAreaCalc, float closedPerimCalc, float openAreaCalc, float openPerimCalc, string typeOfArea);
int mainMenuScreen(float openArea, float openPerimeter, float closedArea, float closedPerimeter);
float surfaceAreaOfWalls(float heightOfRoom, float closedPerimeter);
void findTotalStud(float closedPerimeter);
void paintingWalls(float surfaceArea);
void baseboard(float closedPerimeter);
int getButtonValue(int numOfOptions);
void drywall(float surfaceArea);
void concrete(float openArea, float thicknessOfPavement);
int getHeight(int interval);

// Area Perimeter Function Prototypes
void configure_sensors(void);
void drive(float left_pow, float right_pow);
void turn_left(int &mode, float &area, float &perimeter, float &origin_tol, present &curr, coordinate &C1, coordinate &C2, bool &roomFinished);
void correct_path(past &prev, present &curr);
void drive_dist(float motor_pow, float dist);
void rotate(float motor_pow, float ang);
void drive_to_wall(int &mode, float &initial_dist);
void Area_Perimeter(float &area, float &perimeter, past &prev, present &curr);
bool coordinate_process(int turn_mode, present & curr, float &area, float &perimeter, float &origin_tol, coordinate &C1, coordinate &C2);
void reset_variables(past & prev, present & curr, float & area, float & perimeter, coordinate & C1, coordinate & C2);
void calibrate_gyro(void);

// Draw Corner Function Prototypes
void rotate_gyro(float motor_pow, float ang);
void map_corner(int & data_size, past & prev, present & curr);
void trace_corner(int & data_size);
void Draw_Corner(past & prev, present & curr);
void set_pencil(bool action);
void wait_read(void);

// IR Function Prototypes
void Remote_Area_Perimeter(float & area, float & perimeter, past & prev, present & curr);
void correct_IR(past & prev, present & curr);
void remote_drive();
void reset_IR();
void mission_possible();



task main() 
{
  bool shutDown = false;

  float openAreaCalc = 0;
  float openPerimCalc = 0;
  float closedAreaCalc = 0;
  float closedPerimCalc = 0;
  float heightOfRoom = 1;
  float thicknessOfPavement = 1;

  present curr;
  past prev;
  coordinate C1, C2;

  reset_variables(prev, curr, closedAreaCalc, closedPerimCalc, C1, C2);

  // Runs code while shut down option isn't pressed
  while (shutDown == false) 
  {
    bool returnToMain = false;
    while (returnToMain == false) 
    {
      // Main Menu Screen
      int buttonValue =
          mainMenuScreen(openAreaCalc, openPerimCalc, closedAreaCalc, closedPerimCalc); // change later
      eraseDisplay();

      // Area and Perimeter Screen
      while (buttonValue == 1 && returnToMain == false) 
      {
        int buttonValue = 1;
        string areaDisplayOption[2] = {" Enclosed", " Open"};
        displayBigTextLine(5, "%d %s", buttonValue, areaDisplayOption[buttonValue - 1]);

        displayBigTextLine(3, "Area/Perimeter");
        displayBigTextLine(8, "1 Enclosed");
        displayBigTextLine(10, "2 Open");

        int areaButtonValue = getButtonValue(2);

        if (areaButtonValue == 1) 
        {
          eraseDisplay();
          wait_read();
          eraseDisplay();
          displayBigTextLine(3, "Calculating");
          displayBigTextLine(5, "Area and");
          displayBigTextLine(7, "Perimeter ...");
          Area_Perimeter(closedAreaCalc, closedPerimCalc, prev, curr);
          eraseDisplay();
          displayBigTextLine(3, "Area:");
          displayBigTextLine(5, "%.2fm^2", closedAreaCalc);
          displayBigTextLine(9, "Perimeter:");
          displayBigTextLine(11, "%.2fm", closedPerimCalc);
          while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight))
          {}
          while (!getButtonPress(buttonEnter) && !getButtonPress(buttonLeft) && !getButtonPress(buttonRight)) 
          {}
          returnToMain = true;
          eraseDisplay();
        } 
        else if (areaButtonValue == 2) 
        {
          eraseDisplay();
          wait_read();
          eraseDisplay();
          displayBigTextLine(3, "Calculating");
          displayBigTextLine(5, "Area and");
          displayBigTextLine(7, "Perimeter ...");
          Remote_Area_Perimeter(openAreaCalc, openPerimCalc, prev, curr);
          eraseDisplay();
          displayBigTextLine(3, "Area:");
          displayBigTextLine(5, "%.2fm^2", openAreaCalc);
          displayBigTextLine(9, "Perimeter:");
          displayBigTextLine(11, "%.2fm", openPerimCalc);
          while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
          {}
          while (!getButtonPress(buttonEnter) && !getButtonPress(buttonLeft) && !getButtonPress(buttonRight)) 
          {}
          returnToMain = true;
          eraseDisplay();
        }

        if (getButtonPress(buttonLeft)) 
        {
          returnToMain = true;
          while (getButtonPress(buttonLeft)) 
          {}
        }
      }

      // Map Angle Function
      if (buttonValue == 2 && returnToMain == false) 
      {
        eraseDisplay();
        displayBigTextLine(3, "Calculating");
        displayBigTextLine(5, "Angle ...");
        Draw_Corner(prev, curr);
        eraseDisplay();
        displayBigTextLine(3, "Angle");
        displayBigTextLine(5, "Calculated!");
        while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
        {}
        while (!getButtonPress(buttonEnter) && !getButtonPress(buttonLeft) && !getButtonPress(buttonRight)) 
        {}
        returnToMain = true;
        eraseDisplay();
      }

      // Materials Screen
      while (buttonValue == 3 && returnToMain == false) 
      {
        int buttonValue = 1;
        int materialsButtonValue = 0;
        string materialDisplayOption[5] = {" Paint", " Drywall", " Baseboard", " Studs", " Concrete"};
        displayBigTextLine(3, "%d %s", buttonValue, materialDisplayOption[buttonValue - 1]);

        displayBigTextLine(1, "   Materials");
        displayBigTextLine(6, "1 Paint");
        displayBigTextLine(8, "2 Drywall");
        displayBigTextLine(10, "3 Baseboard");
        displayBigTextLine(12, "4 Studs");
        displayBigTextLine(14, "5 Concrete");

        string typeOfArea = " ";
        bool hasAreaAndPerimeter =
            checkForArea(closedAreaCalc, closedPerimCalc, openAreaCalc, openPerimCalc, typeOfArea);
        if (hasAreaAndPerimeter) 
        {
          materialsButtonValue = getButtonValue(5);
        } 
        else 
        {
          returnToMain = true;
        }

        if (materialsButtonValue == 1) // Gets surface area, paint time, and amount of paint
        {
          typeOfArea = " Closed ";
          hasAreaAndPerimeter = checkForArea(0, closedPerimCalc, 0, 0, typeOfArea);
          if (hasAreaAndPerimeter) 
          {
            eraseDisplay();
            displayBigTextLine(1, "Enter height");
            displayBigTextLine(3, "of the room: ");
            displayBigTextLine(7, "0 feet");
            heightOfRoom = getHeight(1);
            while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
            {}
            eraseDisplay();
            if (heightOfRoom != 0 && !getButtonPress(buttonLeft)) 
            {
              paintingWalls(surfaceAreaOfWalls(heightOfRoom, closedPerimCalc));
              while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
              {}
              while (!getButtonPress(buttonEnter) && !getButtonPress(buttonLeft) && !getButtonPress(buttonRight)) 
              {}
              eraseDisplay();
            }
          } 
          else 
          {
            returnToMain = true;
          }
        } 
        else if (materialsButtonValue == 2) // Gets amount of drywall needed for surface area
        {
          typeOfArea = " Closed ";
          hasAreaAndPerimeter = checkForArea(0, closedPerimCalc, 0, 0, typeOfArea);
          if (hasAreaAndPerimeter) 
          {
            eraseDisplay();
            displayBigTextLine(1, "Enter height");
            displayBigTextLine(3, "of the room: ");
            displayBigTextLine(7, "0 feet");
            heightOfRoom = getHeight(1);
            while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
            {}
            eraseDisplay();
            if (heightOfRoom != 0 && !getButtonPress(buttonLeft)) 
            {
              drywall(surfaceAreaOfWalls(heightOfRoom, closedPerimCalc));
              while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
              {}
              while (!getButtonPress(buttonEnter) && !getButtonPress(buttonLeft) && !getButtonPress(buttonRight)) 
              {}
              eraseDisplay();
            }
          } 
          else 
          {
            returnToMain = true;
          }
        } 
        else if (materialsButtonValue == 3) // Gets amount of baseboard needed for surface area
        {
          typeOfArea = " Closed ";
          hasAreaAndPerimeter = checkForArea(0, closedPerimCalc, 0, 0, typeOfArea);
          if (hasAreaAndPerimeter) 
          {
            eraseDisplay();
            baseboard(closedPerimCalc);
            while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
            {}
            while (!getButtonPress(buttonEnter) && !getButtonPress(buttonLeft) && !getButtonPress(buttonRight)) 
            {}
            eraseDisplay();
          } 
          else 
          {
            returnToMain = true;
          }
        } 
        else if (materialsButtonValue == 4) // Gets amount of studs in wall
        {
          typeOfArea = " Closed ";
          hasAreaAndPerimeter = checkForArea(0, closedPerimCalc, 0, 0, typeOfArea);
          if (hasAreaAndPerimeter) 
          {
            eraseDisplay();
            findTotalStud(closedPerimCalc);
            while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
            {}
            while (!getButtonPress(buttonEnter) && !getButtonPress(buttonLeft) && !getButtonPress(buttonRight)) 
            {}
            eraseDisplay();
          } 
          else 
          {
            returnToMain = true;
          }
        } 
        else if (materialsButtonValue == 5) 
        {
          typeOfArea = " Open ";
          hasAreaAndPerimeter = checkForArea(0, 0, openAreaCalc, 0, typeOfArea);
          if (hasAreaAndPerimeter) 
          {
            eraseDisplay();
            displayBigTextLine(1, "Enter thickness:");
            displayBigTextLine(7, "0 feet");
            thicknessOfPavement = getHeight(1);
            while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
            {}
            eraseDisplay();
            if (thicknessOfPavement != 0 && !getButtonPress(buttonLeft)) 
            {
              concrete(openAreaCalc, thicknessOfPavement);
              while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
              {}
              while (!getButtonPress(buttonEnter) && !getButtonPress(buttonLeft) && !getButtonPress(buttonRight)) 
              {}
              eraseDisplay();
            }
          } 
          else 
          {
            returnToMain = true;
          }
        }

        if (getButtonPress(buttonLeft)) 
        {
          returnToMain = true;
          while (getButtonPress(buttonLeft)) 
          {}
        }
      }

      // Shut down
      if (buttonValue == 4 && returnToMain == false) 
      {
        returnToMain = true;
        shutDown = true;
      }
    }
  }
}

/************************************************************************
Displays main menu screen 
************************************************************************/
int mainMenuScreen(float openArea, float openPerimeter, float closedArea, float closedPerimeter)
{
  eraseDisplay();
  int buttonValue = 1;
  string menuDisplayOption[4] = {" Area", " Draw Angle", " Materials", " Shut Down"};

  // Main Menu Screen
  displayBigTextLine(3, "  Select Mode");
  displayBigTextLine(8, "1 Area");
  displayBigTextLine(10, "2 Draw Angle");
  displayBigTextLine(12, "3 Materials");
  displayBigTextLine(14, "4 Shut Down");

  displayBigTextLine(5, "%d %s", buttonValue, menuDisplayOption[buttonValue - 1]);
  buttonValue = getButtonValue(4);

  return buttonValue;
}

/********************************************************
Find surface area of walls
********************************************************/
float surfaceAreaOfWalls(float heightOfRoom, float closedPerimeter) 
{
  const float feetToMeters = 0.3048;
  float heightToMeters = heightOfRoom * feetToMeters;
  float surfaceArea = heightToMeters * closedPerimeter;
  return surfaceArea;
}

/********************************************************
Find paint time and amount function
*********************************************************/
void paintingWalls(float surfaceArea) 
{
  const int PAINT_PER_LITER = 6;
  const float LITER_TO_GAL = 3.785;
  const int AVERAGE_PAINT_PER_HOUR = 7;

  float totalPaintLiters = surfaceArea / PAINT_PER_LITER;
  float totalPaintGal = totalPaintLiters / LITER_TO_GAL;

  float paintTime = surfaceArea / AVERAGE_PAINT_PER_HOUR;

  displayBigTextLine(1, "Surface area:");
  displayBigTextLine(3, "%.2fm^2", surfaceArea);
  displayBigTextLine(6, "Paint Time:");
  displayBigTextLine(8, "%.2f hours", paintTime);
  displayBigTextLine(11, "Liters: %.2f", totalPaintLiters);
  displayBigTextLine(14, "Gallons: %.2f", totalPaintGal);
}

/*********************************************************
Drywall Function
*********************************************************/
void drywall(float surfaceArea) 
{
  float drywallConversion = 2.88;
  float amountOfDrywall = surfaceArea / drywallConversion;

  displayBigTextLine(1, "Surface area:");
  displayBigTextLine(3, "%.2fm^2", surfaceArea);
  displayBigTextLine(7, "# of Boards: ");
  displayBigTextLine(9, "%.2f", amountOfDrywall);
}

/***********************************************************
Baseboard Function
***********************************************************/
void baseboard(float closedPerimeter) 
{
  int amountOfBaseboards = closedPerimeter;

  displayBigTextLine(1, "Baseboards: ");
  displayBigTextLine(3, "%.2f", amountOfBaseboards);
}

// Find total amount of studs
void findTotalStud(float closedPerimeter) 
{
  const float AMOUNT_OF_STUDS_DISTANCE = 0.4064;
  int totalAmountOfStuds = ceil(closedPerimeter / AMOUNT_OF_STUDS_DISTANCE);

  displayBigTextLine(3, "Estimated Studs: ");
  displayBigTextLine(6, "%d", totalAmountOfStuds);
}
/**************************************************
Finds Total Amount of Concrete Needed
***************************************************/
void concrete(float openArea, float thicknessOfPavement) 
{
  float areaVolume = openArea * thicknessOfPavement;
  float concreteConversion = 5.4;
  float concreteAmount = areaVolume / concreteConversion;

  displayBigTextLine(3, "# of Concrete: ");
  displayBigTextLine(6, "%dm^3", concreteAmount);
}

/********************************************************
Gets Height of Room with Button Presses
*********************************************************/
int getHeight(int interval) 
{
  int buttonValue = 0;

  while (getButtonPress(buttonEnter) || getButtonPress(buttonRight)) 
  {}
  while (!getButtonPress(buttonEnter) && !getButtonPress(buttonRight)) 
  {
    if (getButtonPress(buttonLeft)) // Leaves and changes nothing in task main when left button is pressed
    {
      return 0;
    }
    if (getButtonPress(buttonDown)) 
    {
      if (buttonValue > 0) 
      {
        buttonValue -= interval;
        displayBigTextLine(7, "%d feet", buttonValue);
        while (getButtonPress(buttonDown)) 
        {}
      }
    } 
    else if (getButtonPress(buttonUp)) 
    {
      if (buttonValue < 1000) 
      {
        buttonValue += interval;
        displayBigTextLine(7, "%d feet", buttonValue);
        while (getButtonPress(buttonUp)) 
        {}
      }
    }
  }
  return buttonValue;
}

/**************************************************************************
Checks To See If There Are Values For Area And Perimeter
***************************************************************************/
bool checkForArea(float closedAreaCalc, float closedPerimCalc, float openAreaCalc, float openPerimCalc, string typeOfArea) 
{
  if (closedAreaCalc != 0 || closedPerimCalc != 0 || openAreaCalc != 0 || openPerimCalc != 0) 
  {
    return true;
  } 
  else 
  {
    eraseDisplay();
    displayBigTextLine(3, "No%sArea", typeOfArea);
    displayBigTextLine(5, "and Perimeter ");
    displayBigTextLine(7, "Found.");
    displayBigTextLine(11, "Run%sArea", typeOfArea);
    displayBigTextLine(13, "Function");
    while (getButtonPress(buttonEnter) || getButtonPress(buttonLeft) || getButtonPress(buttonRight)) 
    {}
    while (!getButtonPress(buttonEnter) && !getButtonPress(buttonLeft) && !getButtonPress(buttonRight))
    {}
    eraseDisplay();
    return false;
  }
}

/********************************************************************************************
Changes the button values with respected menu values and returns the value when
pressed
********************************************************************************************/
int getButtonValue(int numOfOptions) 
{
  int buttonValue = 1;

  // Can change to passing in an array
  string menuDisplayOption[4] = {" Area", " Draw Angle", " Materials",
                                 " Shut Down"};
  string materialDisplayOption[5] = {" Paint", " Drywall", " Baseboard",
                                     " Studs", " Concrete"};
  string areaDisplayOption[2] = {" Enclosed", " Open"};

  string displayOption = menuDisplayOption[buttonValue - 1];

  while (getButtonPress(buttonEnter) || getButtonPress(buttonRight)) 
  {} // When button is released, run the rest of the code (only for press left)
  while (!getButtonPress(buttonEnter) && !getButtonPress(buttonRight)) 
  {
    if (getButtonPress(buttonLeft)) // Leaves and changes nothing in task main when left button is pressed
    {
      return 0;
    }
    if (getButtonPress(buttonUp)) 
    {
      if (buttonValue > 0) 
      {
        buttonValue--;
        if (buttonValue == 0) 
        {
          buttonValue = numOfOptions;
        }
        if (numOfOptions == 4) 
        {
          displayOption = menuDisplayOption[buttonValue - 1];
          displayBigTextLine(5, "%d %s", buttonValue, displayOption);
        } 
        else if (numOfOptions == 5)
        {
          displayOption = materialDisplayOption[buttonValue - 1];
          displayBigTextLine(3, "%d %s", buttonValue, displayOption);
        } 
        else if (numOfOptions == 2) 
        {
          displayOption = areaDisplayOption[buttonValue - 1];
          displayBigTextLine(5, "%d %s", buttonValue, displayOption);
        }
      }
      while (getButtonPress(buttonUp))
      {}
    } 
    else if (getButtonPress(buttonDown)) 
    {
      if (buttonValue < numOfOptions + 1) 
      {
        buttonValue++;
        if (buttonValue == numOfOptions + 1) 
        {
          buttonValue = 1;
        }
        if (numOfOptions == 4) 
        {
          displayOption = menuDisplayOption[buttonValue - 1];
          displayBigTextLine(5, "%d %s", buttonValue, displayOption);
        } 
        else if (numOfOptions == 5) 
        {
          displayOption = materialDisplayOption[buttonValue - 1];
          displayBigTextLine(3, "%d %s", buttonValue, displayOption);
        } 
        else if (numOfOptions == 2) 
        {
          displayOption = areaDisplayOption[buttonValue - 1];
          displayBigTextLine(5, "%d %s", buttonValue, displayOption);
        }
      }
      while (getButtonPress(buttonDown)) 
      {}
    }
  }
  return buttonValue;
}

// ----------------------- END OF MAIN MENU FUNCTIONS ------------------------------------------------
// ---------------------------------------------------------------------------------------------------

void Area_Perimeter(float &area, float &perimeter, past &prev, present &curr)
{
  configure_sensors();
  calibrate_gyro();
  bool roomFinished = false;
  time1[T1] = 0; // PID
  time1[T2] = 0; // Area Perim
  time1[T3] = 0; // IR PID

  coordinate C1, C2;
  reset_variables(prev, curr, area, perimeter, C1, C2);

  int turn_mode = 0;
  int index = 0;
  int index1 = 0;
  float origin_tol = 0;

  // Drive out of the tolerance of the origin vector
  drive(DRIVE, DRIVE);
  wait1Msec(1000);



  // find the initial gyro reading to offset all calculations accordingly

  // Drive until you complete the room
  while (!roomFinished && !getButtonPress(buttonEnter)) 
  {
    if (time1[T1] > index) 
    {
      turn_mode = 0;
      curr.initial_ang = getGyroDegrees(S2) * DEG_TO_RAD;
      correct_path(prev, curr);

      if (readMuxSensor(msensor_S1_3) == 1) 
      {
        drive(0, 0);
        turn_mode = 1;
        turn_left(turn_mode, area, perimeter, origin_tol, curr, C1, C2, roomFinished);

      } 
      else if (readMuxSensor(msensor_S1_1) == 1)
      {
        drive(0, 0);
        turn_mode = 2;
        turn_left(turn_mode, area, perimeter, origin_tol, curr, C1, C2, roomFinished);
      }
      index = time1[T1] + MUX_TOL;

    }
    // Calculate x2, y2, and process the perimeter and area
    if (time1[T2] > index1) 
    {
    	curr.final_ang = getGyroDegrees(S2) * DEG_TO_RAD;
      roomFinished = coordinate_process(turn_mode, curr, area, perimeter, origin_tol, C1, C2);
      index1 = time1[T2] + TIME_GAP;
    }
  }
  while (getButtonPress(buttonEnter)) 
  {}

  area = (pow(fabs(area), AREA_OFFSET_FACTOR)) * CM2_TO_M2;
  perimeter = (pow(fabs(perimeter + origin_tol), PERIM_OFFSET_FACTOR)) * CM_TO_M;
  drive(0, 0);
  mission_possible();
}

bool coordinate_process(int turn_mode, present & curr, float &area, float &perimeter, float &origin_tol, coordinate &C1, coordinate &C2)
{
  if (fabs(area) > ORIGIN_SHIFT)
        origin_tol = ORIGIN_FACTOR * sqrt(fabs(area) - ORIGIN_SHIFT);
  // Because we are driving counter-clockwise

  // Note that all the display commands that are commented out are for testing purposes.

  //curr.initial_ang = curr.initial_ang * DEG_TO_RAD;
  //curr.final_ang = curr.final_ang * DEG_TO_RAD;

  C2.encoder = nMotorEncoder[motorA];

  // displayBigTextLine(2, "init Ang = %f", curr.initial_ang);
  // displayBigTextLine(4, "fin Ang = %f", curr.final_ang );
  // displayBigTextLine(6, "Gyr Ang = %f", getGyroDegrees(S2));

  // If driving alongside a wall or in any right turn
  if (turn_mode == 0) 
  {
    C2.x = C1.x + (-(C2.encoder - C1.encoder) * ENC_TO_DIST) * cos(curr.final_ang);
    C2.y = C1.y + (-(C2.encoder - C1.encoder) * ENC_TO_DIST) * sin(curr.final_ang);
  }

  // If in an obtuse left turn
  else if (turn_mode == 1) 
  {
    C2.x = C1.x + TIRE_TO_WALL * cos(curr.initial_ang);
    C2.y = C1.y + TIRE_TO_WALL * sin(curr.initial_ang);
  }

  // If in an acute left turn
  // else if (turn_mode == 2)
  else
  {
		float interior_ang = PI - curr.final_ang + curr.initial_ang;
		C2.x = C1.x + (RENO_WIDTH * cos(interior_ang) / sin(interior_ang) + TIRE_TO_WALL) * cos(curr.initial_ang);
		C2.y = C1.y + (RENO_WIDTH * cos(interior_ang) / sin(interior_ang) + TIRE_TO_WALL) * cos(curr.initial_ang);
  }
  //displayBigTextLine(10, "X = %f", C2.x);
  //displayBigTextLine(12, "Y = %f", C2.y);

  C1.encoder = C2.encoder;

  // Beginning of Point_process---------------------------------------------
  // Calculate area with Shoelace algorithm
  area += ((C1.x * C2.y) - (C1.y * C2.x)) / 2;

  // Calculate perimeter
  perimeter += sqrt(pow(C2.y - C1.y, 2) + pow(C2.x - C1.x, 2));

  // Check if you have returned to the origin
  if (fabs(area) > ORIGIN_SHIFT) 
  {
    if (fabs(C2.x) < origin_tol && fabs(C2.y) < origin_tol)
      return true;
  }
  C1.x = C2.x;
  C1.y = C2.y;

  return false;
}

void turn_left(int &mode, float &area, float &perimeter, float &origin_tol, present &curr, coordinate &C1, coordinate &C2, bool &roomFinished)
{
  float initial_dist = 0;
  // Right touch sensor hit, angles = A, C, I
  while (mode != 0) 
  {
    initial_dist = fabs(nMotorEncoder[motorA] * ENC_TO_DIST);
    if (mode == 1) 
    {
      // Obtuse Angle
      while (readMuxSensor(msensor_S1_3) == 1) 
      {
      	curr.initial_ang = getGyroDegrees(S2) * DEG_TO_RAD;
      	roomFinished = coordinate_process(mode, curr, area, perimeter, origin_tol, C1, C2);
        drive_dist(TURN_DRIVE, MARGINAL_DIST);
        rotate(TURN_DRIVE, MARGINAL_ANG);
        drive_to_wall(mode, initial_dist);
       }
    }
    // Acute Angle
    // if (mode == 2)
    else 
    {
    	curr.initial_ang = getGyroDegrees(S2) * DEG_TO_RAD;
    	while(readMuxSensor(msensor_S1_3) == 0 || readMuxSensor(msensor_S1_1) == 1)
    	{
      	drive_dist(TURN_DRIVE, ACUTE_MARGINAL_DIST);
      	rotate(TURN_DRIVE, ACUTE_MARGINAL_ANG);
      	drive_to_wall(mode, initial_dist);
    	}
    	mode = 1;
    	curr.final_ang = getGyroDegrees(S2) * DEG_TO_RAD;
    	roomFinished = coordinate_process(mode, curr, area, perimeter, origin_tol, C1, C2);
    }
  }
}


void drive_to_wall(int &mode, float &initial_dist) 
{
  drive(TURN_DRIVE, TURN_DRIVE);

  if (mode == 2)
  {
  	while (readMuxSensor(msensor_S1_1) == 0 && readMuxSensor(msensor_S1_3) == 0)
  	{
  		wait1Msec(MUX_TOL);
  	}
  }
  else
  {
  	while (readMuxSensor(msensor_S1_3) == 0 && fabs(nMotorEncoder[motorA] * ENC_TO_DIST) - initial_dist < DIST_TOL) 
    {
      wait1Msec(MUX_TOL);
  	}
  }
  drive(0, 0);

  if (mode != 2)
  {
  	if (fabs(nMotorEncoder[motorA] * ENC_TO_DIST) - initial_dist > DIST_TOL)
  	{
    	mode = 0;
  	}
  	else
  	{
    	mode = 1;
  	}
  }
}


// PID control algorithm to hold renobot a defined distance from the perimeter by adjusting power on each tire
// Ultrasonic sensor measures the distance of the renobot from the wall
void correct_path(past &prev, present &curr) 
{
  float control_signal = 0;
  float magnitude_of_power = 0;
  float error = 0;

  // Calculate the change in time (in milliseconds)
  float curr_time = time1[T1];
  float delta_time = curr_time - prev.time;
  prev.time = curr_time;

  // Calculate info on the error/time graph
  error = SensorValue[S3] - PID_TARGET;
  curr.proportional = fabs(error);
  curr.integral = (curr.integral + curr.proportional) / delta_time;
  curr.derivative = (curr.proportional - prev.proportional) / delta_time;
  prev.proportional = curr.proportional;

  control_signal = CONST_PROPORTIONAL * curr.proportional + CONST_INTEGERAL * curr.integral + CONST_DERIVATIVE * curr.derivative;
  magnitude_of_power = fabs(control_signal);

  if (error >= 0 && error < ERROR_TOL) 
  {
    curr.right = DRIVE - magnitude_of_power;
    curr.left = DRIVE + magnitude_of_power;
  } 
  else if (error < 0) 
  {
    curr.right = DRIVE + 2 * magnitude_of_power;
    curr.left = DRIVE - 2 * magnitude_of_power;
  } 
  else 
  {
    curr.right = 1 / 3.0 * DRIVE;
    curr.left = DRIVE + DRIVE * 1 / 3.0;
  }

  if (prev.right != curr.right) 
  {
    drive(curr.left, curr.right);
  }
  prev.right = curr.right;
  prev.left = curr.left;
}


void drive_dist(float motor_pow, float dist) 
{
  int orig_enc = nMotorEncoder[motorA];
  int target_encoder = 0;

  target_encoder = orig_enc - (dist * DIST_TO_ENC);

  if (target_encoder > orig_enc) 
  {
    drive(-motor_pow, -motor_pow);

    while (target_encoder > nMotorEncoder[motorA] + ENCODER_OFFSET) 
    {}
  } 
  else 
  {
    drive(motor_pow, motor_pow);

    while (target_encoder < nMotorEncoder[motorA] - ENCODER_OFFSET) 
    {}
  }
  drive(0, 0);
}


void drive(float left_pow, float right_pow) 
{
  motor[motorA] = -1 * right_pow;
  motor[motorD] = -1 * left_pow;
}


void rotate(float motor_pow, float ang) 
{
  int orig_ang = SensorValue[S2];
  int target_ang = 0;

  target_ang = orig_ang + ang;

  if (target_ang > orig_ang) 
  {
    drive(motor_pow, -motor_pow);
    while (target_ang > SensorValue[S2]) 
    {}
  } 
  else 
  {
    drive(-motor_pow, motor_pow);
    while (target_ang < SensorValue[S2]) 
    {}
  }
  drive(0, 0);
}

void calibrate_gyro(void)
{
	wait1Msec(100);
	SensorMode[S2] = modeEV3Gyro_Calibration;
  wait1Msec(100);
  SensorMode[S2] = modeEV3Gyro_RateAndAngle;
  wait1Msec(100);
  resetGyro(S2);
}

void configure_sensors(void) 
{
  SensorType[S1] = sensorEV3_GenericI2C;
  wait1Msec(100);
  if (!initSensorMux(msensor_S1_1, touchStateBump))
    return;
  if (!initSensorMux(msensor_S1_3, touchStateBump))
    return;

  SensorType[S2] = sensorEV3_Gyro;
  wait1Msec(50);
  calibrate_gyro();
  SensorType[S3] = sensorEV3_Ultrasonic;
  wait1Msec(50);
  SensorType[S4] = sensorEV3_IRSensor;
  wait1Msec(50);
}

void reset_variables(past & prev, present & curr, float & area, float & perimeter, coordinate & C1, coordinate & C2)
{
  curr.left = 0;
  curr.right = 0;
  curr.proportional = 0;
  curr.integral = 0;
  curr.derivative = 0;
  curr.initial_ang = 0;
  curr.final_ang = 0;
  curr.IR = 0;

  prev.left = 0;
  prev.right = 0;
  prev.proportional = 0;
  prev.time = 0;
  prev.IR = 0;

  area = 0;
  perimeter = 0;

  C1.x = 0;
  C1.y = 0;
  C1.encoder = 0;
  C2.x = 0;
  C2.y = 0;
  C2.encoder = 0;
  nMotorEncoder[motorA] = 0;
  nMotorEncoder[motorD] = 0;
}

// -------------------- Draw Angle Functions --------------------------------

void Draw_Corner(past & prev, present & curr)
{

	int data_size = 0;
	wait_read();
	configure_sensors();
	calibrate_gyro();
	map_corner(data_size, prev, curr);
	wait_read();
	set_pencil(true);
	calibrate_gyro();
	trace_corner(data_size);
	set_pencil(false);
}

void map_corner(int & data_size, past & prev, present & curr)
{
  time1[T1] = 0;
  time1[T2] = 0;
  int index = 0;
  int index1 = 0;
  int turn_mode = 0;

  float perimeter = 0;
  float area = 0;

  float origin_tol = 0;
  bool roomFinished = false;

  coordinate C1, C2, C3;
  C3.x = 0; C3.y = 0; C3.encoder = 0;

  reset_variables(prev, curr, area, perimeter, C1, C2);

  while (!getButtonPress(buttonEnter))
  {
	if (time1[T1] > index) 
  {
      turn_mode = 0;
      curr.initial_ang = getGyroDegrees(S2) * DEG_TO_RAD;
      correct_path(prev, curr);

      if (readMuxSensor(msensor_S1_3) == 1) 
      {
        drive(0, 0);
        turn_mode = 1;
        turn_left(turn_mode, area, perimeter, origin_tol, curr, C1, C2, roomFinished);

      } 
      else if (readMuxSensor(msensor_S1_1) == 1) 
      {
        drive(0, 0);
        turn_mode = 2;
        turn_left(turn_mode, area, perimeter, origin_tol, curr, C1, C2, roomFinished);
      }
      index = time1[T1] + MUX_TOL;
    }

    // Calculate coordinate
    if (time1[T2] > index1)
    {
    	curr.final_ang = getGyroDegrees(S2) * DEG_TO_RAD;
    	C3.x = C1.x;
    	C3.y = C1.y;
    	C3.encoder = C1.encoder;
      roomFinished = coordinate_process(turn_mode, curr, area, perimeter, origin_tol, C1, C2);

	  	if(fabs(atan2(C2.y, C2.x) - atan2(C3.y, C3.x)) * RAD_TO_DEG > ANG_TOL || getButtonPress(buttonEnter))
			{
				data[data_size].x = C2.x;
      	data[data_size].y = C2.y;

  			data_size++;
  		}
			index1 = time1[T2] + TIME_GAP;
		}
	}
	drive(0,0);
	while(getButtonPress(buttonEnter))
	{}
}


void trace_corner(int & data_size)
{
	float angle = 0;
	float dist = 0;

  for(int index = 0; index < data_size; index++)
  {
  	if((index - 1) < 0)
  	{
  		angle = atan2(data[index].x, -data[index].y) * RAD_TO_DEG -90;
  		dist = sqrt(pow(data[index].x, 2) + pow(data[index].y, 2));

  	}
  	else
  	{
  		angle = atan2((data[index].x - data[index - 1].x), (-data[index].y + data[index - 1].y)) * RAD_TO_DEG -90;
  		dist = sqrt(pow(data[index].x - data[index - 1].x, 2) + pow(data[index].y - data[index - 1].y, 2));

  	}
  	rotate_gyro(TURN_DRIVE, angle);
    drive_dist(TRACE_DRIVE, dist);
  }
}



void rotate_gyro(float motor_pow, float ang)
{
	if(ang > SensorValue[S2])
	{
		drive(motor_pow, -motor_pow);
		while(SensorValue[S2] < ang)
		{}
	}
	else
	{
		drive(-motor_pow, motor_pow);
		while(SensorValue[S2] > ang)
		{}
	}

		drive(0,0);
}


void set_pencil(bool action)
{
	if (action)
	{
		int target = nMotorEncoder[motorC] + PENCIL_INCREMENT;
		motor[motorC] = PENCIL_POWER;
		while (nMotorEncoder[motorC] < target)
		{}
		motor[motorC] = 0;
	}
	else
	{
		int target = nMotorEncoder[motorC] - PENCIL_INCREMENT;
		motor[motorC] = -PENCIL_POWER;
		while (nMotorEncoder[motorC] > target)
		{}
		motor[motorC] = 0;
	}
}

void wait_read(void) {
	while (getButtonPress(buttonEnter)) 
  {}
  while (!getButtonPress(buttonEnter)) 
  {
    displayBigTextLine(3, "Press enter ");
    displayBigTextLine(5, "when you're ");
    displayBigTextLine(7, "ready.");
  }
  while (getButtonPress(buttonEnter)) 
  {}
}


// ---------------------------- IR FUNCTIONS ----------------------------------------
// ----------------------------------------------------------------------------------

void Remote_Area_Perimeter(float & area, float & perimeter, past & prev, present & curr)
{
  time1[T1] = 0;
  time1[T2] = 0;
  time1[T3] = 0;
  nMotorEncoder[motorA] = 0;
  resetGyro(S2);
  int index = 0;
  int index1 = 0;
  configure_sensors();
	bool roomFinished = false;

  coordinate C1, C2;

  reset_variables(prev, curr, area, perimeter, C1, C2);

  float origin_tol = 0;

  while(!roomFinished && !getButtonPress(buttonEnter))
  {
  	if (time1[T1] > index)
  	{
  		// commented out because unecessary
  		// can be implemented in future
	    //correct_IR(prev, curr);
	    index = time1[T1] + IR_TIME_TOL;
		}

    while(getIRRemoteChannelButtons(S4, 1))
    {
    	remote_drive();

      curr.final_ang = getGyroDegrees(S2) * DEG_TO_RAD;


      if (time1[T2] > index1) 
      {
        if (fabs(area) > ORIGIN_SHIFT)
          origin_tol = ORIGIN_FACTOR * sqrt(fabs(area) - ORIGIN_SHIFT);
         roomFinished = coordinate_process(IR_TURN_MODE, curr, area, perimeter, origin_tol, C1, C2);
         index1 = time1[T2] + TIME_GAP;
      }
    }
    drive(0, 0);
  }
  area = (pow(fabs(area), IR_AREA_OFFSET_FACTOR)) * CM2_TO_M2;
  perimeter = (pow(fabs(perimeter + origin_tol), IR_PERIM_OFFSET_FACTOR)) * CM_TO_M;
  drive(0, 0);
  mission_possible();
}



void remote_drive()
{
	int left_power = 0, right_power = 0;

    if (getIRRemoteChannelButtons(S4, 1) == 5)
    {
    	left_power = IR_DRIVE_MOTOR_POWER;
    	right_power = IR_DRIVE_MOTOR_POWER;
    }
    else if (getIRRemoteChannelButtons(S4, 1) == 8)
    {
    	//left_power = -IR_DRIVE_MOTOR_POWER;
    	//right_power = -IR_DRIVE_MOTOR_POWER;
    }
    else if (getIRRemoteChannelButtons(S4, 1) == 6)
    {
    	//left_power = IR_DRIVE_MOTOR_POWER;
    	//right_power = -IR_DRIVE_MOTOR_POWER;
    }
    else if (getIRRemoteChannelButtons(S4, 1) == 7)
    {
    	left_power = -IR_DRIVE_MOTOR_POWER;
    	right_power = IR_DRIVE_MOTOR_POWER;
    }
    else if (getIRRemoteChannelButtons(S4, 1) == 1)
    {
    	left_power = IR_DRIVE_MOTOR_POWER;
    	right_power = 0;
    }
    else if (getIRRemoteChannelButtons(S4, 1) == 3)
    {
    	left_power = 0;
    	right_power = IR_DRIVE_MOTOR_POWER;
    }
    else if (getIRRemoteChannelButtons(S4, 1) == 2)
    {
    	left_power = -IR_DRIVE_MOTOR_POWER;
    	right_power = 0;
    }
    else if (getIRRemoteChannelButtons(S4, 1) == 4)
    {
    	mission_possible();
    	//left_power = 0;
    	//right_power = -IR_DRIVE_MOTOR_POWER;
    }

    if (left_power != motor[motorD] || right_power != motor[motorA])
    	drive(left_power, right_power);
}


void correct_IR(past & prev, present & curr)
{

  // PID control function adjusting to motor power ratings
  float control_signal = 0;
  float magnitude_of_power = 0;
  float error = 0;

  // Calculate the change in time (in milliseconds)
  float curr_time = time1[T1];
  float delta_time = curr_time - prev.time;
  prev.time = curr_time;

  // Calculate info on the error/time graph
  error = getIRBeaconChannelDirection(S4, 2);
  curr.proportional = fabs(error);
  curr.integral = (curr.integral + curr.proportional) / delta_time;
  curr.derivative = (curr.proportional - prev.proportional) / delta_time;
  prev.proportional = curr.proportional;

  control_signal = IR_CONST_PROPORTIONAL * curr.proportional + IR_CONST_INTEGERAL * curr.integral + IR_CONST_DERIVATIVE * curr.derivative;
  magnitude_of_power = fabs(control_signal);

  if (nMotorEncoder[motorB] >= IR_RESET_ANGLE || nMotorEncoder[motorB] <= -IR_RESET_ANGLE)
  	reset_IR();

  if (magnitude_of_power > IR_POWER_MAX)
  {
  	magnitude_of_power = IR_POWER_MAX;
  }

  if (error > 0 && error > IR_ERROR_TOL)
  {
  	curr.IR = magnitude_of_power;
  }
  else if (error < 0 && error < -IR_ERROR_TOL)
  {
    curr.IR = -magnitude_of_power;
  }

  if (prev.IR != curr.IR) 
  {
    motor[motorB] = curr.IR;
  }
  prev.IR = curr.IR;
}

void reset_IR()
{
	if (nMotorEncoder[motorB] >= IR_RESET_ANGLE)
		motor[motorB] = -IR_POWER_MAX;
	else if (nMotorEncoder[motorB] <= -IR_RESET_ANGLE)
		motor[motorB] = IR_POWER_MAX;

	while (abs(nMotorEncoder[motorB]) > IR_RESET_TOL)
	{}
	motor[motorB] = 0;
}


void mission_possible()
{

  //        100 = Tempo
  //          6 = Default octave
  //    Quarter = Default note length
  //        10% = Break between notes
  //
  playTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
  playTone(  933,    7); wait1Msec(  75);  // Note(D#, Duration(32th))
  playTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
  playTone(  933,    7); wait1Msec(  75);  // Note(D#, Duration(32th))
  playTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
  playTone(  933,    7); wait1Msec(  75);  // Note(D#, Duration(32th))
  playTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
  playTone(  933,    7); wait1Msec(  75);  // Note(D#, Duration(32th))
  playTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
  playTone(  880,    7); wait1Msec(  75);  // Note(D, Duration(32th))
  playTone(  933,    7); wait1Msec(  75);  // Note(D#, Duration(32th))
  playTone(  988,    7); wait1Msec(  75);  // Note(E, Duration(32th))
  playTone( 1047,    7); wait1Msec(  75);  // Note(F, Duration(32th))
  playTone( 1109,    7); wait1Msec(  75);  // Note(F#, Duration(32th))
  playTone( 1175,    7); wait1Msec(  75);  // Note(G, Duration(32th))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
  playTone( 1398,   14); wait1Msec( 150);  // Note(A#, Duration(16th))
  playTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
  playTone(  784,   14); wait1Msec( 150);  // Note(C, Duration(16th))
  playTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
  playTone( 1047,   14); wait1Msec( 150);  // Note(F, Duration(16th))
  playTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
  playTone( 1109,   14); wait1Msec( 150);  // Note(F#, Duration(16th))
  playTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
  playTone( 1398,   14); wait1Msec( 150);  // Note(A#, Duration(16th))
  playTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
  playTone(  784,   14); wait1Msec( 150);  // Note(C, Duration(16th))
  playTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(    0,   27); wait1Msec( 300);  // Note(Rest, Duration(Eighth))
  playTone( 1047,   14); wait1Msec( 150);  // Note(F, Duration(16th))
  playTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
  playTone( 1109,   14); wait1Msec( 150);  // Note(F#, Duration(16th))
  playTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
  playTone( 1398,   14); wait1Msec( 150);  // Note(A#, Duration(16th))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(  880,  108); wait1Msec(1200);  // Note(D, Duration(Half))
  playTone(    0,    7); wait1Msec(  75);  // Note(Rest, Duration(32th))
  playTone( 1398,   14); wait1Msec( 150);  // Note(A#, Duration(16th))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(  831,  108); wait1Msec(1200);  // Note(C#, Duration(Half))
  playTone(    0,    7); wait1Msec(  75);  // Note(Rest, Duration(32th))
  playTone( 1398,   14); wait1Msec( 150);  // Note(A#, Duration(16th))
  playTone( 1175,   14); wait1Msec( 150);  // Note(G, Duration(16th))
  playTone(  784,  108); wait1Msec(1200);  // Note(C, Duration(Half))
  playTone(    0,   14); wait1Msec( 150);  // Note(Rest, Duration(16th))
  playTone(  932,   14); wait1Msec( 150);  // Note(A#5, Duration(16th))
  playTone(  784,   14); wait1Msec( 150);  // Note(C, Duration(16th))
  return;
}
