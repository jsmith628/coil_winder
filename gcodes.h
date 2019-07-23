#ifndef _GCODE_H_
  #define _GCODE_H_

//G codes define movement and interpretation commands

//Rapid move (A axis position, B axis position, Speed, Feedrate)
void g0 (float a, float b, float s, float f);

//Linear interpolate (A axis position, B axis position, Speed | Feedrate)
void g1 (float a, float b, float s, float f);

//Dwell (P (millis) | S (seconds) )
void g4 (float p, float s);

//Programming in inches
void g20 ();

//Programming in millimeters
void g21 ();

//Home axis (A final position, B final position, Speed)
void g28 (float a, float b, float s);

//Feed until skip (A axis enable, B axis enable, Speed)
void g31 (bool a, bool b, float s);

//Single point threading, non-cycle (for cycle, use G76) (A axis position, B axis position, Spindle speed, Feedrate)
void g32 (float a, float b, float s, float f);

//Define maximum spindle Speed (Speed)
void g50 (float s);

//Local coordinates, defines program zero to a new location (A position 0, B position 0)
void g52 (float a, float b);

//Repetitive threading cycle (A axis position, B axis position, Spindle speed, Feedrate, Repititions, Symmetrical)
void g76 (float a, float b, float s, float f, int r, bool d);

//Absolute positioning (position defined from machine zero)
void g90();

//Incremental positioning (position defined relative to previous position)
void g91();

//Feedrate per minute
void g94();

//Feedrate per revolution
void g95();

//M codes define miscellaneous commands

//Unconditional stop
void m0();

//Spindle on, CW (Speed)
void m3(float s);

//Spindle on, CCW (Speed)
void m4(float s);

//Spindle stop
void m5();

//Enable steppers
void m17();

//Disable steppers
void m18();

//End of program, return to program top
void m30();

//Subprogram call
void m98();

//Subprogram end
void m99();

//Emergency stop, immediately stop program and disable all steppers
void m112();

//Current position
float m114();
void m114(bool a, bool b);

//Park (A axis position, B axis postion)
void m125(float a, float b);

//Enable software endstops
void m120();

//Disable software endstops
void m121();

//Set max feedrate (Feedrate)
void m203(float f);

//Save settings to EEPROM
void m500();

//Load settings from EEPROM
void m501();

//Read out settings from EEPROM
void m503();

#endif
