#ifndef _GCODE_H_
  #define _GCODE_H_

//G codes define movement and interpretation commands

//Rapid move (A axis position, B axis position, Spindle rotations, Speed, Feedrate)
void g0 (float a, float b, float w, float s, float f);

//Linear interpolate (A axis position, B axis position, Spindle rotations, Speed, Feedrate)
void g1 (float a, float b, float w, float s, float f);

//Dwell (P (millis) | S (seconds) )
void g4 (float p, float s);

//Programming in inches
void g20 ();

//Programming in millimeters
void g21 ();

//Home axis (A axis enable, B axis enable)
void g28 (bool a, bool b);

//Feed until skip (A axis enable, B axis enable)
void g31 (int8_t a, int8_t b);

//Define maximum spindle Speed (Speed)
void g50 (float s);

//Local coordinates, defines program zero to a new location (A position 0, B position 0, Spindle position 0)
void g52 (float a, float b, float w);

//Absolute positioning (position defined from machine zero)
void g90();

//Incremental positioning (position defined relative to previous position)
void g91();

//Set current position to specified value
void g92(float a, float b, float w);

//Feedrate per minute
void g94();

//Feedrate per revolution
void g95();

//M codes define miscellaneous commands

//Unconditional stop
void m0();

//Enable steppers
void m17();

//Disable steppers
void m18();

//End of program, return to program top
void m30();

//Spindle absolute positioning
void m82();

//Spindle incremental positioning
void m83();

//Set axis steps per unit
void m92(float a, float b, float w);

//Subprogram call
void m98();

//Subprogram end
void m99();

//Emergency stop, immediately stop program and disable all steppers
void m112();

//Current position
float m114();
void m114(bool a, bool b);

//Enable software endstops
void m120();

//Disable software endstops
void m121();

//Park (A axis position, B axis postion)
void m125(float a, float b, float w);

//Set max acceleration
void m201(float s);

//Set max feedrate (Feedrate)
void m203(float f);

//Set starting acceleration
void m204(float a, float b, float w);

//Save settings to EEPROM
void m500();

//Load settings from EEPROM
void m501();

//Read out settings from EEPROM
void m503();

#endif
