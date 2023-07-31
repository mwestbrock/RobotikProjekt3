/// Definition of encoder pins

#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H
  
//static float const meterPerRev = 6.35e-3F;
static float const meterPerClick = 6.35e-3F/20.F;

/// On/Off switches per step that would invoke the position changed callback function?
static byte const rotary_steps_per_click = 4;

#define ROTARY_PIN_X1 16
#define ROTARY_PIN_X2 5
#define ROTARY_PIN_Y1 0
#define ROTARY_PIN_Y2 2
#define ROTARY_PIN_Z1 14
#define ROTARY_PIN_Z2 12

#endif