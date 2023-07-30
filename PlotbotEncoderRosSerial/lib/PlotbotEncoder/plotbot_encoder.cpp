/// Source file for PlotbotEncoder

#include "plotbot_encoder.h"
#include "plotbot_parameter.h"

int PlotbotEncoder::setup()
{
  rx.begin(ROTARY_PIN_X1, ROTARY_PIN_X2, rotary_steps_per_click);
  ry.begin(ROTARY_PIN_Y1, ROTARY_PIN_Y2, rotary_steps_per_click);
  rz.begin(ROTARY_PIN_Z1, ROTARY_PIN_Z2, rotary_steps_per_click);

  if((rx.getID() == ry.getID()) || (rx.getID() == rz.getID()) || (ry.getID() == rz.getID()))
  {
    return -1;
  }
  return 1;
}


void PlotbotEncoder::update()
{
  rx.loop();
  ry.loop();
  rz.loop();
}

void PlotbotEncoder::getPositions(float *positions) const
{
  positions[0] = static_cast<float>(rx.getPosition()) * meterPerClick;
  positions[1] = static_cast<float>(ry.getPosition()) * meterPerClick;
  positions[2] = static_cast<float>(rz.getPosition()) * meterPerClick;
}

void PlotbotEncoder::resetPositions(float *positions /* = NULL */)
{
  if (positions)
  {
    rx.resetPosition(positions[0] / meterPerClick);
    ry.resetPosition(positions[1] / meterPerClick);
    rz.resetPosition(positions[2] / meterPerClick);
  }
  else
  {
    rx.resetPosition();
    ry.resetPosition();
    rz.resetPosition();
  }
}