#include <Arduino.h>

#include "plotbot_encoder_ros2_serializer.h"

PlotbotEncoder plotbot;
PlotbotEncoderRos2Serializer plotbotSerializer;

void setup()
{
  plotbot.setup();
  plotbotSerializer.setup(plotbot);
}

void loop()
{
  plotbot.update();
}