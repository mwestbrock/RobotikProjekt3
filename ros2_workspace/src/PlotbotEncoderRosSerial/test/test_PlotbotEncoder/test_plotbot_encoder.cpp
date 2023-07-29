#include "Arduino.h"
#include "unity.h"
#include "plotbot_encoder.h"
#include "plotbot_parameter.h"

void setUp() {/*not needed*/}
void tearDown() {/*not needed*/}

void simulateRotation(uint8_t pin1, uint8_t pin2)
{
  if (digitalRead(pin1))
  {
    if (digitalRead(pin2))
    {
      digitalWrite(pin1, LOW);
    }
    else
    {
      digitalWrite(pin2, HIGH);
    }
  }
  else
  {
    if (digitalRead(pin2))
    {
      digitalWrite(pin2, LOW);
    }
    else
    {
      digitalWrite(pin1, HIGH);
    }
  }
}

void test_PlotbotEncoderSetup_CallSetup_ReturnOne(void)
{
  PlotbotEncoder pb;
  int const setupRes = pb.setup();
  TEST_ASSERT_EQUAL(1, setupRes);
}

void test_PlotbotEncoderGetPosition_NoTurn_ReturnZeros(void)
{
  PlotbotEncoder pb;
  TEST_ASSERT(pb.setup());
  float pbPos[3];
  pb.getPositions(pbPos);
  for (float pos : pbPos)
  {
    TEST_ASSERT_EQUAL_FLOAT(pos, 0.F);
  }
}

void test_Rotary_Turn_ReturnAnyPos(void)
{
  ESPRotary r;
  r.begin(ROTARY_PIN_Z1, ROTARY_PIN_Z2);
  for (int i = 0; i<1000; ++i)
  {
    r.loop();
    delay(10);
  }
  TEST_ASSERT(r.getPosition());
}

void test_PlotbotEncoderGetPosition_TurnZ_PosZChanged(void)
{
  PlotbotEncoder pb;
  TEST_ASSERT(pb.setup());
  float pbPos[3];
  pb.getPositions(pbPos);
  for (int i = 0; i<500; ++i)
  {
    pb.update();
    delay(10);
  }
  TEST_ASSERT(abs(pbPos[2]) > 0.F);
}

void test_PlotbotEncoderResetPosition_Set0_ReturnPosZero(void)
{
  PlotbotEncoder pb;
  TEST_ASSERT(pb.setup());
  pb.resetPositions();
  float pbPos[3];
  pb.getPositions(pbPos);
  for (float pos : pbPos)
  {
    TEST_ASSERT_EQUAL_FLOAT(pos, 0.F);
  }
}

void test_PlotbotEncoderResetPosition_SetVal_ReturnVal(void)
{
  PlotbotEncoder pb;
  TEST_ASSERT(pb.setup());
  float setPos[3] = {1.0F, 2.1F, 3.21F};
  pb.resetPositions(setPos);

  float pbPos[3];
  pb.getPositions(pbPos);
  for (int i = 0; i < 3; ++i)
  {
    TEST_ASSERT_FLOAT_WITHIN(1e-3,setPos[i], pbPos[i]);
  }
}

// using arduino framework, we need setup and loop functions
void setup()
{
  delay(2000);

  UNITY_BEGIN();
  RUN_TEST(test_PlotbotEncoderSetup_CallSetup_ReturnOne);
  RUN_TEST(test_PlotbotEncoderGetPosition_NoTurn_ReturnZeros);
  RUN_TEST(test_PlotbotEncoderGetPosition_TurnZ_PosZChanged);
  RUN_TEST(test_PlotbotEncoderResetPosition_Set0_ReturnPosZero);
  RUN_TEST(test_PlotbotEncoderResetPosition_SetVal_ReturnVal);
  UNITY_END();
}

void loop() {/* do nothing */}


// int main(int argc, char **argv)
// {
//   UNITY_BEGIN();
//   RUN_TEST(test_PlotbotEncoderSetup_CallSetup_ReturnOne);
//   RUN_TEST(test_PlotbotEncoderGetPosition_NoTurn_ReturnZeros);
//   RUN_TEST(test_PlotbotEncoderGetPosition_TurnZ_ReturnPos);
//   UNITY_END();
//   return 0;
// }