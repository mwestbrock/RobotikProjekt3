#include "Arduino.h"
#include "unity.h"

#include "ucdr/microcdr.h"
//#include "ros_defines.h"

void setUp() {/*not needed*/}
void tearDown() {/*not needed*/}

namespace ros2
{
typedef uint8_t topic_id_size_t;
struct __attribute__((packed)) COBSHeader
{
  topic_id_size_t topic_ID;
  uint8_t payload_len_h;
  uint8_t payload_len_l;
  uint8_t crc_h;
  uint8_t crc_l;
};
}

void test_floatArraySerialization_nonzeroNums_correctSerialization(void)
{
  size_t bufferLen = 12;
  uint8_t buffer[bufferLen];

  float pos[3] = {1.0F, 2.1F, 3.21F};
  ucdrBuffer writer;
  ucdr_init_buffer(&writer, buffer, bufferLen);
  bool couldSerialize = ucdr_serialize_array_float(&writer, pos, 3);
  TEST_ASSERT_TRUE(couldSerialize);

  float decodedPos[3];
  ucdrBuffer reader;
  ucdr_init_buffer(&reader, buffer, bufferLen);
  bool couldDeserialize = ucdr_deserialize_array_float(&reader, decodedPos, 3);
  TEST_ASSERT_TRUE(couldDeserialize);

  TEST_ASSERT_EQUAL_FLOAT_ARRAY(pos, decodedPos, 3);
}

// using arduino framework, we need setup and loop functions
void setup()
{
  delay(2000);

  UNITY_BEGIN();
  RUN_TEST(test_floatArraySerialization_nonzeroNums_correctSerialization);
  UNITY_END();
}

void loop() {/* do nothing */}
