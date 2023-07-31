/// Source file for PlotbotEncoderRos2Serializer class

#include "ucdr/microcdr.h"

#include "plotbot_encoder_ros2_serializer.h"

bool const PlotbotEncoderRos2Serializer::sendPositions()
{
  size_t const headerLen = sizeof(ros2::COBSHeader);
  size_t sendBufferLen = headerLen + payloadLen;
  uint8_t sendBuffer[sendBufferLen];

  bool const payloadSerialized = writePayload(sendBuffer + headerLen);
  if(payloadSerialized)
  {
    fillHeader((ros2::COBSHeader *)sendBuffer, sendBuffer + headerLen);
    packetSerial.send(sendBuffer, sendBufferLen);
    return true;
  }
  return false;
}

void PlotbotEncoderRos2Serializer::fillHeader(ros2::COBSHeader *header, uint8_t const *const payload)
{
  uint16_t crc = ros2::crc16(payload, payloadLen);

  header->topic_ID = 0x3;
  header->payload_len_h = 0x0;
  header->payload_len_l = 0xC; // 12 (payload_len)
  header->crc_h = *((uint8_t *)&(crc) + 1);
  header->crc_l = *((uint8_t *)&crc);
}

bool PlotbotEncoderRos2Serializer::writePayload(uint8_t *buffer)
{
  float positions[PlotbotEncoder::numAxes];
  plotbotEncoder->getPositions(positions);

  ucdrBuffer writer;
  ucdr_init_buffer(&writer, buffer, payloadLen);
  return ucdr_serialize_array_float(&writer, positions, PlotbotEncoder::numAxes);
}
 