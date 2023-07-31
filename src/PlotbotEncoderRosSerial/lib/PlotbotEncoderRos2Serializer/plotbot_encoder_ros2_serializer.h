/// Header file for PlotbotEncoderRos2Serializer class
// todo: make generic
#ifndef PLOTBOT_ENCODER_ROS2_SERIALIZER_H
#define PLOTBOT_ENCODER_ROS2_SERIALIZER_H

#include <Arduino.h>
#include <PacketSerial.h>
#include <Ticker.h>

#include "plotbot_encoder.h"
#include "ros_defines.h"

/// @brief Plotbot with simple encoder on each axis that sends
/// the position data as ROS2 message via serial so that it can
/// be relayed via ros2_serial_example
class PlotbotEncoderRos2Serializer
{
public:
  /// @brief Setup the PlotbotEncoderRos2Serializer. Must be called before
  /// it can be used.
  /// @param[in] pb Plotbot whose positions shall be serialized and sent.
  /// pb Must be set up by the caller already.
  void setup(PlotbotEncoder &pb)
  {
    plotbotEncoder = &pb;
    packetSerial.begin(BAUDRATE);
    sendPositionsTicker.attach_ms(
        100, +[](PlotbotEncoderRos2Serializer *node)
             { node->sendPositions(); },
        this); // send position with 10Hz todo: evaluate this
  }

  /// @brief Send the positions of the axes as cobs encoded ROS message for
  /// further relay via ros2_serial_example node
  /// @return True if positions could be serialiyed and sent, false otherwise.
  bool const sendPositions();

private:
  /// @brief Fill the header of the to be transmitted message.
  /// @param header Header with topic ID, payload length and crc checksum.
  /// @param payload Message payload (cdr encoded payload (axes positions))
  void fillHeader(ros2::COBSHeader *header, uint8_t const *const payload);

  /// @brief Get the axes positions, cdr decode/serialize them and write the
  /// result to the specified buffer.
  /// @param buffer Array of 12 bytes into which the serialized float array
  /// with axes positions should be written to.
  bool writePayload(uint8_t *buffer);

private:
  /// Plotbot encoder to get positions
  PlotbotEncoder const * plotbotEncoder;

  /// Serial communication with COBS encoded messages
  PacketSerial packetSerial;

  /// Timer to send plotbot axis positions as ROS topic via serial
  Ticker sendPositionsTicker;

  /// Length of the payload of the message (4bytes for each axis)
  static size_t const payloadLen = PlotbotEncoder::numAxes * 4;
};

#endif
