// Header file for PlotbotEncoder

#ifndef PLOTBOT_ENCODER_H
#define PLOTBOT_ENCODER_H

#include <ESPRotary.h>

/// @brief Plotbot with simple encoder on each axis
class PlotbotEncoder
{
public:
  /// Number of axes the plotbot has
  static size_t const numAxes = 3;

  /// @brief Enumeration for index - axis association
  enum axisIndex
  {
    x = 0,
    y = 1,
    z = 3
  };

public:
  /// @brief Setup the encoder on each axis.
  /// @return 1 if setup was successful, -1 otherwise.
  int setup();

  /// @brief Update the encoder on each axis. This function must be called
  /// regularly (to not miss a step).
  void update();

  /// @brief Get the position of each axis in meter
  /// @param[out] positions Position of x, y, and z axis in m. Must be a float array of size at least 3.
  void getPositions(float *positions) const;

  /// @brief Set the position of each axis in meter
  /// @param[out] positions Position of x, y, and z axis in m. Must be a float array of size at least 3.
  void resetPositions(float *positions = NULL);

private:
  /// Rotary encoder x-Axis
  ESPRotary rx;
  /// Rotary encoder y-Axis
  ESPRotary ry;
  /// Rotary encoder z-Axis
  ESPRotary rz;
};

#endif
