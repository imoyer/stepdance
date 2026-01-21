#include "math_utils.hpp"

Vector2DToAngle::Vector2DToAngle()
{
}

void Vector2DToAngle::begin()
{
    input_x.begin(&input_x_position);
    input_y.begin(&input_y_position);
    output_theta.begin(&output_theta_position);

    register_plugin();
}


void Vector2DToAngle::run()
{
    // convert the 2D XY vector to its angle wrt the X axis (ccw)
    input_x.pull();
    input_y.pull();
    input_x.update();
    input_y.update();

    DecimalPosition theta = atan2(input_y.read(INCREMENTAL), input_x.read(INCREMENTAL));

    output_theta.set(theta, ABSOLUTE);
    output_theta.push();
}

void Vector2DToAngle::debugPrint() {
  Serial.print("input vec: ");
  Serial.print(input_x.read(INCREMENTAL));
  Serial.print(",");
  Serial.print(input_y.read(INCREMENTAL));
  Serial.print(" output_theta: ");
  Serial.println(output_theta.read(ABSOLUTE));
}

MoveDurationToFrequency::MoveDurationToFrequency()
{
}

void MoveDurationToFrequency::begin()
{
  input_move_duration.begin(&input_move_duration_value);
  output_frequency.begin(&output_frequency_value);

  register_plugin();
}

void MoveDurationToFrequency::debugPrint()
{
  Serial.print("input move duration: ");
  Serial.println(input_move_duration.read(ABSOLUTE));
}

void MoveDurationToFrequency::run()
{
  input_move_duration.pull();

  DecimalPosition freq = target_frequency;
  if (input_move_duration.absolute_buffer > 0)
  {
    // freq = std::round(input_move_duration.absolute_buffer * target_frequency) / input_move_duration.absolute_buffer;
    freq = std::round(input_move_duration.absolute_buffer * target_frequency);
  }

  output_frequency.set(freq, ABSOLUTE);
  output_frequency.push();

}
