#include "geometry.hpp"

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