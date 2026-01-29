#include "homing.hpp"

HomingAxis::HomingAxis(){}

HomingAxis::HomingAxis(
    uint8_t limit_switch_port, 
    DecimalPosition value_at_limit, 
    int direction, 
    DecimalPosition velocity)
{
    this->limit_switch_port_number = limit_switch_port;
    this->value_at_limit = value_at_limit;
    this->homing_direction = direction;
    this->homing_velocity = velocity;

    current_homing_state = HOMING_STATE_WAITING;
}

void HomingAxis::begin()
{
    // initialize internal blockport
    output.begin(&current_position, BLOCKPORT_OUTPUT);

    limit_switch_button.begin(limit_switch_port_number, INPUT_PULLDOWN);
    limit_switch_button.set_mode(BUTTON_MODE_STANDARD);

    register_plugin();
}

uint8_t HomingAxis::read_state()
{
    return this->current_homing_state;
}

void HomingAxis::start_homing_routine()
{
    Serial.println("start homing axis");
    current_homing_state = HOMING_AXIS_STATE_BEGIN;
}

void HomingAxis::wait_for_homing()
{
    current_homing_state = HOMING_STATE_WAITING;
}

void HomingAxis::run()
{
    // Serial.print("homing axis state: ");
    // Serial.print(current_homing_state);
    // Serial.print("\n");

    // Check and update state if needed
    switch(current_homing_state) {
        case HOMING_STATE_WAITING:
            break;

        case HOMING_STATE_FINISHED:
            break;
        
        case HOMING_AXIS_STATE_BEGIN:
            // are we already at the limit switch?
            if (limit_switch_button.read_raw()) {
                // we are already at the switch => back off
                current_homing_state = HOMING_AXIS_STATE_BACKING_OFF;
                Serial.println("MOVING AWAY FROM SWITCH (already on switch)");

                // Going backwards compared to the normal homing direction
            }
            else {
                // we are not at the switch => move forward
                Serial.println("MOVING TOWARDS SWITCH");
                current_homing_state = HOMING_AXIS_STATE_SEEKING;
            }
            break;

        case HOMING_AXIS_STATE_BACKING_OFF:
            // are we still pressing the limit switch?
            if (limit_switch_button.read_raw()) {
                // we are still pressing switch => continue backing off
                // Going backwards compared to the normal homing direction
                move_backward();
            }
            else {
                // we are not at the switch anymore => we can start seeking
                current_homing_state = HOMING_AXIS_STATE_SEEKING;
                Serial.println("MOVING TOWARDS SWITCH");
            }
            break;
    
        case HOMING_AXIS_STATE_SEEKING:
            // are we at the limit switch?
            if (limit_switch_button.read_raw()) {
                // we reached the limit
                current_homing_state = HOMING_STATE_FINISHED;
                Serial.println("HIT SWITCH (FINISHED HOMING)");
            }
            else {
                // we did not hit the switch yet => keep moving forward
                move_forward();
            }
            break;
    }


}


void HomingAxis::set_axis_value()
{
    // Set state throughout the chain to be consistent with value_at_limit
    // Serial.print("Reset deep: ");
    // Serial.println(value_at_limit);
    output.reset_deep(value_at_limit);
}

void HomingAxis::move_forward()
{
    output.set(1.0 * homing_direction * homing_velocity * CORE_FRAME_PERIOD_S, INCREMENTAL);
    output.push();
}

void HomingAxis::move_backward()
{
    output.set(-1.0 * homing_direction * homing_velocity * CORE_FRAME_PERIOD_S, INCREMENTAL);
    output.push();
}

Homing::Homing()
{}

void Homing::begin()
{
    register_plugin(PLUGIN_LOOP);

    for (int i = 0; i < nb_axes ; i++) {
        axes[i].begin();
    }

}

void Homing::add_axis(
    uint8_t limit_switch_port, 
    DecimalPosition value_at_limit, 
    int direction, 
    DecimalPosition velocity,
    BlockPort *target)
{
    if (nb_axes >= MAX_NUM_AXES) {
        Serial.println("WARNING: failed to add axis, number of axes > max number of axes to home.");
        return;
    }

    axes[nb_axes] = HomingAxis(
        limit_switch_port,
        value_at_limit,
        direction,
        velocity
    );


    Serial.print("add axis:");
    Serial.print(((unsigned int)&axes[nb_axes]));  
    Serial.print("\n");

    // map output to that axis
    axes[nb_axes].output.map(target);
    // TODO: probably should try to make sure no other source of motion is mapped to the axis during homing

    nb_axes++;
}

void Homing::start_homing_routine()
{
    // Mark all axes as "waiting"
    for (int i = 0; i < nb_axes; i++) {
        axes[i].wait_for_homing();
    }

    // Start with the first axis
    axis_currently_homing = 0;
}

void Homing::loop()
{
    // Deal with currently active axis
    if (axis_currently_homing < 0) {
        // homing has not begun, or has ended
        return;
    }
    else {
        if (axis_currently_homing > nb_axes - 1) {
            // we finished all axes => 
            // set state to be consistent with value_at_limit

            for (int i = 0; i < nb_axes; i++) {
                axes[i].set_axis_value();
            }

            // Mark the homing as finished
            axis_currently_homing = -1;
        }
        else {
            // we are still dealing with current axis maybe?
            // - check its state
            if (axes[axis_currently_homing].read_state() == HOMING_STATE_FINISHED) {
                axis_currently_homing++; // move on to the next axis
                // Serial.print("Homing next axis: ");
                // Serial.print(axis_currently_homing);
                // Serial.print("\n");
            }
            else {
                if (axes[axis_currently_homing].read_state() == HOMING_STATE_WAITING) {
                    axes[axis_currently_homing].start_homing_routine();
                }

            }
        }
    }
}
