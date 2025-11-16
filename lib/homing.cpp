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
    output.begin(&current_position);

    limit_switch_button.begin(limit_switch_port_number, INPUT_PULLDOWN);
    limit_switch_button.set_mode(BUTTON_MODE_TOGGLE);

    // Serial.print("Address of output blockport (begin) ");
    // Serial.println(((unsigned int)&this->output));  

    // Serial.print("begin: ");
    // Serial.print(*this->output.target);
    // Serial.print("\n");

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
            // Serial.print("limit button value: ");
            // Serial.print(limit_switch_button.read());
            // Serial.print("\n");
            // are we already at the limit switch?
            if (limit_switch_button.read()) {
                // we are already at the switch => back off
                current_homing_state = HOMING_AXIS_STATE_BACKING_OFF;
                Serial.println("AXIS BACKING OFF (already on switch)");

                // Going backwards compared to the normal homing direction
            }
            else {
                // we are not at the switch => move forward
                Serial.println("AXIS MOVING FWD");
                current_homing_state = HOMING_AXIS_STATE_SEEKING;
            }
            break;

        case HOMING_AXIS_STATE_BACKING_OFF:
            // are we still pressing the limit switch?
            if (limit_switch_button.read()) {
                // we are still pressing switch => continue backing off
                // Going backwards compared to the normal homing direction
                move_backward();
            }
            else {
                // we are not at the switch anymore => we can start seeking
                current_homing_state = HOMING_AXIS_STATE_SEEKING;
                Serial.println("AXIS GOING BACK FWD (left switch)");
            }
            break;
    
        case HOMING_AXIS_STATE_SEEKING:
            // are we at the limit switch?
            if (limit_switch_button.read()) {
                // we reached the limit
                current_homing_state = HOMING_STATE_FINISHED;
                Serial.println("AXIS FINISHED HOMING (HIT LIMIT)");
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
    // TODO: call push_deep or something on the axis output blockport
    // in order to set state throughout the chain to be consistent with value_at_limit

    // output.push_deep(value_at_limit);
}

void HomingAxis::move_forward()
{
    // output.set(1, INCREMENTAL);
    output.set(1.0 * homing_direction * homing_velocity * CORE_FRAME_PERIOD_S, INCREMENTAL);
    output.push();

    // Serial.print("forward move value:");
    // Serial.print(-1.0 * homing_direction * homing_velocity * CORE_FRAME_PERIOD_S);
    // Serial.print(output.read(INCREMENTAL));
    // Serial.print("\n");
}

void HomingAxis::move_backward()
{
    
    // output.set(1, INCREMENTAL);
    output.set(-1.0 * homing_direction * homing_velocity * CORE_FRAME_PERIOD_S, INCREMENTAL);
    output.push();

    // Serial.print("backward move value:");
    // Serial.print(-1.0 * homing_direction * homing_velocity * CORE_FRAME_PERIOD_S);
    // Serial.print(output.read(INCREMENTAL));
    // Serial.print("\n");
}

Homing::Homing()
{}

void Homing::begin()
{
    // register_plugin();
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

    // axes[nb_axes].begin();
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
        // Serial.print("in homing loop, axis id = ");
        // Serial.print(axis_currently_homing);
        // Serial.print("\n");
        if (axis_currently_homing > nb_axes - 1) {
            // we finished all axes => 
            // call push_deep or something on the axis output blockport
            // in order to set state throughout the chain to be consistent with value_at_limit

            for (int i = 0; i < nb_axes; i++) {
                axes[i].set_axis_value();
            }

            // Mark the homing as finished
            axis_currently_homing = -1;
        }
        else {
            // still dealing with current axis maybe?
            // - check its state
            if (axes[axis_currently_homing].read_state() == HOMING_STATE_FINISHED) {
                axis_currently_homing++; // move on to the next axis
                Serial.print("Homing next axis: ");
                Serial.print(axis_currently_homing);
                Serial.print("\n");
            }
            else {
                if (axes[axis_currently_homing].read_state() == HOMING_STATE_WAITING) {
                    axes[axis_currently_homing].start_homing_routine();
                }

                // axes[axis_currently_homing].run();
                // Serial.print("Address of axis we run on is ");
                // Serial.println(((unsigned int)&axes[axis_currently_homing]));  

            }
        }
    }
}
