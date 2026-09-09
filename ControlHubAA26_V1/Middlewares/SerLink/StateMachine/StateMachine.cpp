/*
 * StateMachine.cpp
 *
 *  Created on: 22 Apr 2023
 *      Author: rw123
 */

#include "StateMachine.hpp"

uint8_t StateMachine::getCurrentState()
{
    return this->currentState;
}

char* StateMachine::getCurrentStateName()
{
    return "N/A";
}

