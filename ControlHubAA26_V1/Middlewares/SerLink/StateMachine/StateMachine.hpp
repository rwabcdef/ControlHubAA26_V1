/*
 * StateMachine.hpp
 *
 *  Created on: 22 Apr 2023
 *      Author: rw123
 */

#ifndef STATEMACHINE_HPP_
#define STATEMACHINE_HPP_

#include<stdint.h>

class StateMachine
{
    protected:
      uint8_t currentState = 0;

    public:
        virtual void run() = 0;
        uint8_t getCurrentState();
        char* getCurrentStateName();
};

#endif /* STATEMACHINE_HPP_ */
