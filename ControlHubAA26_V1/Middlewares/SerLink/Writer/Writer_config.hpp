/*
 * Writer_config.hpp
 *
 *  Created on: 31 Oct 2023
 *      Author: rw123
 */

 #ifndef WRITER_CONFIG_HPP_
 #define WRITER_CONFIG_HPP_
 
 #define WRITER_CONFIG__WRITER0
 
 
 //----------------------------------------------------------
 // WRITER0
 #ifdef WRITER_CONFIG__WRITER0
 

 #define WRITER_CONFIG__WRITER0_ID 1 // WRITER0 instance id
 
 #endif
 //----------------------------------------------------------
 
 
 
 //----------------------------------------------------------
 // WRITER1 - SerLink1, over the radio. Its link is the writeData function
 // passed to init(), so it has no WRITER_CONFIG__WRITER1 switch.
 #define WRITER_CONFIG__WRITER1_ID 2 // WRITER1 instance id
 //----------------------------------------------------------

 //----------------------------------------------------------
 // WRITER2 - SerLink2, over MQTT. Its link is the writeData function
 // passed to init() (SerLinkMqttAdapter::write()), so no switch either.
 #define WRITER_CONFIG__WRITER2_ID 3 // WRITER2 instance id
 //----------------------------------------------------------

 #endif /* WRITER_CONFIG_HPP_ */
 