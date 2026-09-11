/*
 * Reader_config.hpp
 *
 *  Created on: 23 Apr 2023
 *      Author: rw123
 */

#ifndef READER_CONFIG_HPP_
#define READER_CONFIG_HPP_

#define READER_CONFIG__READER0

#define READER_CONFIG__MAX_NUM_INSTANT_HANDLERS 5

//----------------------------------------------------------
// READER0
#ifdef READER_CONFIG__READER0

#define READER_CONFIG__READER0_ID 1 // READER0 instance id

#endif
//----------------------------------------------------------

//----------------------------------------------------------
// READER1 - SerLink1, over the radio. It sends acks to the extTxOutQueue
// passed to init() rather than a uart, so it has no READER_CONFIG__READER1
// switch.
#define READER_CONFIG__READER1_ID 2 // READER1 instance id
//----------------------------------------------------------

#endif /* READER_CONFIG_HPP_ */
