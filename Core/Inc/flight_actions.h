/*
 * flight_actions.h
 *
 *  Created on: Feb 5, 2025
 *      Author: bencekanyok
 */

#ifndef INC_FLIGHT_ACTIONS_H_
#define INC_FLIGHT_ACTIONS_H_

void flightActions(double altitude, float mag, int *state);
void flightActionsLedTest(int *state);
void fillAltitude(double altitude);
void init_presets();

#endif /* INC_FLIGHT_ACTIONS_H_ */
