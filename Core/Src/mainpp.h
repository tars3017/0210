/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus

extern int count;
extern double get_vel_x, get_vel_y, get_vel_z;

extern "C"
{
#endif

void setup(void);
void loop(void);
void publish_vel(double, double, double);

#ifdef __cplusplus
}
#endif

#endif /* MAINPP_H_ */
