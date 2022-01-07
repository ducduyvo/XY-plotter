#ifndef MAIN_H_
#define MAIN_H_

/* X, Y stepper motor */
#define motorXPort      0
#define motorXPin       27
#define motorXDirPort   0
#define motorXDirPin    28

#define motorYPort
#define motorYPin
#define motorYDirPort
#define motorYDirPin


/* Limit switches */
#define limXMinPort   0
#define limXMinPin    0
#define limXMaxPort   1
#define limXMaxPin    3

#define limYMinPort   0
#define limYMinPin    29
#define limYMaxPort   0
#define limYMaxPin    9

/* Plotter setting */
#define mapWidth    340
#define mapHeight   310

void prvSetupHardware(void);
#endif /* MAIN_H_ */

