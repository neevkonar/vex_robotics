#pragma once

/* These prevent header files from being included multiple times*/


void drivePID(double targetInches, double kP, double kI, double kD);
void turnPID(double targetAngle, double kP, double kI, double kD);


