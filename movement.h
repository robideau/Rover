/*
 * movement.h
 *
 * Created: 1/27/2015 12:34:18 PM
 *  Author: robideau
 */ 

void moveForward(oi_t *sensor, int cm);

void moveBackward(oi_t *sensor, int cm);

void rotateClockwise(oi_t *sensor, int degrees);

void rotateClockwiseFine(oi_t *sensor, int degrees);

void rotateCounterClockwise(oi_t *sensor, int degrees);

void rotateCounterClockwiseFine(oi_t *sensor, int degrees);

void checkSensors(oi_t *sensor_data);

void colorCheck(int frontLeft, int left, int right, int frontRight);