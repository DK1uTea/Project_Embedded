
#ifndef FALL_DETECTION_MODEL_H
#define FALL_DETECTION_MODEL_H

#include <Arduino.h>

void extract_features(float accel_x[], float accel_y[], float accel_z[], 
                     int window_size, float features[]);
int detect_activity(float features[]);

bool detect_fall(float features[]);

#endif // FALL_DETECTION_MODEL_H
