#ifndef KALMAN_FILTER4_H_
#define KALMAN_FILTER4_H_

// altitude noise variance can be measured offline by calculating the 
// statistical variance in cm^2 of altitude samples from 
// the baro sensor at rest
#define KF_Z_MEAS_VARIANCE            200
// KF4 Acceleration Measurement Noise variance
#define KF_A_MEAS_VARIANCE   		50.0f
// This is set low as the residual acceleration bias after calibration
// is expected to have little variation/drift
#define KF_ACCELBIAS_VARIANCE   0.005f
// injects additional uncertainty depending on magnitude of acceleration
// helps respond quickly to large accelerations while heavily filtering
// in low acceleration situations.  Range : 0.5 - 1.5
#define KF_ADAPT			1.0f

void kalmanFilter4d_configure(float aVariance, float kAdapt, float zInitial, float vInitial, float aInitial);
void kalmanFilter4d_predict(float dt);
void kalmanFilter4d_update(float zm, float am, float* pz, float* pv);

#endif

