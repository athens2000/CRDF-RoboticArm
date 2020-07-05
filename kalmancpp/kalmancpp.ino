#ifndef _Kalman_h_
#define _Kalman_h_

class Kalman {
public:
    Kalman();

    
    float getAngle(float newAngle, float newRate, float dt);

    void setAngle(float angle); // Used to set angle, 
    float getRate(); // Return the unbiased rate

    /*  used to tune the Kalman filter */
    void setQangle(float Q_angle);
   
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQangle();
    float getQbias();
    float getRmeasure();

private:
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter
    float bias; // The gyro bias calculated by the Kalman filter
    float rate; // Unbiased rate calculated from the rate and the calculated bias 

    float P[2][2]; // Error covariance matrix 
};

#endif
