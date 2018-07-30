# CarND_Project07UnscentedKalmanFilter

Solution for Udacity Self driving car Nano degree seventh project: Unscented Kalman Filter Project.

---

Abbreviations:

KF:  Kalman Filter.

EKF: Extended Kalman Filter.

UKF: Unscented Kalman Filter

[//]: # (Image Referensers)

[LRImg]: ./images/Output/L_R_DS1.png
[LR2Img]: ./images/Output/L_R_DS2.png
[LImg]: ./images/Output/L_DS1.png
[L2Img]: ./images/Output/L_DS2.png
[RImg]: ./images/Output/R_DS1.png
[R2Img]: ./images/Output/R_DS2.png

[NISImg]: ./Tuning/03_L0.3_P0.4/Figure_1.png

---

## Objective

Implement an Unscented Kalman filter tracking a bicycle motion, using measurements from a LIDAR and a RADAR to perform sensor fusion on those measuerments and reach a set of performance metrics defined by Udacity.

---

### Reflection

This project aims to enhance improve upon the achieved results during previous sensor fusion project, i.e. EKF, by implementing a UKF. There are two datasets representing measurements from a LIDAR and a RADAR including both the sensor measurements and the ground truth to be used as a metrics during performance evaluation.

Udacity provided a simulator that allows visualize of the different lidar, radar measurement points and the output UKF data points.

There are two different types of readings based on the sensor type:-
  * LIDAR:
    - Provides a point cloud where each point has a cartesian coordinates system.
    - Not able to provide speed measurements.
  * RADAR:
    - Provides a range and bearing reading in a polar coordinates system.
    - Able to provide angular speed measurements.

The UKF needs to handle each type of measurements based on it's mentioned properties.

---

### Theory

In this project we apply a CTRV motion model thus the UKF state consists of:
    - Position in X axis.
    - Position in Y axis.
    - Tangential Velocity.
    - Yaw angle.
    - Yaw rate.

KF has two main stages:
  * Prediction:
    - Based on the pre-defined motion model the KF can estimate, i.e. predict, the motion of the tracked object and determine it's new    state and covariance variables.
  * Measurement update:
    - In this stage the KF use the sensor measurements, taking in consideration the uncertainty in sensor measurements, to correct any errors in prediction state.

- KF vs EKF vs UKF:
The KF has a restriction that the tracked process must have a linear property, if the tracked process is not linear then the equations of KF will not hold.

The EKF aim to fix this issue by linearizing the non-linear process at the point of estimation using a first Taylor expansion, i.e. Jacobian calculation.

The UKF targets a completely different approch by applying a numerical approximation trying to approximate the output of a non-linear process using certain key points, aka sigma points.

---

### Implementation

Udacity have provided most of the control code as:
  * Communication between UKF and Udacity simulator.
  * Parsing of the measurements points and feeding the values to the UKF.
  * Triggering the performance evaluation process.
  
I have made some modifications to this control code to achieve some of C++ concepts as data locality and encapsulation by setting UKF data class members as private.
  
I had to implement the following parts:
  * UKF: Complete implementation of the UKF project consisting of:
    - UKF state initialization
    - UKF numerical approximations calculation, i.e. sigma points generation and prediction.
    - Handling the measurements of the LIDAR and RADAR and processing each type of measurements based on the sensor properties mentioned before.
    - Calculating the NIS value for each type of sensor to support in tuning the filter by measuring it's consistency.

  * RMSE
    - Used as a metric during performance evaluation
    
I have added some pre-processor switch as well to:
  * Enable of disabling/enabling of handling one of the two sensors.
  * Enable various level of debugging of the code.


---

### Results

Udacity define that for a successful project submission your code must produce RMSE values equal or lower to [.09, .10, 0.40, 0.30] for data set 1.

After finishing the implementation, connecting to the simulator and tuning the filter, i was able to see that the RMSE error values were within the acceptable range defined by Udacity. Using the RADAR and LIDAR on dataset 1 [0.06, 0.092, 0.33, 0.22].

To tune the filter I had to take into consideration both the NIS and RMSE values, there is only two parameters to tune, process  longitudinal acceleration noise and process yaw acceleration noise.

We started with kind of very high values of 30.0 for both parameters, I tried lowering it gradually to 3 then 1.5 to see how the NIS and RMSE is affected.

I observed that the NIS points that the filter results is nearly consistent in all the values, as can be seen in images under the tuning folder and the RMSE value kept improving.

I decided to go to a more statistcal approch, since we are going to track a moving bicycle:
  - Logitudal accelration is stated in this paper, http://eprints.uwe.ac.uk/20767/, as being 0.231 m/s^2, I decided to set it as 0.3 to have a better handling of sudden movements.
  - Yaw accelration can be estimated as a 20-25 deg/s^2 which maps to 0.4 rad/s^2 which seems as a reasonable value.

Using these two parameters as the final parameters I observed the following NIS which is consistent as discussed during the lectures.
![NISImg][NISImg]

Following this, I have made a number of trials to analyze how the UKF performance and how much is the actual gain from the sensor fusion ?

|   Index  |    Sensors    |   Data Set    |            RMSE          |
|----------| ------------- | ------------- |--------------------------|
|     1    | RADAR + LIDAR |       1       | [0.06, 0.09, 0.33, 0.22] |
|     2    | RADAR + LIDAR |       2       | [0.09, 0.06, 0.65, 0.28] |
|     3    |     LIDAR     |       1       | [0.10, 0.10, 0.60, 0.23] |
|     4    |     LIDAR     |       2       | [0.10, 0.08, 0.51, 0.29] |
|     5    |     RADAR     |       1       | [0.17, 0.20, 0.37, 0.34] |
|     6    |     RADAR     |       2       | [0.32, 0.25, 0.67, 0.39] |

It is quite clear that the standalone RADAR performs much worse than standalone LIDAR.

But with the fusion between the LIDAR and the RADAR we obtain a 50.5% average improve over standalone LIDAR RMSEs.

The below images show the output of some of the sequences following the index in the table:

1) 
![LRImg][LRImg]

---

2) 
![LR2Img][LR2Img]

---

3) 
![LImg][LImg]

---

4)
![L2Img][L2Img]

---

5)
![RImg][RImg]

---

6)
![R2Img][R2Img]

---

### Conclusion

I think I have reached quite a good understanding of the theory and application of the EKF and how it works to achieve the goal of a high confidence in tracked objects.
