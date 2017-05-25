# CarND-Unscented-Kalman-Filter-Project-mwolfram

* There are two functioning ```main.cpp``` versions, ```main.cpp.default``` and ```main.cpp.test```
* Currently, ```main.cpp``` is equal to ```main.cpp.default``` and contains the code to run the filter on a given dataset.
* The ```main.cpp.test``` version contains a set of test cases to verify the individual steps of the pipeline. This is based on the test data that was used in class for the programming assignments.
* For running ```main.cpp.test```, public access has to be granted to all methods in ```ukf.h```, by simply commenting out line 119.

## Results

#### RMSE

```
0.0621695
0.0829473
 0.324728
  0.21989
```

#### NIS

* **2%** of the laser measurements had a higher NIS than 6
* **4.8%** of the radar measurements had a higher NIS than 7.8

#### Settings for process noise

* Process noise standard deviation longitudinal acceleration: **0.6** m/s^2
* Process noise standard deviation yaw acceleration: **0.5** rad/s^2:
