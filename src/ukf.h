#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "tools.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Radar measurement dimension
  int n_z_radar_;

  ///* Laser measurement dimension
  int n_z_laser_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  double NIS_laser_; // TODO set

  double NIS_radar_; // TODO set

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

private:

  Tools tools_;

  /**
   * @brief GenerateAugmentedSigmaPoints
   * @param Xsig_aug outarg: will be defined in the function
   */
  void GenerateAugmentedSigmaPoints(MatrixXd& Xsig_aug);

  /**
   * @brief SigmaPointPrediction, defines the filter member Xsig_pred_
   * @param Xsig_aug const
   * @param delta_t const
   */
  void SigmaPointPrediction(const Eigen::MatrixXd &Xsig_aug, const double delta_t);

  /**
   * @brief PredictMeanAndCovariance, changes x_ and P_ to the predicted state
   */
  void PredictMeanAndCovariance();

  /**
   * @brief PredictLaserMeasurement
   * @param Zsig outarg: will be defined in the function
   * @param z_pred outarg: will be defined in the function
   * @param S outarg: will be defined in the function
   */
  void PredictLaserMeasurement(Eigen::MatrixXd& Zsig, Eigen::MatrixXd& z_pred, Eigen::MatrixXd& S);

  /**
   * @brief PredictRadarMeasurement
   * @param Zsig outarg: will be defined in the function
   * @param z_pred outarg: will be defined in the function
   * @param S outarg: will be defined in the function
   */
  void PredictRadarMeasurement(Eigen::MatrixXd &Zsig, Eigen::MatrixXd &z_pred, Eigen::MatrixXd &S);

  /**
   * @brief UpdateState, can be used for both laser and radar measurements
   * @param Zsig
   * @param z_pred
   * @param S
   * @param z
   * @param n_z
   */
  void UpdateState(const Eigen::MatrixXd &Zsig, const Eigen::VectorXd &z_pred, const Eigen::MatrixXd &S, const Eigen::VectorXd &z, const int n_z);

};

#endif /* UKF_H */
