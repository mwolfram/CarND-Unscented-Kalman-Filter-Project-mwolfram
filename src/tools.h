#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * @brief Tools::normalize
   * @param angle_rad
   * @return angle in radians normalized to range between -pi and pi
   */
  float normalize(const float angle_rad);

  /**
   * @brief Tools::calculateNIS
   * @param z
   * @param z_pred
   * @param S
   * @return
   */
  float calculateNIS(const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred, const Eigen::MatrixXd& S);

};

#endif /* TOOLS_H_ */
