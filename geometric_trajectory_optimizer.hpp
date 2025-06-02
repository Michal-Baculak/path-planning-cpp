#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <limits>
#include <Python.h>
#include <nlopt.hpp>
#include <bits/stdc++.h>
#include <stdexcept>
#include <unordered_map>

namespace global_planning
{

struct Point
{
  double x, y;
  double norm() const { return std::sqrt(x * x + y * y); }
  Point normalized() const
  {
    double n = norm();
    return {x / n, y / n};
  }
};
class GeometricTrajectoryOptimizer
{
  struct ObjFunData
  {
    std::vector<std::vector<double>> base;
    double w;
  };

public:
  struct Config
  {
    double safety_margin = 1.5;
    double parametrization_spacing = 3;
    double k_l_weight = 0.5;
    nlopt::algorithm nlopt_algorithm = nlopt::LD_SLSQP;
    nlopt::vfunc objective_function = k2LGradObjectiveFunction;
    bool enable_time_limit = false;
    double time_limit = 0.2;
    double x_tol_rel = 1e-9;  // 5e-3 still works
    double x_tol_abs = 1e-12; // 1e-2 still works
  };
  struct VehicleModel
  {
    double a_lat_max = 19.62;
    double a_front_max = 19.62;
    double a_max_brake = 19.62;
    double max_power = 100e3;
    double v_max = 33.3;
    double mass = 270;
    double c_steering = 1;
  };

private:
  const static std::unordered_map<std::string, nlopt::vfunc> strToVfuncMap;

  Config config_;
  VehicleModel vehicle_model_;

  std::vector<Point> inner_cones_;
  std::vector<Point> outer_cones_;
  std::vector<std::vector<double>> base_;
  std::vector<double> alphas_;
  std::vector<double> v_prof_;

  static double distance(const Point &a, const Point &b);
  static double linearInterp(const std::vector<double> &t, const std::vector<double> &v, double s);
  static bool crossesBetween(const Point &A, const Point &B, const Point &origin, const Point &direction, Point *intersection);
  static std::vector<std::vector<double>> parametrize(const std::vector<Point> &inner_cones, const std::vector<Point> &outer_cones, int resolution);
  static std::vector<std::vector<double>> parametrizeGradual(std::vector<Point> inner_cones, std::vector<Point> outer_cones, double ds, bool is_closed);
  static std::vector<double> angleProfile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas);
  static std::vector<double> angleProfile(const std::vector<Point> &pts);
  static std::vector<double> distanceProfile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas);
  static std::vector<double> distanceProfile(const std::vector<Point> &pts);
  static std::vector<double> distance2Profile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas);
  // static std::vector<double> distance2Profile(const std::vector<Point> &pts); // appears to be unrequired
  static std::vector<double> curvatureProfile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas);
  static std::vector<double> curvatureProfile(const std::vector<Point> &pts);
  static std::vector<double> curvature2Profile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas);
  // static std::vector<double> curvature2Profile(const std::vector<Point> &pts); // unused too

  // template <typename T> static T sumstd::vector(const std::vector<T>& vec); //overkill
  static double sum(const std::vector<double> &vec);

  static std::vector<double> gradK2(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas);
  static std::vector<double> gradL(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas);
  static std::vector<double> gradL2(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas);
  static std::vector<double> gradK2L(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas, double w);
  static std::vector<double> gradK2L2(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas, double w);

  static std::vector<Point> getPoints(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas);
  static double getAngle(const Point &A, const Point &B, const Point &C);
  static void plotDemo(const std::vector<double> &x, const std::vector<double> &y);
  static void plotAll(
      const std::vector<Point> &inner_cones,
      const std::vector<Point> &outer_cones,
      const std::vector<std::vector<double>> &base,
      const std::vector<double> &alphas);
  static std::vector<double> optimize(const std::vector<Point> &inner_cones, const std::vector<Point> &outer_cones, std::vector<std::vector<double>> &base);
  static std::pair<std::vector<Point>, std::vector<Point>> safetyMargin(const std::vector<Point> &inner, const std::vector<Point> &outer, double margin);

public:
  bool updateTrajectory(const std::vector<Point> &inner_cones, const std::vector<Point> &outer_cones);
  Config &getConfig();
  VehicleModel &getVehicleModel();
  void setConfig(const Config &config);
  void setVehicleModel(const VehicleModel &vehicle_model);
  std::vector<Point> getTrajectory() const;
  void updateRefSpeed(const Point &pose, double v0);
  void calcOptimalRefSpeed();
  const std::vector<double> &getRefSpeed() const;
  void plotAll();
  void logAll(std::string folder);
  double getLapTimeEst();
  void setToMidpath();
  static std::vector<double> updateRefSpeed(const Point &pose, double v0, const VehicleModel& model, const std::vector<Point>& points, std::vector<double>& v_prof_);

  static nlopt::algorithm intToNLOPTAlgorithm(int in);
  static nlopt::vfunc stringToObjectiveFunction(std::string in);
  // objective functions made accessible for config_
  // function signature in compliance with https://nlopt.readthedocs.io/en/latest/NLopt_C-plus-plus_Reference/
  static double k2ObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
  static double k2GradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
  static double l2ObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
  static double l2GradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
  static double lObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
  static double lGradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
  static double k2LObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
  static double k2LGradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
  static double k2L2ObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
  static double k2L2GradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
};

}
