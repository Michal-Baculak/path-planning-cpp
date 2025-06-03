#include "geometric_trajectory_optimizer.hpp"

namespace global_planning
{

  Point operator-(const Point &a, const Point &b)
  {
    return {a.x - b.x, a.y - b.y};
  }
  Point operator+(const Point &a, const Point &b)
  {
    return {a.x + b.x, a.y + b.y};
  }
  Point operator*(const Point &a, const double &scalar)
  {
    return {a.x * scalar, a.y * scalar};
  }

  bool operator==(const Point &a, const Point &b)
  {
    return (a.x == b.x) && (a.y == b.y);
  }
  bool operator!=(const Point &a, const Point &b)
  {
    return (a.x != b.x) || (a.y != b.y);
  }

  const std::unordered_map<std::string, nlopt::vfunc> GeometricTrajectoryOptimizer::strToVfuncMap =
      {
          {"k2", k2ObjectiveFunction},
          {"k2Grad", k2GradObjectiveFunction},
          {"l2", l2ObjectiveFunction},
          {"l2Grad", l2GradObjectiveFunction},
          {"l", lObjectiveFunction},
          {"lGrad", lGradObjectiveFunction},
          {"k2L", k2LObjectiveFunction},
          {"k2LGrad", k2LGradObjectiveFunction},
          {"k2L2", k2L2ObjectiveFunction},
          {"k2L2Grad", k2L2GradObjectiveFunction}};

  double GeometricTrajectoryOptimizer::getAngle(const Point &A, const Point &B, const Point &C)
  {
    Point l1 = B - A;
    Point l2 = C - B;
    double a1 = atan2(l1.y, l1.x);
    double a2 = atan2(l2.y, l2.x);
    double a = a2 - a1;
    if (a > M_PI)
      a = a - 2 * M_PI;
    if (a < -M_PI)
      a = a + 2 * M_PI;
    return a;
  }
  std::vector<double> GeometricTrajectoryOptimizer::angleProfile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    auto pts = getPoints(base, alphas);
    std::vector<double> output(alphas.size(), 0.0);
    pts.insert(pts.begin(), pts.back());
    pts.push_back(pts.at(1));
    for (int i = 1; i < alphas.size() + 1; i++)
      output.at(i - 1) = getAngle(pts.at(i - 1), pts.at(i), pts.at(i + 1));
    return output;
  }
  std::vector<double> GeometricTrajectoryOptimizer::angleProfile(const std::vector<Point> &pts)
  {
    std::vector<double> output;
    auto _pts = pts;
    output = std::vector<double>(_pts.size() - 1, 0.0);
    _pts.push_back(_pts.at(0));
    _pts.insert(_pts.begin(), _pts.at(_pts.size() - 2));
    for (int i = 1; i < output.size() + 1; i++)
      output.at(i - 1) = getAngle(_pts.at(i - 1), _pts.at(i), _pts.at(i + 1));
    return output;
  }
  std::vector<double> GeometricTrajectoryOptimizer::distanceProfile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    auto pts = getPoints(base, alphas);
    std::vector<double> output(alphas.size(), 0.0);
    pts.push_back(pts.front());
    for (int i = 0; i < alphas.size(); i++)
      output.at(i) = distance(pts.at(i), pts.at(i + 1));
    return output;
  }
  std::vector<double> GeometricTrajectoryOptimizer::distanceProfile(const std::vector<Point> &pts)
  {
    std::vector<double> output;
    auto _pts = pts;
    _pts.push_back(_pts.at(0));
    output = std::vector<double>(pts.size(), 0.0);
    for (int i = 0; i < output.size(); i++)
      output.at(i) = distance(_pts.at(i), _pts.at(i + 1));
    return output;
  }
  std::vector<double> GeometricTrajectoryOptimizer::distance2Profile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    auto pts = getPoints(base, alphas);
    std::vector<double> output(alphas.size(), 0.0);
    pts.push_back(pts.front());
    for (int i = 0; i < alphas.size(); i++)
      output.at(i) = pow(pts.at(i).x - pts.at(i + 1).x, 2) + pow(pts.at(i).y - pts.at(i + 1).y, 2);
    return output;
  }

  std::vector<double> GeometricTrajectoryOptimizer::curvatureProfile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    auto angle_prof = angleProfile(base, alphas);
    auto dist_prof = distanceProfile(base, alphas);
    std::vector<double> output(alphas.size(), 0.0);
    for (int i = 0; i < alphas.size(); i++)
      output.at(i) = angle_prof.at(i) / dist_prof.at(i);
    return output;
  }
  std::vector<double> GeometricTrajectoryOptimizer::curvatureProfile(const std::vector<Point> &pts)
  {
    auto angle_prof = angleProfile(pts);
    auto dist_prof = distanceProfile(pts);
    std::vector<double> output(angle_prof.size(), 0.0);
    for (int i = 0; i < angle_prof.size(); i++)
      output.at(i) = angle_prof.at(i) / dist_prof.at(i);
    return output;
  }
  std::vector<double> GeometricTrajectoryOptimizer::curvature2Profile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    auto angle_prof = angleProfile(base, alphas);
    auto dist_prof = distanceProfile(base, alphas);
    std::vector<double> output(alphas.size(), 0.0);
    for (int i = 0; i < alphas.size(); i++)
      output.at(i) = pow(angle_prof.at(i) / dist_prof.at(i), 2);
    return output;
  }

  std::vector<Point> GeometricTrajectoryOptimizer::getPoints(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    std::vector<Point> P;
    for (size_t i = 0; i < alphas.size(); ++i)
    {
      Point A = {base[i][0], base[i][1]};
      Point B = {base[i][2], base[i][3]};

      Point new_point = {A.x + alphas[i] * (B.x - A.x),
                         A.y + alphas[i] * (B.y - A.y)};

      P.push_back(new_point);
    }
    return P;
  }

  double GeometricTrajectoryOptimizer::distance(const Point &a, const Point &b)
  {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
  }
  bool GeometricTrajectoryOptimizer::crossesBetween(const Point &A, const Point &B, const Point &origin, const Point &direction, Point *intersection)
  {
    Point D = {origin.x + direction.x, origin.y + direction.y};

    double x1 = A.x, y1 = A.y;
    double x2 = B.x, y2 = B.y;
    double x3 = origin.x, y3 = origin.y;
    double x4 = D.x, y4 = D.y;

    double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::fabs(denominator) < std::numeric_limits<double>::epsilon())
    {
      return false;
    }
    double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;

    if (t >= 0 && t <= 1)
    {
      intersection->x = x1 + t * (x2 - x1);
      intersection->y = y1 + t * (y2 - y1);
      return true;
    }
    return false;
  }
  double GeometricTrajectoryOptimizer::linearInterp(const std::vector<double> &t, const std::vector<double> &v, double s)
  {
    for (size_t i = 0; i < t.size() - 1; ++i)
    {
      if (s >= t[i] && s <= t[i + 1])
      {
        double ratio = (s - t[i]) / (t[i + 1] - t[i]);
        return v[i] + ratio * (v[i + 1] - v[i]);
      }
    }
    throw std::runtime_error("Interpolation at \"s = " + std::to_string(s) + "\" is undefined!");
  }

  std::vector<std::vector<double>> GeometricTrajectoryOptimizer::parametrizeGradual(
      std::vector<Point> inner_cones,
      std::vector<Point> outer_cones,
      double ds)
  {
    std::vector<std::vector<double>> base;

    std::vector<double> t_orig, origins_x, origins_y;
    std::vector<double> t_dir, dir_x, dir_y;

    double total_length = 0.0;
    std::vector<Point> cones;

    outer_cones.push_back(outer_cones.front());

    cones.push_back(inner_cones.back());
    cones.insert(cones.end(), inner_cones.begin(), inner_cones.end());
    cones.push_back(inner_cones.front());
    cones.push_back(inner_cones[1]);

    for (size_t i = 0; i < inner_cones.size(); ++i)
    {
      t_orig.push_back(total_length);
      origins_x.push_back(cones[i + 1].x);
      origins_y.push_back(cones[i + 1].y);
      total_length += distance(cones[i + 1], cones[i + 2]);
    }

    t_orig.push_back(total_length);
    origins_x.push_back(cones[1].x);
    origins_y.push_back(cones[1].y);

    double s = 0;
    for (size_t i = 0; i < inner_cones.size() + 2; ++i)
    {
      double dx = cones[i + 1].x - cones[i].x;
      double dy = cones[i + 1].y - cones[i].y;
      double dist = distance(cones[i + 1], cones[i]);
      s += dist;

      t_dir.push_back(s - dist / 2);
      dir_x.push_back(-dy);
      dir_y.push_back(dx);
    }
    // t_dir[0] -= distance(cones[1], cones[2]); //THIS IS WRONG LOOKIN
    // This should do the trick
    double offset = distance(cones[0], cones[1]);
    for (double &t : t_dir)
      t -= offset;

    for (double s = 0; s <= total_length; s += ds)
    {
      double p1_x = linearInterp(t_orig, origins_x, s);
      double p1_y = linearInterp(t_orig, origins_y, s);
      double k_x = linearInterp(t_dir, dir_x, s);
      double k_y = linearInterp(t_dir, dir_y, s);

      Point p1 = {p1_x, p1_y};
      Point k_n = {k_x, k_y};

      std::vector<Point> intersections;
      for (size_t j = 0; j < outer_cones.size() - 1; ++j)
      {
        Point p;
        if (crossesBetween(outer_cones[j], outer_cones[j + 1], p1, k_n, &p))
        {
          intersections.push_back(p);
        }
      }

      if (intersections.empty())
        continue;

      Point p2 = intersections.front();
      double min_dist = distance(p1, p2);
      for (const auto &p : intersections)
      {
        double dst = distance(p1, p);
        if (dst < min_dist)
        {
          min_dist = dst;
          p2 = p;
        }
      }

      if (!base.empty())
      {
        Point prev_A = {base.back()[0], base.back()[1]};
        Point prev_B = {base.back()[2], base.back()[3]};
        Point temp;
        if (crossesBetween(prev_A, prev_B, p1, {p2.x - p1.x, p2.y - p1.y}, &temp))
        {
          continue;
        }
      }

      base.push_back({p1.x, p1.y, p2.x, p2.y});
    }
    return base;
  }

  std::vector<double> GeometricTrajectoryOptimizer::gradL2(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    std::vector<double> grad(alphas.size(), 0.0);
    auto pts = getPoints(base, alphas);
    // add last element to the front and first element to the back
    pts.insert(pts.begin(), pts.back());
    pts.push_back(pts.at(1));
    for (size_t i = 1; i < alphas.size() + 1; i++)
    {
      double x_im1 = pts.at(i).x - pts.at(i - 1).x;
      double y_im1 = pts.at(i).y - pts.at(i - 1).y;
      double x_i = pts.at(i + 1).x - pts.at(i).x;
      double y_i = pts.at(i + 1).y - pts.at(i).y;
      double x_bi0 = base.at(i - 1).at(0);
      double y_bi0 = base.at(i - 1).at(1);
      double x_bi1 = base.at(i - 1).at(2);
      double y_bi1 = base.at(i - 1).at(3);
      grad.at(i - 1) = 2 * (x_bi1 - x_bi0) * (x_im1 - x_i) + 2 * (y_bi1 - y_bi0) * (y_im1 - y_i);
    }
    return grad;
  }
  double GeometricTrajectoryOptimizer::sum(const std::vector<double> &vec)
  {
    double output = 0;
    for (auto val : vec)
      if (!isnan(val))
        output += val;
    return output;
  }
  std::vector<double> GeometricTrajectoryOptimizer::gradK2(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    std::vector<double> grad(alphas.size(), 0.0);
    auto pts = getPoints(base, alphas);
    auto angles = angleProfile(base, alphas);
    // add last element to the front and first element to the back
    angles.insert(angles.begin(), angles.back());
    angles.push_back(angles.at(1));
    pts.insert(pts.begin(), pts.back());
    pts.push_back(pts.at(1));
    pts.push_back(pts.at(2));
    for (size_t i = 1; i < alphas.size() + 1; i++)
    {
      double x_im1 = pts.at(i).x - pts.at(i - 1).x;
      double y_im1 = pts.at(i).y - pts.at(i - 1).y;
      double x_i = pts.at(i + 1).x - pts.at(i).x;
      double y_i = pts.at(i + 1).y - pts.at(i).y;
      double x_ip1 = pts.at(i + 2).x - pts.at(i + 1).x;
      double y_ip1 = pts.at(i + 2).y - pts.at(i + 1).y;
      double A = -angles.at(i - 1);
      double B = angles.at(i);
      double C = base.at(i - 1).at(2) - base.at(i - 1).at(0); // base is indexed with i - 1, because we did not append last element to the front
      double D = base.at(i - 1).at(3) - base.at(i - 1).at(1);
      double E = x_i * x_i + y_i * y_i;
      double F = x_im1 * x_im1 + y_im1 * y_im1;
      double G = angles.at(i + 1);
      double dkim1_dai = (2 * y_im1 * A - A * A * 2 * x_im1) / (F * F) * C +
                         (2 * x_im1 * (-A) - A * A * 2 * y_im1) * D / (F * F);
      double dki_dai = ((2 * y_i * (-B) * (-C) / E + 2 * y_im1 * B * C / F + 2 * x_i * B * (-D) / E + 2 * x_im1 * (-B) * D / F) * E -
                        B * B * (2 * x_i * (-C) + 2 * y_i * (-D))) /
                       (E * E);
      double dkip1_dai = (2 * y_i * G * (-C) / E + 2 * x_i * (-G) * (-D) / E) / (x_ip1 * x_ip1 + y_ip1 * y_ip1);
      grad.at(i - 1) = dkim1_dai + dki_dai + dkip1_dai;
    }
    return grad;
  }
  std::vector<double> GeometricTrajectoryOptimizer::gradL(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    std::vector<double> grad(alphas.size(), 0.0);
    auto pts = getPoints(base, alphas);
    // add last element to the front and first element to the back
    pts.insert(pts.begin(), pts.back());
    pts.push_back(pts.at(1));
    for (size_t i = 1; i < alphas.size() + 1; i++)
    {
      double x_im1 = pts.at(i).x - pts.at(i - 1).x;
      double y_im1 = pts.at(i).y - pts.at(i - 1).y;
      double x_i = pts.at(i + 1).x - pts.at(i).x;
      double y_i = pts.at(i + 1).y - pts.at(i).y;
      double x_bi0 = base.at(i - 1).at(0);
      double y_bi0 = base.at(i - 1).at(1);
      double x_bi1 = base.at(i - 1).at(2);
      double y_bi1 = base.at(i - 1).at(3);
      grad.at(i - 1) = (x_bi1 - x_bi0) * (x_im1 / sqrt(x_im1 * x_im1 + y_im1 * y_im1) - x_i / sqrt(x_i * x_i + y_i * y_i)) +
                       (y_bi1 - y_bi0) * (y_im1 / sqrt(x_im1 * x_im1 + y_im1 * y_im1) - y_i / sqrt(x_i * x_i + y_i * y_i));
    }
    return grad;
  }
  std::vector<double> GeometricTrajectoryOptimizer::gradK2L(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas, double w)
  {
    // w = 0 -> optimize based on length
    // w = 1 -> optimize based on k2
    std::vector<double> grad(alphas.size(), 0.0);

    auto g_k2 = gradK2(base, alphas);
    auto g_l = gradL(base, alphas);

    const std::vector<double> alpha_ref(alphas.size(), 0.5);
    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2Profile(base, alpha_ref));
    auto l_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distanceProfile(base, alpha_ref));

    for (size_t i = 0; i < alphas.size(); i++)
      grad.at(i) = w * g_k2.at(i) / k2_ref + (1 - w) * g_l.at(i) / l_ref;
    return grad;
  }
  std::vector<double> GeometricTrajectoryOptimizer::gradK2L2(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas, double w)
  {
    // w = 0 -> optimize based on length2
    // w = 1 -> optimize based on k2
    std::vector<double> grad(alphas.size(), 0.0);

    auto g_k2 = gradK2(base, alphas);
    auto g_l2 = gradL2(base, alphas);

    const std::vector<double> alpha_ref(alphas.size(), 0.5);
    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2Profile(base, alpha_ref));
    auto l2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance2Profile(base, alpha_ref));

    for (size_t i = 0; i < alphas.size(); i++)
      grad.at(i) = w * g_k2.at(i) / k2_ref + (1 - w) * g_l2.at(i) / l2_ref;
    return grad;
  }

  double GeometricTrajectoryOptimizer::k2ObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
  {
    auto data = static_cast<ObjFunData *>(f_data);
    auto base = data->base;
    auto k2_prof = GeometricTrajectoryOptimizer::curvature2Profile(base, x);
    double output = 0;
    for (auto k2 : k2_prof)
      if (!isnan(k2))
        output += k2;
    return output;
  }
  double GeometricTrajectoryOptimizer::k2GradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
  {
    auto data = static_cast<ObjFunData *>(f_data);
    auto base = data->base;
    auto k2_prof = GeometricTrajectoryOptimizer::curvature2Profile(base, x);
    grad = GeometricTrajectoryOptimizer::gradK2(base, x);
    double output = 0;
    for (auto k2 : k2_prof)
      if (!isnan(k2))
        output += k2;
    return output;
  }
  double GeometricTrajectoryOptimizer::l2ObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
  {
    auto data = static_cast<ObjFunData *>(f_data);
    auto base = data->base;
    auto l2_prof = GeometricTrajectoryOptimizer::distance2Profile(base, x);
    double output = 0;
    for (auto l2 : l2_prof)
      if (!isnan(l2))
        output += l2;
    return output;
  }
  double GeometricTrajectoryOptimizer::l2GradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
  {
    auto data = static_cast<ObjFunData *>(f_data);
    auto base = data->base;
    auto l2_prof = GeometricTrajectoryOptimizer::distance2Profile(base, x);
    grad = GeometricTrajectoryOptimizer::gradL2(base, x);
    double output = 0;
    for (auto l2 : l2_prof)
      if (!isnan(l2))
        output += l2;
    return output;
  }
  double GeometricTrajectoryOptimizer::lObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
  {
    auto data = static_cast<ObjFunData *>(f_data);
    auto base = data->base;
    auto l_prof = GeometricTrajectoryOptimizer::distanceProfile(base, x);
    double output = 0;
    for (auto l : l_prof)
      if (!isnan(l))
        output += l;
    return output;
  }
  double GeometricTrajectoryOptimizer::lGradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
  {
    auto data = static_cast<ObjFunData *>(f_data);
    auto base = data->base;
    auto l_prof = GeometricTrajectoryOptimizer::distanceProfile(base, x);
    grad = GeometricTrajectoryOptimizer::gradL(base, x);
    double output = 0;
    for (auto l : l_prof)
      if (!isnan(l))
        output += l;
    return output;
  }
  double GeometricTrajectoryOptimizer::k2LObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
  {
    // w = 0 -> optimize based on length
    // w = 1 -> optimize based on k2
    const std::vector<double> alpha_ref(x.size(), 0.5);
    auto data = static_cast<ObjFunData *>(f_data);
    auto base = data->base;
    double w = data->w;

    auto k2 = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2Profile(base, x));
    auto l = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distanceProfile(base, x));

    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2Profile(base, alpha_ref));
    auto l_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distanceProfile(base, alpha_ref));
    std::string skibidi = "toilet"; // prod by Jakub Maslen
    double output = w * k2 / k2_ref + (1 - w) * l / l_ref;
    return output;
  }
  double GeometricTrajectoryOptimizer::k2LGradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
  {
    // w = 0 -> optimize based on length
    // w = 1 -> optimize based on k2
    const std::vector<double> alpha_ref(x.size(), 0.5);
    auto data = static_cast<ObjFunData *>(f_data);
    auto base = data->base;
    double w = data->w;
    grad = GeometricTrajectoryOptimizer::gradK2L(base, x, w);

    auto k2 = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2Profile(base, x));
    auto l = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distanceProfile(base, x));

    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2Profile(base, alpha_ref));
    auto l_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distanceProfile(base, alpha_ref));
    std::string skibidi = "toilet"; // prod by Jakub Maslen
    double output = w * k2 / k2_ref + (1 - w) * l / l_ref;
    return output;
  }
    double GeometricTrajectoryOptimizer::k2L2ObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
  {
    // w = 0 -> optimize based on length
    // w = 1 -> optimize based on k2
    const std::vector<double> alpha_ref(x.size(), 0.5);
    auto data = static_cast<ObjFunData *>(f_data);
    auto base = data->base;
    double w = data->w;

    auto k2 = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2Profile(base, x));
    auto l2 = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance2Profile(base, x));

    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2Profile(base, alpha_ref));
    auto l2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance2Profile(base, alpha_ref));
    double output = w * k2 / k2_ref + (1 - w) * l2 / l2_ref;
    return output;
  }
  double GeometricTrajectoryOptimizer::k2L2GradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
  {
    // w = 0 -> optimize based on length
    // w = 1 -> optimize based on k2
    const std::vector<double> alpha_ref(x.size(), 0.5);
    auto data = static_cast<ObjFunData *>(f_data);
    auto base = data->base;
    double w = data->w;
    grad = GeometricTrajectoryOptimizer::gradK2L2(base, x, w);

    auto k2 = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2Profile(base, x));
    auto l2 = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance2Profile(base, x));

    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2Profile(base, alpha_ref));
    auto l2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance2Profile(base, alpha_ref));
    double output = w * k2 / k2_ref + (1 - w) * l2 / l2_ref;
    return output;
  }

  Point line_intersect(const Point &A1, const Point &A2, const Point &B1, const Point &B2)
  {
    double x1 = A1.x, y1 = A1.y, x2 = A2.x, y2 = A2.y;
    double x3 = B1.x, y3 = B1.y, x4 = B2.x, y4 = B2.y;

    double t_n = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4));
    double t_d = ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    if (t_d == 0)
    {
      // paralel or coincident lines, from the use case we know they must be coincident
      // so we implement use case specific behaviour -> p = (A2+B1)/2
      return {(A2.x + B1.x) / 2, (A2.y + B1.y) / 2};
    }
    double t = t_n / t_d;
    return {x1 + t * (x2 - x1), y1 + t * (y2 - y1)};
  }

  // Offset points function
  std::vector<Point> offset_points(const Point &A, const Point &B, const Point &C, double offset, double dir)
  {
    Point V1 = B - A;
    Point V2 = C - B;

    Point N1 = {-dir * V1.y, dir * V1.x};
    Point N2 = {-dir * V2.y, dir * V2.x};

    N1 = N1.normalized() * offset;
    N2 = N2.normalized() * offset;

    return {A + N1, B + N1, B + N2, C + N2};
  }

  // Safety margin function for a single boundary
  std::vector<Point> safety_margin_oneline(const std::vector<Point> &in, double margin, double direction)
  {
    size_t len = in.size();
    std::vector<Point> out;

    for (size_t i = 0; i < len; ++i)
    {
      Point A = in[(i + len - 1) % len];
      Point B = in[i];
      Point C = in[(i + 1) % len];

      auto o_p = offset_points(A, B, C, margin, direction);
      Point p_new = line_intersect(o_p[0], o_p[1], o_p[2], o_p[3]);
      out.push_back(p_new);
    }

    return out;
  }

  // Main safety margin function
  std::pair<std::vector<Point>, std::vector<Point>> GeometricTrajectoryOptimizer::safetyMargin(const std::vector<Point> &inner, const std::vector<Point> &outer, double margin)
  {
    Point A = inner[1] - inner[0];
    Point B = outer[0] - inner[0];
    double dir = A.x * B.y - A.y * B.x;

    std::vector<Point> inner_new = safety_margin_oneline(inner, margin, dir);
    std::vector<Point> outer_new = safety_margin_oneline(outer, margin, -dir);
    return std::pair<std::vector<Point>, std::vector<Point>>(inner_new, outer_new);
  }

  bool GeometricTrajectoryOptimizer::updateTrajectory(const std::vector<Point> &inner_cones, const std::vector<Point> &outer_cones)
  {
    inner_cones_ = inner_cones;
    outer_cones_ = outer_cones;

    auto track_margined = safetyMargin(inner_cones, outer_cones, config_.safety_margin);
    auto inner = track_margined.first;
    auto outer = track_margined.second;
    base_ = GeometricTrajectoryOptimizer::parametrizeGradual(inner, outer, config_.parametrization_spacing);

    // if parametrization adds and extra line, or this is the first run, initialize alphas
    // otherwise last iteration's alphas are used as a starting point for current optimization
    // also clear velocity profile, because the dimensions won't match anymore
    if (base_.size() != alphas_.size())
    {
      alphas_ = std::vector<double>(base_.size(), 0.5);
      v_prof_.clear();
    }

    nlopt::opt opt(config_.nlopt_algorithm, alphas_.size());
    ObjFunData data{};
    data.base = base_;
    data.w = config_.k_l_weight;

    opt.set_min_objective(config_.objective_function, static_cast<void *>(&data)); // TODO: implement data to each objective function

    opt.set_lower_bounds(0.0);
    opt.set_upper_bounds(1.0);

    // Multiple stopping criteria are possible, choose at least one, not setting results in that criteria not being applied
    // https://nlopt.readthedocs.io/en/latest/NLopt_C-plus-plus_Reference/#stopping-criteria
    // opt.set_maxeval(40e3);
    // opt.set_ftol_abs(1e-24);
    // opt.set_ftol_rel(1e-18);

    opt.set_xtol_rel(config_.x_tol_rel);
    opt.set_xtol_abs(config_.x_tol_abs);

    // much shorter than needed on purpose
    if (config_.enable_time_limit)
      opt.set_maxtime(config_.time_limit);

    double opt_val = 0;
    nlopt::result result = opt.optimize(alphas_, opt_val);
    if (result >= 1)
    {
      if (result == nlopt::MAXTIME_REACHED)
      {
        return false;
      }
      return true;
    }
    else
    {
      throw std::runtime_error("Optimization failed with NLOPT return code: " + result);
    }
  }
  GeometricTrajectoryOptimizer::Config &GeometricTrajectoryOptimizer::getConfig() { return config_; }
  GeometricTrajectoryOptimizer::VehicleModel &GeometricTrajectoryOptimizer::getVehicleModel() { return vehicle_model_; }
  void GeometricTrajectoryOptimizer::setConfig(const GeometricTrajectoryOptimizer::Config &config) { config_ = config; }
  void GeometricTrajectoryOptimizer::setVehicleModel(const GeometricTrajectoryOptimizer::VehicleModel &vehicle_model) { vehicle_model_ = vehicle_model; }
  std::vector<Point> GeometricTrajectoryOptimizer::getTrajectory() const { return getPoints(base_, alphas_); }
  void GeometricTrajectoryOptimizer::updateRefSpeed(const Point &pose, double v0)
  {
    auto k_prof = curvatureProfile(base_, alphas_);
    auto d_prof = distanceProfile(base_, alphas_);

    std::vector<double> v_prof(alphas_.size(), 4.0);

    // find the starting index by finding the closest point
    auto pts = getPoints(base_, alphas_);
    double min_dist = distance(pose, pts.at(0));
    size_t i0 = 0;
    for (size_t i = 0; i < pts.size(); i++)
    {
      double d = distance(pose, pts.at(i));
      if (d < min_dist)
      {
        min_dist = d;
        i0 = i;
      }
    }
    size_t i0_p1 = (i0 + 1) % alphas_.size();
    // cannot evaluate angle if pts.at(i0) == pose, so check that first
    if (pts.at(i0) == pose || abs(getAngle(pose, pts.at(i0), pts.at(i0_p1))) < M_PI_2)
    {
      // closest point is behind or equal to current position
      // increment i0 to make sure the starting index is ahead and not behind
      i0 = i0_p1;
    }
    size_t lookahead_distance = alphas_.size();
    if (!v_prof_.empty())
    {
      v_prof = v_prof_;
      lookahead_distance -= 3; // recalculating whole length would affect ref speed directly ahead causing sudden spikes
    }

    // basic constraints - max cornering speed, top speed, steering speed
    for (size_t j = 0; j < lookahead_distance; j++)
    {
      size_t i = (i0 + j) % alphas_.size();
      size_t ip1 = (i0 + j + 1) % alphas_.size();
      double v_k = sqrt(vehicle_model_.a_lat_max / abs(k_prof.at(i)));
      double dk = abs(k_prof.at(i) - k_prof.at(ip1));
      v_prof.at(i) = fmin(v_k, vehicle_model_.v_max);
    }

    if (v_prof_.empty())
    {
      // calculate first ref speed considering available acceleration
      double a_lat = v0*v0*k_prof.at(i0);
      double a_res = vehicle_model_.a_front_max * sqrt(1 - pow(a_lat / vehicle_model_.a_lat_max, 2));
      v_prof.at(i0) = sqrt(v0*v0 + 2*a_res*distance(pose, pts.at(i0)));
    }
    else
    {
      v_prof.at(i0) = v_prof_.at(i0);
      // NOTE: for v_prof_.at(i0) to be valid, program needs to make sure, that when the track gets reparametrized
      // with increased number of lines, v_prof_ needs to be cleared, and therefore recalculated in the next iteration
      // instead of being based on existing outdated-parametrization profile (see parametrize)
    }

    // forward pass - consider residual acceleration left in corner for acceleration
    for (size_t j = 0; j < lookahead_distance - 1; j++)
    {
      size_t i = (i0 + j) % alphas_.size();
      size_t ip1 = (i0 + j + 1) % alphas_.size();
      if (v_prof.at(i) > v_prof.at(ip1))
        continue;
      double a_lat = v_prof.at(i) * v_prof.at(i) * k_prof.at(i);
      double a_res = vehicle_model_.a_front_max * sqrt(1 - pow(a_lat / vehicle_model_.a_lat_max, 2));
      double a_engine = vehicle_model_.max_power / (v_prof.at(i) * vehicle_model_.mass);
      double a_avail = fmin(a_res, a_engine);
      double v_avail = sqrt(v_prof.at(i) * v_prof.at(i) + 2 * a_avail * d_prof.at(i));
      v_prof.at(ip1) = fmin(v_prof.at(ip1), v_avail);
    }
    // backward pass - consider braking capabilites to make sure we can manage to brake in time
    for (size_t j = lookahead_distance - 1; j >= 1; j--)
    {
      size_t i = (i0 + j) % alphas_.size();
      size_t im1 = (i0 + j - 1) % alphas_.size();
      if (v_prof.at(i) > v_prof.at(im1))
        continue;
      double cX = 1 / (2 * d_prof.at(im1) * vehicle_model_.a_max_brake);
      double cY = k_prof.at(im1) / vehicle_model_.a_lat_max;
      double v_i = v_prof.at(i);
      double v_avail = sqrt(
          (2 * cX * cX * v_i * v_i + sqrt(4 * pow(cX, 4) * pow(v_i, 4) - 4 * (cX * cX + cY * cY) * (cX * cX * pow(v_i, 4) - 1))) / (2 * (cX * cX + cY * cY)));
      v_prof.at(im1) = fmin(v_prof.at(im1), v_avail);
    }
    v_prof_ = v_prof;
  }
  const std::vector<double> &GeometricTrajectoryOptimizer::getRefSpeed() const
  {
    return v_prof_;
  }

  double GeometricTrajectoryOptimizer::getLapTimeEst()
  {
    auto d_prof = distanceProfile(base_, alphas_);
    double t = 0;
    for (size_t i = 0; i < alphas_.size(); i++)
    {
      size_t ip1 = (i + 1) % alphas_.size();
      // get average speed
      double v_avg = (v_prof_.at(i) + v_prof_.at(ip1)) / 2;
      t += d_prof.at(i) / v_avg;
    }
    return t;
  }

  nlopt::algorithm GeometricTrajectoryOptimizer::intToNLOPTAlgorithm(int in)
  {
    return static_cast<nlopt::algorithm>(in);
  }

  nlopt::vfunc GeometricTrajectoryOptimizer::stringToObjectiveFunction(std::string in)
  {
    return GeometricTrajectoryOptimizer::strToVfuncMap.at(in);
  }
  void GeometricTrajectoryOptimizer::setToMidpath()
  {
    for (size_t i = 0; i < alphas_.size(); i++)
    {
      alphas_.at(i) = 0.5;
    }
  }
}
