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
    if (base.front() != base.back())
    {
      // open track (unconnected)
      output.at(0) = output.back() = nan("");
      for (int i = 1; i < alphas.size() - 1; i++)
        output.at(i) = getAngle(pts.at(i - 1), pts.at(i), pts.at(i + 1));
      return output;
    }

    // closed track
    pts.insert(pts.begin(), pts.back());
    pts.push_back(pts.at(1));
    for (int i = 1; i < alphas.size() + 1; i++)
      output.at(i - 1) = getAngle(pts.at(i - 1), pts.at(i), pts.at(i + 1));
    return output;
  }
  std::vector<double> GeometricTrajectoryOptimizer::angleProfile(const std::vector<Point> &pts)
  {
    std::vector<double> output;
    if (pts.front() != pts.back())
    {
      // open track (unconnected)
      output = std::vector<double>(pts.size(), 0.0);
      output.at(0) = output.back() = nan("");
      for (int i = 1; i < pts.size() - 1; i++)
        output.at(i) = getAngle(pts.at(i - 1), pts.at(i), pts.at(i + 1));
      return output;
    }

    // closed track
    auto _pts = pts;
    output = std::vector<double>(_pts.size() - 1, 0.0);
    _pts.insert(_pts.begin(), _pts.at(_pts.size() - 2));
    for (int i = 1; i < output.size() + 1; i++)
      output.at(i - 1) = getAngle(_pts.at(i - 1), _pts.at(i), _pts.at(i + 1));
    return output;
  }
  std::vector<double> GeometricTrajectoryOptimizer::distanceProfile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    auto pts = getPoints(base, alphas);
    std::vector<double> output(alphas.size(), 0.0);
    if (base.front() != base.back())
    {
      // open track
      output.back() = nan("");
      for (int i = 0; i < alphas.size() - 1; i++)
        output.at(i) = distance(pts.at(i), pts.at(i + 1));
      return output;
    }
    // closed track
    pts.push_back(pts.front());
    for (int i = 0; i < alphas.size(); i++)
      output.at(i) = distance(pts.at(i), pts.at(i + 1));
    return output;
  }
  std::vector<double> GeometricTrajectoryOptimizer::distance2Profile(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    auto pts = getPoints(base, alphas);
    std::vector<double> output(alphas.size(), 0.0);
    if (base.front() != base.back())
    {
      // open track
      output.back() = nan("");
      for (int i = 0; i < alphas.size() - 1; i++)
        output.at(i) = pow(pts.at(i).x - pts.at(i + 1).x, 2) + pow(pts.at(i).y - pts.at(i + 1).y, 2); // distance(pts.at(i), pts.at(i+1));
      return output;
    }
    // closed track
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
  std::vector<std::vector<double>> GeometricTrajectoryOptimizer::parametrize(const std::vector<Point> &inner_cones, const std::vector<Point> &outer_cones, int resolution)
  {
    std::vector<double> dist;
    std::vector<std::vector<double>> base;
    int n = inner_cones.size();

    // 1. Compute all distances between inner_cones
    for (int i = 0; i < n - 1; ++i)
    {
      dist.push_back(distance(inner_cones[i], inner_cones[i + 1]));
    }
    dist.push_back(distance(inner_cones[n - 1], inner_cones[0]));

    // 2. compute total length
    double len = 0;
    for (double d : dist)
    {
      len += d;
    }

    // 3. Determine step size from resolution
    double ds = len / resolution;

    // 4. Move along inner_cones with step size
    for (int i = 1; i <= resolution; ++i)
    {
      double s = i * ds;

      // find out, between which points we lie
      int pos = 0;
      while (s > 0 && pos < dist.size())
      {
        s -= dist[pos];
        pos++;
      }

      // undo last subtraction to find remaining dist
      pos--;
      s += dist[pos];

      Point k = {inner_cones[(pos + 1) % n].x - inner_cones[pos].x, inner_cones[(pos + 1) % n].y - inner_cones[pos].y};

      Point p1 = {inner_cones[pos].x + k.x * s / dist[pos], inner_cones[pos].y + k.y * s / dist[pos]};

      Point k_n = {-k.y, k.x};
      std::vector<Point> p_s;

      for (size_t j = 0; j < outer_cones.size() - 1; ++j)
      {
        Point p;
        if (crossesBetween(outer_cones[j], outer_cones[j + 1], p1, k_n, &p))
        {
          p_s.push_back(p);
        }
      }
      Point p;
      if (crossesBetween(outer_cones.back(), outer_cones.front(), p1, k_n, &p))
      {
        p_s.push_back(p);
      }

      if (p_s.empty())
        continue;

      Point p2 = p_s[0];
      double min_dist = distance(p1, p2);
      for (const auto &p : p_s)
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
      double ds,
      bool is_closed)
  {
    std::vector<std::vector<double>> base;

    std::vector<double> t_orig, origins_x, origins_y;
    std::vector<double> t_dir, dir_x, dir_y;

    double total_length = 0.0;
    std::vector<Point> cones;

    if (is_closed)
    {
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
    }
    else
    {
      // This part is not unit tested, run tests first before using
      cones = inner_cones;
      for (size_t i = 0; i < cones.size() - 1; ++i)
      {
        t_orig.push_back(total_length);
        origins_x.push_back(cones[i].x);
        origins_y.push_back(cones[i].y);
        total_length += distance(cones[i], cones[i + 1]);
      }
      t_orig.push_back(total_length);
      origins_x.push_back(cones.back().x);
      origins_y.push_back(cones.back().y);

      double s = 0;
      for (size_t i = 0; i < cones.size() - 1; ++i)
      {
        double dx = cones[i + 1].x - cones[i].x;
        double dy = cones[i + 1].y - cones[i].y;
        double dist = distance(cones[i + 1], cones[i]);
        s += dist;
        t_dir.push_back(s - dist / 2);
        dir_x.push_back(-dy);
        dir_y.push_back(dx);
      }

      t_dir.insert(t_dir.begin(), 0);
      t_dir.push_back(total_length);
      dir_x.insert(dir_x.begin(), dir_x.front());
      dir_x.push_back(dir_x.back());
      dir_y.insert(dir_y.begin(), dir_y.front());
      dir_y.push_back(dir_y.back());
    }

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

  void GeometricTrajectoryOptimizer::plotDemo(const std::vector<double> &x, const std::vector<double> &y)
  {
    Py_Initialize();
    PyRun_SimpleString("import sys; sys.path.append('.')");
    PyObject *pName = PyUnicode_DecodeFSDefault("plot_demo");
    PyObject *pModule = PyImport_Import(pName);
    Py_XDECREF(pName);
    if (!pModule)
    {
      std::cerr << "Failed to load Python module!" << std::endl;
      PyErr_Print();
      return;
    }
    PyObject *pFunc = PyObject_GetAttrString(pModule, "plot_demo");

    if (pFunc && PyCallable_Check(pFunc))
    {

      PyObject *pListX = PyList_New(x.size());
      PyObject *pListY = PyList_New(y.size());

      for (int i = 0; i < x.size(); i++)
        PyList_SetItem(pListX, i, PyFloat_FromDouble(x.at(i)));

      for (int i = 0; i < y.size(); i++)
        PyList_SetItem(pListY, i, PyFloat_FromDouble(y.at(i)));

      PyObject *pArgs = PyTuple_Pack(2, pListX, pListY);
      PyObject *pValue = PyObject_CallObject(pFunc, pArgs);
      Py_XDECREF(pArgs);
      Py_XDECREF(pListX);
      Py_XDECREF(pListY);

      if (pValue)
      {
        Py_XDECREF(pValue);
      }
    }
  }
  void GeometricTrajectoryOptimizer::plotAll(
      const std::vector<Point> &inner_cones,
      const std::vector<Point> &outer_cones,
      const std::vector<std::vector<double>> &base,
      const std::vector<double> &alphas)
  {
    // TODO: robust memory management
    Py_Initialize();
    PyRun_SimpleString("import sys; sys.path.append('.')");
    PyObject *pName = PyUnicode_DecodeFSDefault("plot_all");
    PyObject *pModule = PyImport_Import(pName);
    Py_XDECREF(pName);
    if (!pModule)
    {
      std::cerr << "Failed to load Python module \"plot_all\"!" << std::endl;
      PyErr_Print();
      Py_XDECREF(pModule);
      return;
    }
    PyObject *pFunc = PyObject_GetAttrString(pModule, "plot_all");

    if (pFunc && PyCallable_Check(pFunc))
    {

      PyObject *pListInnX = PyList_New(inner_cones.size());
      PyObject *pListInnY = PyList_New(inner_cones.size());
      PyObject *pListOutX = PyList_New(outer_cones.size());
      PyObject *pListOutY = PyList_New(outer_cones.size());
      PyObject *pListBase = PyList_New(base.size());
      PyObject *pListPtsX = PyList_New(alphas.size());
      PyObject *pListPtsY = PyList_New(alphas.size());

      for (int i = 0; i < inner_cones.size(); i++)
      {
        PyList_SetItem(pListInnX, i, PyFloat_FromDouble(inner_cones.at(i).x));
        PyList_SetItem(pListInnY, i, PyFloat_FromDouble(inner_cones.at(i).y));
      }

      for (int i = 0; i < outer_cones.size(); i++)
      {
        PyList_SetItem(pListOutX, i, PyFloat_FromDouble(outer_cones.at(i).x));
        PyList_SetItem(pListOutY, i, PyFloat_FromDouble(outer_cones.at(i).y));
      }
      // for memory management
      std::vector<PyObject *> baseLines;

      for (int i = 0; i < base.size(); i++)
      {
        PyObject *pLineBaseI = PyList_New(4);
        PyList_SetItem(pLineBaseI, 0, PyFloat_FromDouble(base.at(i).at(0)));
        PyList_SetItem(pLineBaseI, 1, PyFloat_FromDouble(base.at(i).at(1)));
        PyList_SetItem(pLineBaseI, 2, PyFloat_FromDouble(base.at(i).at(2)));
        PyList_SetItem(pLineBaseI, 3, PyFloat_FromDouble(base.at(i).at(3)));
        baseLines.push_back(pLineBaseI);
        PyList_SetItem(pListBase, i, pLineBaseI);
      }

      auto pts = getPoints(base, alphas);
      for (int i = 0; i < alphas.size(); i++)
      {
        PyList_SetItem(pListPtsX, i, PyFloat_FromDouble(pts.at(i).x));
        PyList_SetItem(pListPtsY, i, PyFloat_FromDouble(pts.at(i).y));
      }

      PyObject *pArgs = PyTuple_Pack(7, pListInnX, pListInnY, pListOutX, pListOutY, pListBase, pListPtsX, pListPtsY);
      PyObject *pValue = PyObject_CallObject(pFunc, pArgs);
      Py_XDECREF(pArgs);
      Py_XDECREF(pListInnX);
      Py_XDECREF(pListInnY);
      Py_XDECREF(pListOutX);
      Py_XDECREF(pListOutY);
      Py_XDECREF(pListBase);
      Py_XDECREF(pListPtsX);
      Py_XDECREF(pListPtsY);
      for (auto line : baseLines)
        Py_XDECREF(line);
      if (pValue)
      {
        Py_XDECREF(pValue);
      }
    }
    Py_XDECREF(pModule);
  }
  std::vector<double> GeometricTrajectoryOptimizer::gradL2(const std::vector<std::vector<double>> &base, const std::vector<double> &alphas)
  {
    std::vector<double> grad(alphas.size(), 0.0);
    if (base.front() != base.back())
    {
      return grad;
      std::cerr << "Open track gradient calculation is not yet supported!" << std::endl;
    }
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
    if (base.front() != base.back())
    {
      return grad;
      std::cerr << "Open track gradient calculation is not yet supported!" << std::endl;
    }
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
    if (base.front() != base.back())
    {
      return grad;
      std::cerr << "Open track gradient calculation is not yet supported!" << std::endl;
    }
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
    if (base.front() != base.back())
    {
      return grad;
      std::cerr << "Open track gradient calculation is not yet supported!" << std::endl;
    }
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
    if (base.front() != base.back())
    {
      return grad;
      std::cerr << "Open track gradient calculation is not yet supported!" << std::endl;
    }
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
  std::vector<double> GeometricTrajectoryOptimizer::optimize(const std::vector<Point> &inner_cones, const std::vector<Point> &outer_cones, std::vector<std::vector<double>> &base)
  {
    // https://nlopt.readthedocs.io/en/latest/NLopt_Reference/

    auto [inner, outer] = safetyMargin(inner_cones, outer_cones, 1.5);
    // base = GeometricTrajectoryOptimizer::parametrize(inner,outer, 100);
    base = GeometricTrajectoryOptimizer::parametrizeGradual(inner, outer, 3, true);
    // base = GeometricTrajectoryOptimizer::parametrize(inner_cones,outer_cones, 100);

    // some setups have varying results based on initial guess, 1.0 seems to be better for w_k2_l_grad OF (f.e.)
    std::vector<double> alphas(base.size(), 1.0);

    base.push_back(base.at(0));
    // nlopt::opt opt(nlopt::LN_BOBYQA, alphas.size());
    nlopt::opt opt(nlopt::LD_SLSQP, alphas.size());

    // LN_BOBYQA - perfect for gradient-free optimization
    // LD_TNEWTON - perfect for l and l2, lacks in k2
    // LD_SLSQP - okay for l and l2, perfect for k2
    // LD_LBFGS - perfect for w
    opt.set_min_objective(k2LGradObjectiveFunction, static_cast<void *>(&base));

    opt.set_lower_bounds(0.0);
    opt.set_upper_bounds(1.0);

    //"MaxFunctionEvaluations",10e3, "StepTolerance",1e-20
    opt.set_maxeval(40e3);
    opt.set_xtol_rel(1e-9);
    opt.set_xtol_abs(1e-12);

    // much shorter than needed on purpose
    //  opt.set_maxtime(0.2);

    // opt.set_ftol_abs(1e-24);
    // opt.set_ftol_rel(1e-18);

    double opt_k2 = 0;
    nlopt::result result = opt.optimize(alphas, opt_k2);
    if (result < 1)
    {
      alphas.clear();
      throw std::runtime_error("Optimization failed with NLOPT error code: " + result);
    }
    return alphas;
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
    auto [inner, outer] = safetyMargin(inner_cones, outer_cones, config_.safety_margin);
    base_ = GeometricTrajectoryOptimizer::parametrizeGradual(inner, outer, config_.parametrization_spacing, true);

    // if parametrization adds and extra line, or this is the first run, initialize alphas
    // otherwise last iteration's alphas are used as a starting point for current optimization
    // also clear velocity profile, because the dimensions won't match anymore
    if (base_.size() != alphas_.size())
    {
      alphas_ = std::vector<double>(base_.size(), 0.5);
      v_prof_.clear();
    }
    // close the track (only mode implemented)
    base_.push_back(base_.at(0));

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

    double opt_k2 = 0;
    nlopt::result result = opt.optimize(alphas_, opt_k2);
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
      double v_steering = vehicle_model_.c_steering * d_prof.at(i) / dk;
      v_prof.at(i) = fmin(fmin(v_k, v_steering), vehicle_model_.v_max);
    }

    if (v_prof_.empty())
    {
      v_prof.at(i0) = sqrt(v0*v0 + 2*vehicle_model_.a_front_max*distance(pose, pts.at(i0)));
    }
    else
    {
      v_prof.at(i0) = v_prof_.at(i0);
      // NOTE: for v_prof_.at(i0) to be valid, program needs to make sure, that when the track gets reparametrized
      // with increased number of lines, v_prof_ needs to be cleared, and therefore recalculated in the next iteration
      // instead of being based on existing outdated-parametrization profile (see parametrize)
    }

    // std::cout << "v0 is set at index " << i0 << ", with value of v0 = " << v_prof.at(i0) << "\n";
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

  void GeometricTrajectoryOptimizer::plotAll()
  {
    plotAll(inner_cones_, outer_cones_, base_, alphas_);
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

  void GeometricTrajectoryOptimizer::logAll(std::string folder)
  {
    std::ofstream trackLeftFile(folder + "trackLeft.txt");
    std::ofstream trackRightFile(folder + "trackRight.txt");
    std::ofstream baseFile(folder + "base.txt");
    std::ofstream alphasFile(folder + "alphas.txt");
    std::ofstream velProfFile(folder + "velProf.txt");
    std::ofstream miscFile(folder + "misc.txt");

    if (!trackLeftFile.is_open() || !trackRightFile.is_open() || !baseFile.is_open() || !alphasFile.is_open() || !velProfFile.is_open() || !miscFile.is_open())
    {
      std::cerr << "Failed to open one of the files for logging!\n";
      return;
    }

    // log track
    for (auto p : inner_cones_)
      trackLeftFile << p.x << " " << p.y << "\n";

    for (auto p : outer_cones_)
      trackRightFile << p.x << " " << p.y << "\n";

    // log base
    for (auto bi : base_)
      baseFile << bi.at(0) << " " << bi.at(1) << " " << bi.at(2) << " " << bi.at(3) << "\n";

    // log alphas
    for (auto a : alphas_)
      alphasFile << a << "\n";

    // log velocity profile
    for (auto v : v_prof_)
      velProfFile << v << "\n";

    // log miscellaneous
    double t_est = getLapTimeEst();
    double k2 = sum(curvature2Profile(base_, alphas_));
    double d = sum(distanceProfile(base_, alphas_));
    miscFile << d << "\n"
             << k2 << "\n"
             << t_est << "\n";

    trackLeftFile.close();
    trackRightFile.close();
    baseFile.close();
    alphasFile.close();
    velProfFile.close();
    miscFile.close();
  }

  nlopt::algorithm GeometricTrajectoryOptimizer::intToNLOPTAlgorithm(int in)
  {
    return static_cast<nlopt::algorithm>(in);
  }

  nlopt::vfunc GeometricTrajectoryOptimizer::stringToObjectiveFunction(std::string in)
  {
    return GeometricTrajectoryOptimizer::strToVfuncMap.at(in);
  }
}
