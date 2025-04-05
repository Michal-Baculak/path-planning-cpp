#include "geometric_trajectory_optimizer.hpp"

using namespace std;
Point operator-(const Point& a, const Point& b) 
{
    return {a.x - b.x, a.y - b.y};
}
Point operator+(const Point& a, const Point& b) 
{
    return {a.x + b.x, a.y + b.y};
}
Point operator*(const Point& a, const double& scalar) 
{
    return {a.x * scalar, a.y * scalar};
}

bool operator==(const Point& a, const Point& b) 
{
    return (a.x == b.x) && (a.y == b.y); 
}
bool operator!=(const Point& a, const Point& b) 
{
    return (a.x != b.x) || (a.y != b.y); 
}

vector<double> GeometricTrajectoryOptimizer::velocityProfile(vector<Point> trajectory)
{
    cout << "velocityProfile unimplemented!";
    return {};
}
double GeometricTrajectoryOptimizer::get_angle(const Point& A, const Point& B, const Point& C)
{
    Point l1 = B-A;
    Point l2 = C-B;
    double a1 = atan2(l1.y,l1.x);
    double a2 = atan2(l2.y, l2.x);
    double a = a2 - a1;
    if(a > M_PI)
        a = a - 2*M_PI;
    if(a < -M_PI)
        a = a + 2*M_PI;
    return a;
}
vector<double> GeometricTrajectoryOptimizer::angle_profile(const vector<vector<double>>& base, const vector<double>& alphas)
{
    auto pts = get_points(base, alphas);
    vector<double> output(alphas.size(), 0.0);
    if(base.front() != base.back())
    {
        //open track (unconnected)
        output.at(0) = output.back() = nan("");
        for(int i = 1; i < alphas.size() - 1; i++)
            output.at(i) = get_angle(pts.at(i-1), pts.at(i), pts.at(i+1));
        return output;
    }

    // closed track
    pts.insert(pts.begin(), pts.back());
    pts.push_back(pts.at(1));
    for(int i = 1; i < alphas.size() + 1; i++)
        output.at(i-1) = get_angle(pts.at(i-1), pts.at(i), pts.at(i+1));
    return output;
}
vector<double> GeometricTrajectoryOptimizer::angle_profile(const vector<Point>& pts)
{
    vector<double> output;
    if(pts.front() != pts.back())
    {
        //open track (unconnected)
        output = vector<double>(pts.size(), 0.0);
        output.at(0) = output.back() = nan("");
        for(int i = 1; i < pts.size() - 1; i++)
            output.at(i) = get_angle(pts.at(i-1), pts.at(i), pts.at(i+1));
        return output;
    }

    // closed track
    auto _pts = pts;
    output = vector<double>(_pts.size()-1, 0.0);
    _pts.insert(_pts.begin(), _pts.at(_pts.size()-2));
    for(int i = 1; i < output.size() + 1; i++)
        output.at(i-1) = get_angle(_pts.at(i-1), _pts.at(i), _pts.at(i+1));
    return output;
}
vector<double> GeometricTrajectoryOptimizer::distance_profile(const vector<vector<double>>& base, const vector<double>& alphas)
{
    auto pts = get_points(base, alphas);
    vector<double> output(alphas.size(), 0.0);
    if(base.front() != base.back())
    {
        // open track
        output.back() = nan("");
        for(int i = 0; i < alphas.size() -1; i++)
            output.at(i) = distance(pts.at(i), pts.at(i+1));
        return output;
    }
    //closed track
    pts.push_back(pts.front());
    for(int i = 0; i< alphas.size(); i++)
        output.at(i) = distance(pts.at(i), pts.at(i+1));
    return output;
}
vector<double> GeometricTrajectoryOptimizer::distance2_profile(const vector<vector<double>>& base, const vector<double>& alphas)
{
    auto pts = get_points(base, alphas);
    vector<double> output(alphas.size(), 0.0);
    if(base.front() != base.back())
    {
        // open track
        output.back() = nan("");
        for(int i = 0; i < alphas.size() -1; i++)
            output.at(i) = pow(pts.at(i).x-pts.at(i+1).x,2) + pow(pts.at(i).y - pts.at(i+1).y,2); //distance(pts.at(i), pts.at(i+1));
        return output;
    }
    //closed track
    pts.push_back(pts.front());
    for(int i = 0; i< alphas.size(); i++)
        output.at(i) = pow(pts.at(i).x-pts.at(i+1).x,2) + pow(pts.at(i).y - pts.at(i+1).y,2);
    return output;
}
vector<double> GeometricTrajectoryOptimizer::curvature_profile(const vector<vector<double>>& base, const vector<double>& alphas)
{
    auto angle_prof = angle_profile(base, alphas);
    auto dist_prof = distance_profile(base, alphas);
    vector<double> output(alphas.size(), 0.0);
    for (int i = 0; i < alphas.size(); i++)
        output.at(i) = angle_prof.at(i)/dist_prof.at(i);
    return output;
}
vector<double> GeometricTrajectoryOptimizer::curvature2_profile(const vector<vector<double>>& base, const vector<double>& alphas)
{
    auto angle_prof = angle_profile(base, alphas);
    auto dist_prof = distance_profile(base, alphas);
    vector<double> output(alphas.size(), 0.0);
    for (int i = 0; i < alphas.size(); i++)
        output.at(i) = pow(angle_prof.at(i)/dist_prof.at(i), 2);
    return output;
}

vector<Point> GeometricTrajectoryOptimizer::get_points(const vector<vector<double>> &base, const vector<double> &alphas)
{
    vector<Point> P;
    for (size_t i = 0; i < alphas.size(); ++i) {
        Point A = {base[i][0], base[i][1]};
        Point B = {base[i][2], base[i][3]};
        
        Point newPoint = { A.x + alphas[i] * (B.x - A.x),
                           A.y + alphas[i] * (B.y - A.y) };
        
        P.push_back(newPoint);
    }
    return P;
}

double GeometricTrajectoryOptimizer::distance(const Point& a, const Point& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
bool GeometricTrajectoryOptimizer::crossesBetween(const Point& A, const Point& B, const Point& origin, const Point& direction, Point& intersection)
{
    Point D = {origin.x + direction.x, origin.y + direction.y};
    
    double x1 = A.x, y1 = A.y;
    double x2 = B.x, y2 = B.y;
    double x3 = origin.x, y3 = origin.y;
    double x4 = D.x, y4 = D.y;
    
    double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::fabs(denominator) < std::numeric_limits<double>::epsilon()) {
        return false;
    }
    double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
    
    if (t >= 0 && t <= 1) {
        intersection.x = x1 + t * (x2 - x1);
        intersection.y = y1 + t * (y2 - y1);
        return true;
    }
    return false;
}
std::vector<std::vector<double>> GeometricTrajectoryOptimizer::parametrize(const std::vector<Point>& innerCones, const std::vector<Point>& outerCones, int resolution) {
    std::vector<double> dist;
    std::vector<std::vector<double>> base;
    int n = innerCones.size();

    // 1. Compute all distances between innerCones
    for (int i = 0; i < n - 1; ++i) {
        dist.push_back(distance(innerCones[i], innerCones[i + 1]));
    }
    dist.push_back(distance(innerCones[n - 1], innerCones[0]));

    // 2. compute total length
    double len = 0;
    for (double d : dist) {
        len += d;
    }
    
    //3. Determine step size from resolution
    double ds = len / resolution;
    
    // 4. Move along innerCones with step size
    for (int i = 1; i <= resolution; ++i) {
        double s = i * ds;

        // find out, between which points we lie 
        int pos = 0;
        while (s > 0 && pos < dist.size()) {
            s -= dist[pos];
            pos++;
        }

        // undo last subtraction to find remaining dist
        pos--;
        s += dist[pos];
        
        Point k = {innerCones[(pos + 1) % n].x - innerCones[pos].x, innerCones[(pos + 1) % n].y - innerCones[pos].y};
        
        Point p1 = {innerCones[pos].x + k.x * s / dist[pos], innerCones[pos].y + k.y * s / dist[pos]};
        
        Point k_n = {-k.y, k.x};
        std::vector<Point> p_s;

        for (size_t j = 0; j < outerCones.size() - 1; ++j) {
            Point p;
            if (crossesBetween(outerCones[j], outerCones[j + 1], p1, k_n, p)) {
                p_s.push_back(p);
            }
        }
        Point p;
        if (crossesBetween(outerCones.back(), outerCones.front(), p1, k_n, p)) {
            p_s.push_back(p);
        }
        
        if (p_s.empty()) continue;
        
        Point p2 = p_s[0];
        double min_dist = distance(p1, p2);
        for (const auto& p : p_s) {
            double dst = distance(p1, p);
            if (dst < min_dist) {
                min_dist = dst;
                p2 = p;
            }
        }
        
        if (!base.empty()) {
            Point prev_A = {base.back()[0], base.back()[1]};
            Point prev_B = {base.back()[2], base.back()[3]};
            Point temp;
            if (crossesBetween(prev_A, prev_B, p1, {p2.x-p1.x, p2.y-p1.y}, temp)) {
                continue;
            }
        }
        
        base.push_back({p1.x, p1.y, p2.x, p2.y});
    }
    return base;
}
double linearInterp(const std::vector<double>& t, const std::vector<double>& v, double s) 
{
    for (size_t i = 0; i < t.size() - 1; ++i) {
        if (s >= t[i] && s <= t[i + 1]) {
            double ratio = (s - t[i]) / (t[i + 1] - t[i]);
            return v[i] + ratio * (v[i + 1] - v[i]);
        }
    }
    return v.back();  //TODO: fallback behaviour should either handle the error case or throw an error
}

std::vector<std::vector<double>> GeometricTrajectoryOptimizer::parametrize_gradual(
    std::vector<Point> innerCones,
    std::vector<Point> outerCones,
    double ds,
    bool is_closed
) {
    std::vector<std::vector<double>> base;

    std::vector<double> t_orig, origins_x, origins_y;
    std::vector<double> t_dir, dir_x, dir_y;

    double totalLength = 0.0;
    std::vector<Point> cones;

    if (is_closed) 
    {
        outerCones.push_back(outerCones.front());

        cones.push_back(innerCones.back());
        cones.insert(cones.end(), innerCones.begin(), innerCones.end());
        cones.push_back(innerCones.front());
        cones.push_back(innerCones[1]);

        for (size_t i = 0; i < innerCones.size(); ++i) 
        {
            t_orig.push_back(totalLength);
            origins_x.push_back(cones[i + 1].x);
            origins_y.push_back(cones[i + 1].y);
            totalLength += distance(cones[i + 1], cones[i + 2]);
        }

        t_orig.push_back(totalLength);
        origins_x.push_back(cones[1].x);
        origins_y.push_back(cones[1].y);

        double s = 0;
        for (size_t i = 0; i < innerCones.size() + 2; ++i) 
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
        //This should do the trick
        double offset = distance(cones[0], cones[1]);
        for(double& t:t_dir)
            t -= offset;
        
    } 
    else 
    {
        //This part is not unit tested, run tests first before using
        cones = innerCones;
        for (size_t i = 0; i < cones.size() - 1; ++i) 
        {
            t_orig.push_back(totalLength);
            origins_x.push_back(cones[i].x);
            origins_y.push_back(cones[i].y);
            totalLength += distance(cones[i], cones[i + 1]);
        }
        t_orig.push_back(totalLength);
        origins_x.push_back(cones.back().x);
        origins_y.push_back(cones.back().y);

        double s = 0;
        for (size_t i = 0; i < cones.size() - 1; ++i) {
            double dx = cones[i + 1].x - cones[i].x;
            double dy = cones[i + 1].y - cones[i].y;
            double dist = distance(cones[i + 1], cones[i]);
            s += dist;
            t_dir.push_back(s - dist / 2);
            dir_x.push_back(-dy);
            dir_y.push_back(dx);
        }

        t_dir.insert(t_dir.begin(), 0);
        t_dir.push_back(totalLength);
        dir_x.insert(dir_x.begin(), dir_x.front());
        dir_x.push_back(dir_x.back());
        dir_y.insert(dir_y.begin(), dir_y.front());
        dir_y.push_back(dir_y.back());
    }

    for (double s = 0; s <= totalLength; s += ds) {
        double p1_x = linearInterp(t_orig, origins_x, s);
        double p1_y = linearInterp(t_orig, origins_y, s);
        double k_x = linearInterp(t_dir, dir_x, s);
        double k_y = linearInterp(t_dir, dir_y, s);

        Point p1 = {p1_x, p1_y};
        Point k_n = {k_x, k_y};

        std::vector<Point> intersections;
        for (size_t j = 0; j < outerCones.size() - 1; ++j) {
            Point p;
            if (crossesBetween(outerCones[j], outerCones[j + 1], p1, k_n, p)) {
                intersections.push_back(p);
            }
        }

        if (intersections.empty()) continue;

        Point p2 = intersections.front();
        double min_dist = distance(p1, p2);
        for (const auto& p : intersections) {
            double dst = distance(p1, p);
            if (dst < min_dist) {
                min_dist = dst;
                p2 = p;
            }
        }

        if (!base.empty()) {
            Point prev_A = {base.back()[0], base.back()[1]};
            Point prev_B = {base.back()[2], base.back()[3]};
            Point temp;
            if (crossesBetween(prev_A, prev_B, p1, {p2.x-p1.x, p2.y-p1.y}, temp)) {
                continue;
            }
        }
        
        base.push_back({p1.x, p1.y, p2.x, p2.y});
    }
    return base;
}

void GeometricTrajectoryOptimizer::plot_demo(const vector<double>& x, const vector<double>& y)
{
    Py_Initialize();
    PyRun_SimpleString("import sys; sys.path.append('.')");
    PyObject* pName = PyUnicode_DecodeFSDefault("plot_demo");
    PyObject *pModule = PyImport_Import(pName);
    Py_XDECREF(pName);
    if(!pModule)
    {
        cerr << "Failed to load Python module!" << endl;
        PyErr_Print();
        return;
    }
    PyObject* pFunc = PyObject_GetAttrString(pModule, "plot_demo");

    if (pFunc && PyCallable_Check(pFunc)) {

        PyObject* pListX = PyList_New(x.size());
        PyObject* pListY = PyList_New(y.size());

        for (int i = 0; i < x.size(); i++)
            PyList_SetItem(pListX, i, PyFloat_FromDouble(x.at(i)));

        for (int i = 0; i < y.size(); i++)
            PyList_SetItem(pListY, i, PyFloat_FromDouble(y.at(i)));

        PyObject* pArgs = PyTuple_Pack(2, pListX, pListY);
        PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
        Py_XDECREF(pArgs);
        Py_XDECREF(pListX);
        Py_XDECREF(pListY);

        if (pValue) {
            Py_XDECREF(pValue);
        }
    }
}
void GeometricTrajectoryOptimizer::plot_all(
    const vector<Point>& innerCones, 
    const vector<Point>& outerCones,
    const vector<vector<double>>& base,
    const vector<double>& alphas
)
{
    //TODO: robust memory management
    Py_Initialize();
    PyRun_SimpleString("import sys; sys.path.append('.')");
    PyObject* pName = PyUnicode_DecodeFSDefault("plot_all");
    PyObject *pModule = PyImport_Import(pName);
    Py_XDECREF(pName);
    if(!pModule)
    {
        cerr << "Failed to load Python module \"plot_all\"!" << endl;
        PyErr_Print();
        Py_XDECREF(pModule);
        return;
    }
    PyObject* pFunc = PyObject_GetAttrString(pModule, "plot_all");

    if (pFunc && PyCallable_Check(pFunc)) {

        PyObject* pListInnX = PyList_New(innerCones.size());
        PyObject* pListInnY = PyList_New(innerCones.size());
        PyObject* pListOutX = PyList_New(outerCones.size());
        PyObject* pListOutY = PyList_New(outerCones.size());
        PyObject* pListBase = PyList_New(base.size());
        PyObject* pListPtsX = PyList_New(alphas.size());
        PyObject* pListPtsY = PyList_New(alphas.size());


        for (int i = 0; i < innerCones.size(); i++)
        {
            PyList_SetItem(pListInnX, i, PyFloat_FromDouble(innerCones.at(i).x));
            PyList_SetItem(pListInnY, i, PyFloat_FromDouble(innerCones.at(i).y));
        }

        for (int i = 0; i < outerCones.size(); i++)
        {
            PyList_SetItem(pListOutX, i, PyFloat_FromDouble(outerCones.at(i).x));
            PyList_SetItem(pListOutY, i, PyFloat_FromDouble(outerCones.at(i).y));
        }
        // for memory management
        vector<PyObject*> baseLines;

        for (int i = 0; i < base.size(); i++)
        {
            PyObject* pLineBaseI = PyList_New(4);
            PyList_SetItem(pLineBaseI, 0, PyFloat_FromDouble(base.at(i).at(0)));
            PyList_SetItem(pLineBaseI, 1, PyFloat_FromDouble(base.at(i).at(1)));
            PyList_SetItem(pLineBaseI, 2, PyFloat_FromDouble(base.at(i).at(2)));
            PyList_SetItem(pLineBaseI, 3, PyFloat_FromDouble(base.at(i).at(3)));
            baseLines.push_back(pLineBaseI);
            PyList_SetItem(pListBase, i, pLineBaseI);
        }
        
        auto pts = get_points(base, alphas);
        for (int i = 0; i < alphas.size(); i++)
        {
            PyList_SetItem(pListPtsX, i, PyFloat_FromDouble(pts.at(i).x));
            PyList_SetItem(pListPtsY, i, PyFloat_FromDouble(pts.at(i).y));
        }


        PyObject* pArgs = PyTuple_Pack(7, pListInnX, pListInnY, pListOutX, pListOutY, pListBase, pListPtsX, pListPtsY);
        PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
        Py_XDECREF(pArgs);
        Py_XDECREF(pListInnX);
        Py_XDECREF(pListInnY);
        Py_XDECREF(pListOutX);
        Py_XDECREF(pListOutY);
        Py_XDECREF(pListBase);
        Py_XDECREF(pListPtsX);
        Py_XDECREF(pListPtsY);
        for (auto line:baseLines)
            Py_XDECREF(line);
        if (pValue) {
            Py_XDECREF(pValue);
        }
    }
    Py_XDECREF(pModule);
}
vector<double> GeometricTrajectoryOptimizer::grad_l2(const vector<vector<double>>& base, const vector<double>& alphas)
{
    vector<double> grad(alphas.size(), 0.0);
    if(base.front() != base.back())
    {
        return grad;
        cerr << "Open track gradient calculation is not yet supported!" << endl;
    }
    auto pts = get_points(base, alphas);
    // add last element to the front and first element to the back
    pts.insert(pts.begin(), pts.back());
    pts.push_back(pts.at(1));
    for (size_t i = 1; i < alphas.size()+1; i++)
    {
        double x_im1 = pts.at(i).x - pts.at(i-1).x;
        double y_im1 = pts.at(i).y - pts.at(i-1).y;
        double x_i = pts.at(i+1).x - pts.at(i).x;
        double y_i = pts.at(i+1).y - pts.at(i).y;
        double x_bi0 = base.at(i-1).at(0); 
        double y_bi0 = base.at(i-1).at(1);
        double x_bi1 = base.at(i-1).at(2);
        double y_bi1 = base.at(i-1).at(3);
        grad.at(i-1) = 2*(x_bi1-x_bi0)*(x_im1-x_i) + 2*(y_bi1-y_bi0)*(y_im1-y_i);
    }
    return grad;
}
double GeometricTrajectoryOptimizer::sum(const std::vector<double> &vec)
{
    double output = 0;
    for (auto val:vec)
        if(!isnan(val))
            output += val;
    return output;
}
vector<double> GeometricTrajectoryOptimizer::grad_k2(const vector<vector<double>> &base, const vector<double> &alphas)
{
    vector<double> grad(alphas.size(), 0.0);
    if(base.front() != base.back())
    {
        return grad;
        cerr << "Open track gradient calculation is not yet supported!" << endl;
    }
    auto pts = get_points(base, alphas);
    auto angles = angle_profile(base, alphas);
    // add last element to the front and first element to the back
    angles.insert(angles.begin(), angles.back());
    angles.push_back(angles.at(1));
    pts.insert(pts.begin(), pts.back());
    pts.push_back(pts.at(1));
    pts.push_back(pts.at(2));
    for (size_t i = 1; i < alphas.size()+1; i++)
    {
        double x_im1 = pts.at(i).x - pts.at(i-1).x;
        double y_im1 = pts.at(i).y - pts.at(i-1).y;
        double x_i = pts.at(i+1).x - pts.at(i).x;
        double y_i = pts.at(i+1).y - pts.at(i).y;
        double x_ip1 = pts.at(i+2).x - pts.at(i+1).x;
        double y_ip1 = pts.at(i+2).y - pts.at(i+1).y;
        double A = -angles.at(i-1);
        double B = angles.at(i);
        double C = base.at(i-1).at(2) - base.at(i-1).at(0); // base is indexed with i - 1, because we did not append last element to the front
        double D = base.at(i-1).at(3) - base.at(i-1).at(1);
        double E = x_i*x_i + y_i*y_i;
        double F = x_im1*x_im1 + y_im1*y_im1;
        double G = angles.at(i+1);
        double dkim1_dai =  (2*y_im1*A - A*A*2*x_im1)/(F*F)*C +
                            (2*x_im1*(-A) - A*A*2*y_im1)*D/(F*F);
        double dki_dai = ((2*y_i*(-B)*(-C)/E + 2*y_im1*B*C/F + 2*x_i*B*(-D)/E+2*x_im1*(-B)*D/F)*E -
                            B*B*(2*x_i*(-C) +2*y_i*(-D)))/(E*E);
        double dkip1_dai = (2*y_i*G*(-C)/E + 2*x_i*(-G)*(-D)/E)/(x_ip1*x_ip1 + y_ip1*y_ip1);
        grad.at(i-1) = dkim1_dai + dki_dai + dkip1_dai;
    }
    return grad;
}
vector<double> GeometricTrajectoryOptimizer::grad_l(const vector<vector<double>>& base, const vector<double>& alphas)
{
    vector<double> grad(alphas.size(), 0.0);
    if(base.front() != base.back())
    {
        return grad;
        cerr << "Open track gradient calculation is not yet supported!" << endl;
    }
    auto pts = get_points(base, alphas);
    // add last element to the front and first element to the back
    pts.insert(pts.begin(), pts.back());
    pts.push_back(pts.at(1));
    for (size_t i = 1; i < alphas.size()+1; i++)
    {
        double x_im1 = pts.at(i).x - pts.at(i-1).x;
        double y_im1 = pts.at(i).y - pts.at(i-1).y;
        double x_i = pts.at(i+1).x - pts.at(i).x;
        double y_i = pts.at(i+1).y - pts.at(i).y;
        double x_bi0 = base.at(i-1).at(0); 
        double y_bi0 = base.at(i-1).at(1);
        double x_bi1 = base.at(i-1).at(2);
        double y_bi1 = base.at(i-1).at(3);
        grad.at(i-1) =  (x_bi1-x_bi0)*(x_im1/sqrt(x_im1*x_im1 + y_im1*y_im1) - x_i/sqrt(x_i*x_i + y_i*y_i)) + 
                        (y_bi1-y_bi0)*(y_im1/sqrt(x_im1*x_im1 + y_im1*y_im1) - y_i/sqrt(x_i*x_i + y_i*y_i));
    }
    return grad;
}
vector<double> GeometricTrajectoryOptimizer::grad_w_k2_l(const vector<vector<double>>& base, const vector<double>& alphas, double w)
{
    //w = 0 -> optimize based on length
    //w = 1 -> optimize based on k2 
    vector<double> grad(alphas.size(), 0.0);
    if(base.front() != base.back())
    {
        return grad;
        cerr << "Open track gradient calculation is not yet supported!" << endl;
    }
    auto g_k2 = grad_k2(base, alphas);
    auto g_l = grad_l(base, alphas);

    const vector<double> alpha_ref(alphas.size(),0.5); 
    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2_profile(base, alpha_ref));
    auto l_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance_profile(base, alpha_ref));

    for (size_t i = 0; i < alphas.size(); i++)
        grad.at(i) = w*g_k2.at(i)/k2_ref + (1-w)*g_l.at(i)/l_ref;
    return grad;    
}
vector<double> GeometricTrajectoryOptimizer::grad_w_k2_l2(const vector<vector<double>> &base, const vector<double> &alphas, double w)
{
    //w = 0 -> optimize based on length2
    //w = 1 -> optimize based on k2 
    vector<double> grad(alphas.size(), 0.0);
    if(base.front() != base.back())
    {
        return grad;
        cerr << "Open track gradient calculation is not yet supported!" << endl;
    }
    auto g_k2 = grad_k2(base, alphas);
    auto g_l2 = grad_l2(base, alphas);

    const vector<double> alpha_ref(alphas.size(),0.5); 
    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2_profile(base, alpha_ref));
    auto l2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance2_profile(base, alpha_ref));

    for (size_t i = 0; i < alphas.size(); i++)
        grad.at(i) = w*g_k2.at(i)/k2_ref + (1-w)*g_l2.at(i)/l2_ref;
    return grad; 
}

double GeometricTrajectoryOptimizer::k2_objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
    auto base = static_cast<vector<std::vector<double>>*>(f_data);
    auto k2_prof = GeometricTrajectoryOptimizer::curvature2_profile(*base, x);
    double output = 0;
    for (auto k2:k2_prof)
        if(!isnan(k2))
            output += k2;
    cout << "Objective function called! F_val: " << output << endl;
    return output;
}
double GeometricTrajectoryOptimizer::k2_grad_objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
    auto base = static_cast<vector<std::vector<double>>*>(f_data);
    auto k2_prof = GeometricTrajectoryOptimizer::curvature2_profile(*base, x);
    grad = GeometricTrajectoryOptimizer::grad_k2(*base, x);
    double output = 0;
    for (auto k2:k2_prof)
        if(!isnan(k2))
            output += k2;
    cout << "Objective function called! F_val: " << output << endl;
    return output;
}
double GeometricTrajectoryOptimizer::l2_objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
    auto base = static_cast<vector<std::vector<double>>*>(f_data);
    auto l2_prof = GeometricTrajectoryOptimizer::distance2_profile(*base, x);
    double output = 0;
    for (auto l2:l2_prof)
        if(!isnan(l2))
            output += l2;
    cout << "Objective function called! F_val: " << output << endl;
    return output;
}
double GeometricTrajectoryOptimizer::l2_grad_objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
    auto base = static_cast<vector<std::vector<double>>*>(f_data);
    auto l2_prof = GeometricTrajectoryOptimizer::distance2_profile(*base, x);
    grad = GeometricTrajectoryOptimizer::grad_l2(*base, x);
    double output = 0;
    for (auto l2:l2_prof)
        if(!isnan(l2))
            output += l2;
    cout << "Objective function called! F_val: " << output << endl;
    return output;
}
double GeometricTrajectoryOptimizer::l_objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
    auto base = static_cast<vector<std::vector<double>>*>(f_data);
    auto l_prof = GeometricTrajectoryOptimizer::distance_profile(*base, x);
    double output = 0;
    for (auto l:l_prof)
        if(!isnan(l))
            output += l;
    cout << "Objective function called! F_val: " << output << endl;
    return output;
}
double GeometricTrajectoryOptimizer::l_grad_objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
    auto base = static_cast<vector<std::vector<double>>*>(f_data);
    auto l_prof = GeometricTrajectoryOptimizer::distance_profile(*base, x);
    grad = GeometricTrajectoryOptimizer::grad_l(*base, x);
    double output = 0;
    for (auto l:l_prof)
        if(!isnan(l))
            output += l;
    cout << "Objective function called! F_val: " << output << endl;
    return output;
}
double GeometricTrajectoryOptimizer::w_k2_l_objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
    //w = 0 -> optimize based on length
    //w = 1 -> optimize based on k2 
    const double w = 0.5; //will preferably once become argument, not just hardcoded like this
    const vector<double> alpha_ref(x.size(),0.5); 
    auto base = static_cast<vector<std::vector<double>>*>(f_data);

    auto k2 = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2_profile(*base, x));
    auto l = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance_profile(*base, x));

    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2_profile(*base, alpha_ref));
    auto l_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance_profile(*base, alpha_ref));
    string skibidi = "toilet"; //prod by Jakub Maslen
    double output = w*k2/k2_ref + (1-w)*l/l_ref;
    cout << "Objective function called! F_val: " << output << endl;
    return output;
}
double GeometricTrajectoryOptimizer::w_k2_l_grad_objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
    //w = 0 -> optimize based on length
    //w = 1 -> optimize based on k2 
    const double w = 0.5; //will preferably once become argument, not just hardcoded like this
    const vector<double> alpha_ref(x.size(),0.5); 
    auto base = static_cast<vector<std::vector<double>>*>(f_data);
    grad = GeometricTrajectoryOptimizer::grad_w_k2_l(*base, x, w);

    auto k2 = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2_profile(*base, x));
    auto l = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance_profile(*base, x));

    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2_profile(*base, alpha_ref));
    auto l_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance_profile(*base, alpha_ref));
    string skibidi = "toilet"; //prod by Jakub Maslen
    double output = w*k2/k2_ref + (1-w)*l/l_ref;
    cout << "Objective function called! F_val: " << output << endl;
    return output;
}
double GeometricTrajectoryOptimizer::w_k2_l2_grad_objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
    //w = 0 -> optimize based on length
    //w = 1 -> optimize based on k2 
    const double w = 0.75; //will preferably once become argument, not just hardcoded like this
    const vector<double> alpha_ref(x.size(),0.5); 
    auto base = static_cast<vector<std::vector<double>>*>(f_data);
    grad = GeometricTrajectoryOptimizer::grad_w_k2_l2(*base, x, w);

    auto k2 = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2_profile(*base, x));
    auto l2 = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance2_profile(*base, x));

    auto k2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::curvature2_profile(*base, alpha_ref));
    auto l2_ref = GeometricTrajectoryOptimizer::sum(GeometricTrajectoryOptimizer::distance2_profile(*base, alpha_ref));
    double output = w*k2/k2_ref + (1-w)*l2/l2_ref;
    cout << "Objective function called! F_val: " << output << endl;
    return output;
}
vector<double> GeometricTrajectoryOptimizer::optimize(const vector<Point>& innerCones, const vector<Point>& outerCones, vector<vector<double>>& base)
{
    //https://nlopt.readthedocs.io/en/latest/NLopt_Reference/

    auto [inner, outer] = safety_margin(innerCones, outerCones, 1.5);
    // base = GeometricTrajectoryOptimizer::parametrize(inner,outer, 100);
    base = GeometricTrajectoryOptimizer::parametrize_gradual(inner,outer, 3, true);
    // base = GeometricTrajectoryOptimizer::parametrize(innerCones,outerCones, 100);

    // some setups have varying results based on initial guess, 1.0 seems to be better for w_k2_l_grad OF (f.e.)
    vector<double> alphas(base.size(), 1.0);

    base.push_back(base.at(0));
    // nlopt::opt opt(nlopt::LN_BOBYQA, alphas.size());
    nlopt::opt opt(nlopt::LD_SLSQP, alphas.size());

     //LN_BOBYQA - perfect for gradient-free optimization
     //LD_TNEWTON - perfect for l and l2, lacks in k2
     //LD_SLSQP - okay for l and l2, perfect for k2
     //LD_LBFGS - perfect for w
    opt.set_min_objective(w_k2_l_grad_objective_function, static_cast<void*>(&base));

    opt.set_lower_bounds(0.0);
    opt.set_upper_bounds(1.0);

    //"MaxFunctionEvaluations",10e3, "StepTolerance",1e-20
    opt.set_maxeval(40e3);
    // cout << "setting minimum tolerances: " << numeric_limits<double>::min() << endl;
    opt.set_xtol_rel(1e-9);
    opt.set_xtol_abs(1e-12);

    //much shorter than needed on purpose
    // opt.set_maxtime(0.2);

    // opt.set_ftol_abs(1e-24);
    // opt.set_ftol_rel(1e-18);

    double opt_k2 = 0;
    nlopt::result result = opt.optimize(alphas, opt_k2);
    if(result >=1 )
        cout << "Optimization successful!" << endl;
    else
        cout << "Optimization failed!" << endl;
    return alphas;
}

Point line_intersect(const Point& A1, const Point& A2, const Point& B1, const Point& B2) {
    double x1 = A1.x, y1 = A1.y, x2 = A2.x, y2 = A2.y;
    double x3 = B1.x, y3 = B1.y, x4 = B2.x, y4 = B2.y;
    
    double t_n = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4));
    double t_d = ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    if(t_d == 0)
    {
        // paralel or coincident lines, from the use case we know they must be coincident
        // so we implement use case specific behaviour -> p = (A2+B1)/2
        return {(A2.x + B1.x)/2, (A2.y + B1.y)/2};
    }
    double t = t_n/t_d;
    return {x1 + t * (x2 - x1), y1 + t * (y2 - y1)};
}

// Offset points function
std::vector<Point> offset_points(const Point& A, const Point& B, const Point& C, double offset, double dir) {
    Point V1 = B - A;
    Point V2 = C - B;
    
    Point N1 = {-dir * V1.y, dir * V1.x};
    Point N2 = {-dir * V2.y, dir * V2.x};
    
    N1 = N1.normalized() * offset;
    N2 = N2.normalized() * offset;
    
    return {A + N1, B + N1, B + N2, C + N2};
}

// Safety margin function for a single boundary
vector<Point> safety_margin_oneline(const vector<Point>& in, double margin, double direction) {
    size_t len = in.size();
    vector<Point> out;
    
    for (size_t i = 0; i < len; ++i) {
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
pair<vector<Point>,vector<Point>> GeometricTrajectoryOptimizer::safety_margin
    (const vector<Point>& inner, const vector<Point>& outer, double margin) 
{
    Point A = inner[1] - inner[0];
    Point B = outer[0] - inner[0];
    double dir = A.x * B.y - A.y * B.x;
    
    vector<Point> innerNew = safety_margin_oneline(inner, margin, dir);
    vector<Point> outerNew = safety_margin_oneline(outer, margin, -dir);
    return pair<vector<Point>,vector<Point>>(innerNew, outerNew);
}
