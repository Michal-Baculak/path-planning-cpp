#include "geometric_trajectory_optimizer.hpp"

using namespace std;
Point operator-(const Point& a, const Point& b) 
{
    return {a.x - b.x, a.y - b.y};
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
        output.at(i) = get_angle(pts.at(i-1), pts.at(i), pts.at(i+1));
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
vector<Point> GeometricTrajectoryOptimizer::get_points(const vector<vector<double>>& base, const vector<double>& alphas)
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
double k2_objective_function(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
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
vector<double> GeometricTrajectoryOptimizer::optimize(const vector<Point>& innerCones, const vector<Point>& outerCones, vector<vector<double>>& base)
{
    //https://nlopt.readthedocs.io/en/latest/NLopt_Reference/
    base = GeometricTrajectoryOptimizer::parametrize(innerCones,outerCones, 100);
    // base.push_back(base.at(0));
    vector<double> alphas(base.size(), 0.0);
    nlopt::opt opt(nlopt::LN_BOBYQA, alphas.size());
    opt.set_min_objective(k2_objective_function, static_cast<void*>(&base));

    opt.set_lower_bounds(0.0);
    opt.set_upper_bounds(1.0);

    //"MaxFunctionEvaluations",10e3, "StepTolerance",1e-20
    opt.set_maxeval(40e3);
    // cout << "setting minimum tolerances: " << numeric_limits<double>::min() << endl;
    opt.set_xtol_rel(1e-9);
    opt.set_xtol_abs(1e-12);

    double opt_k2 = 0;
    nlopt::result result = opt.optimize(alphas, opt_k2);
    if(result >=1 )
        cout << "Optimization successful!" << endl;
    else
        cout << "Optimization failed!" << endl;
    return alphas;
}
