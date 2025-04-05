#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <Python.h>
#include <nlopt.hpp>
#include <bits/stdc++.h>

using namespace std;
struct Point
{
    double x, y;
    double norm() const { return std::sqrt(x * x + y * y); }
    Point normalized() const { double n = norm(); return {x / n, y / n}; }
};
class GeometricTrajectoryOptimizer
{
    public: 
        static vector<double> velocityProfile(vector<Point> trajectory);
        static double distance(const Point& a, const Point& b);
        static bool crossesBetween(const Point& A, const Point& B, const Point& origin, const Point& direction, Point& intersection);
        static std::vector<std::vector<double>> parametrize(const std::vector<Point>& innerCones, const std::vector<Point>& outerCones, int resolution);
        static std::vector<std::vector<double>> parametrize_gradual(std::vector<Point> innerCones,std::vector<Point> outerCones,double ds,bool is_closed);
        static vector<double> angle_profile(const vector<vector<double>>& base, const vector<double>& alphas);
        static vector<double> angle_profile(const vector<Point>& pts);
        static vector<double> distance_profile(const vector<vector<double>>& base, const vector<double>& alphas);
        static vector<double> distance2_profile(const vector<vector<double>>& base, const vector<double>& alphas);
        static vector<double> curvature_profile(const vector<vector<double>>& base, const vector<double>& alphas);
        static vector<double> curvature2_profile(const vector<vector<double>>& base, const vector<double>& alphas);

        // template <typename T> static T sumVector(const std::vector<T>& vec); //overkill
        static double sum(const std::vector<double>& vec);

        static vector<double> grad_k2(const vector<vector<double>>& base, const vector<double>& alphas);
        static vector<double> grad_l(const vector<vector<double>>& base, const vector<double>& alphas);
        static vector<double> grad_l2(const vector<vector<double>>& base, const vector<double>& alphas);
        static vector<double> grad_w_k2_l(const vector<vector<double>>& base, const vector<double>& alphas, double w);
        static vector<double> grad_w_k2_l2(const vector<vector<double>>& base, const vector<double>& alphas, double w);


        static vector<Point> get_points(const vector<vector<double>>& base, const vector<double>& alphas);
        static double get_angle(const Point& A, const Point& B, const Point& C);
        static void plot_demo(const vector<double>& x, const vector<double>& y);
        static void plot_all(
            const vector<Point>& innerCones, 
            const vector<Point>& outerCones,
            const vector<vector<double>>& base,
            const vector<double>& alphas
        );
        static vector<double> optimize(const vector<Point>& innerCones, const vector<Point>& outerCones, vector<vector<double>>& base);
        static pair<vector<Point>,vector<Point>> safety_margin(const vector<Point>& inner, const vector<Point>& outer, double margin) ;

};