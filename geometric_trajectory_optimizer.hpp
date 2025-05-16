#include <iostream>
#include <cmath>
#include <limits>
#include <Python.h>
#include <nlopt.hpp>
#include <bits/stdc++.h>

// using namespace std;
struct Point
{
    double x, y;
    double norm() const { return std::sqrt(x * x + y * y); }
    Point normalized() const { double n = norm(); return {x / n, y / n}; }
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
            public:
                double safetyMargin = 1.5;
                double parametrizationSpacing = 3;
                double klWeight = 0.5;
                nlopt::algorithm nlopt_algorithm = nlopt::LD_SLSQP;
                nlopt::vfunc objective_function = k2LGradObjectiveFunction;
                bool enableTimeLimit = false;
                double timeLimit = 0.2;
                double xtolRel = 1e-9; //5e-3 still works
                double xtolAbs = 1e-12; //1e-2 still works
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
        GeometricTrajectoryOptimizer::Config config_;
        GeometricTrajectoryOptimizer::VehicleModel vehicle_model_;
        
        std::vector<Point> innerCones_;
        std::vector<Point> outerCones_;
        std::vector<std::vector<double>> base_;
        std::vector<double> alphas_;
        std::vector<double> v_prof_;

        static double distance(const Point& a, const Point& b);
        static double linearInterp(const std::vector<double>& t, const std::vector<double>& v, double s);
        static bool crossesBetween(const Point& A, const Point& B, const Point& origin, const Point& direction, Point& intersection);
        static std::vector<std::vector<double>> parametrize(const std::vector<Point>& innerCones, const std::vector<Point>& outerCones, int resolution);
        static std::vector<std::vector<double>> parametrizeGradual(std::vector<Point> innerCones,std::vector<Point> outerCones, double ds,bool is_closed);
        static std::vector<double> angleProfile(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas);
        static std::vector<double> angleProfile(const std::vector<Point>& pts);
        static std::vector<double> distanceProfile(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas);
        static std::vector<double> distance2Profile(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas);
        static std::vector<double> curvatureProfile(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas);
        static std::vector<double> curvature2Profile(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas);

        // template <typename T> static T sumstd::vector(const std::vector<T>& vec); //overkill
        static double sum(const std::vector<double>& vec);

        static std::vector<double> gradK2(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas);
        static std::vector<double> gradL(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas);
        static std::vector<double> gradL2(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas);
        static std::vector<double> gradK2L(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas, double w);
        static std::vector<double> gradK2L2(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas, double w);


        static std::vector<Point> getPoints(const std::vector<std::vector<double>>& base, const std::vector<double>& alphas);
        static double getAngle(const Point& A, const Point& B, const Point& C);
        static void plotDemo(const std::vector<double>& x, const std::vector<double>& y);
        static void plotAll(
            const std::vector<Point>& innerCones, 
            const std::vector<Point>& outerCones,
            const std::vector<std::vector<double>>& base,
            const std::vector<double>& alphas
        );
        static std::vector<double> optimize(const std::vector<Point>& innerCones, const std::vector<Point>& outerCones, std::vector<std::vector<double>>& base);
        static std::pair<std::vector<Point>,std::vector<Point>> safetyMargin(const std::vector<Point>& inner, const std::vector<Point>& outer, double margin) ;
    public: 
        bool update(std::vector<Point> innerCones,std::vector<Point> outerCones);
        GeometricTrajectoryOptimizer::Config& getConfig();
        GeometricTrajectoryOptimizer::VehicleModel& getVehicleModel();
        void setConfig(Config config);
        void setVehicleModel(VehicleModel vehicleModel);
        std::vector<Point> getPath();
        std::vector<double> getRefSpeed(Point pose, double v0);
        void plotAll();

        //objective functions made accessible for config_
        static double k2ObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
        static double k2GradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
        static double l2ObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
        static double l2GradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
        static double lObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
        static double lGradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
        static double k2LObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
        static double k2LGradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
        static double k2L2GradObjectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
};