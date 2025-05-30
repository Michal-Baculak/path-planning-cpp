#include "geometric_trajectory_optimizer.hpp"
#include <string>
#include <chrono>

using namespace std;
using global_planning::GeometricTrajectoryOptimizer;
using global_planning::Point;
int main()
{
    std::cout << "__cplusplus: " << __cplusplus << std::endl;
    
    if (__cplusplus == 199711L) std::cout << "C++98/03" << std::endl;
    else if (__cplusplus == 201103L) std::cout << "C++11" << std::endl;
    else if (__cplusplus == 201402L) std::cout << "C++14" << std::endl;
    else if (__cplusplus == 201703L) std::cout << "C++17" << std::endl;
    else if (__cplusplus == 202002L) std::cout << "C++20" << std::endl;
    else if (__cplusplus > 202002L) std::cout << "C++23 or newer" << std::endl;
    else std::cout << "Unknown C++ version" << std::endl;
    
    // Read cone positions from filea
    ifstream trackFile("track_FSI.txt");
    string line;
    float x, y;
    vector<Point> innerCones, outerCones;
    
    // process innerCones (until '---' is found in file)
    while(getline(trackFile, line))
    {
        if(line == "---")
            break;
        std::istringstream iss(line);
        iss >> x >> y;
        innerCones.push_back({x,y});
    }
    // process outerCones
    while(getline(trackFile, line))
    {
        std::istringstream iss(line);
        iss >> x >> y;
        outerCones.push_back({x,y});
    }
    trackFile.close();
    
    // Parametrize track
    // auto base = GeometricTrajectoryOptimizer::parametrize(innerCones,outerCones, 100);
    // auto base = GeometricTrajectoryOptimizer::parametrizeGradual(innerCones,outerCones, 2, true);

    // // Save parametrization in a file
    // ofstream baseFile("base.txt");
    // if(!baseFile.is_open())
    // {
    //     cout << "Error opening \"base.txt\" file!" << endl;
    //     return 42;
    // }    

    // for(auto entry:base)
    // {
    //     baseFile << entry.at(0) << " " << entry.at(1) << " " << entry.at(2) << " " << entry.at(3) << endl;
    // }
    // baseFile.close();

    // vector<double> alphas(base.size(), 0.0);

    // vector<Point> pts = GeometricTrajectoryOptimizer::getPoints(base, alphas);
    // auto pts = opt.getTrajectory();

    // ofstream trajFile("trajectory.txt");
    
    // if(!trajFile.is_open())
    // {
    //     return 42;
    // }
    // for (auto pt:pts)
    //     trajFile << pt.x << " " << pt.y << endl;
    
    // trajFile.close();
    
    // unit test profile functions;
    // auto s_prof = GeometricTrajectoryOptimizer::distanceProfile(base, alphas);
    // auto k_prof = GeometricTrajectoryOptimizer::curvatureProfile(base, alphas);
    // auto k2_prof = GeometricTrajectoryOptimizer::curvature2Profile(base, alphas);

    // ofstream profFile("profiles.txt");
    // if(!profFile.is_open())
    // {
    //     cout <<  "Error opening file \"profiles.txt\", aborting..." << endl;
    //     return 42;
    // }
    // profFile << "Distance profile" << endl;
    // for(double s:s_prof)
    //     profFile << s << endl;

    // profFile << "Curvature profile" << endl;
    // for(double k:k_prof)
    //     profFile << k << endl;

    // profFile << "Curvature2 profile" << endl;
    // for(double k2:k2_prof)
    //     profFile << k2 << endl;
    // profFile.close();

    // GeometricTrajectoryOptimizer::plotDemo({1,2,3,4,5}, {1,4,9,16,25});
    // GeometricTrajectoryOptimizer::plotAll(innerCones,outerCones,base,alphas);
    // base.push_back(base.at(0));
    // auto grad_w = GeometricTrajectoryOptimizer::gradK2L(base, alphas, 0.5);
    GeometricTrajectoryOptimizer opt{};
    opt.getConfig().objective_function = GeometricTrajectoryOptimizer::stringToObjectiveFunction("k2LGrad"); //GeometricTrajectoryOptimizer::k2LGradObjectiveFunction;
    opt.getConfig().nlopt_algorithm = GeometricTrajectoryOptimizer::intToNLOPTAlgorithm(40);
    opt.getConfig().enable_time_limit = true; //0.2s default
    opt.getConfig().time_limit = 0.5; //0.2s default
    opt.getConfig().parametrization_spacing = 3; 
    opt.getConfig().x_tol_rel = 5e-3; 
    opt.getConfig().x_tol_abs = 1e-12; 
    opt.getConfig().safety_margin = 1; 
    opt.getVehicleModel().a_lat_max = 10;
    opt.getVehicleModel().a_front_max = 10; 
    opt.getVehicleModel().a_max_brake = 10; 
    opt.getVehicleModel().max_power = 70000;
    opt.getVehicleModel().v_max = 15;
    opt.getVehicleModel().mass = 270;
    opt.getVehicleModel().c_steering = 1;

    // opt.getConfig().objective_function = GeometricTrajectoryOptimizer::k2GradObjectiveFunction;
    auto start = std::chrono::high_resolution_clock::now(); 
    bool result = false;
    int iteration = 0;
    while(!result)
    {
        cout << "running " << iteration++ << ". iteration\n"; 
        result = opt.updateTrajectory(innerCones, outerCones);
    }
    // alphas = GeometricTrajectoryOptimizer::optimize(innerCones, outerCones, base);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    cout << "Optimization time in us: " << duration.count() << "us\n";
    // test points: {8,46}, {47,31}, {-10, 22}, {40, -30}
    opt.updateRefSpeed({0,0}, 0); // initializing call 
    auto v_prof = opt.getRefSpeed();
    std::vector<double> v_prof2;
    v_prof2 = GeometricTrajectoryOptimizer::updateRefSpeed({0,0},0,opt.getVehicleModel(),opt.getTrajectory(), v_prof2);
    opt.updateRefSpeed({0,0}, 0); // updating call 
    v_prof = opt.getRefSpeed();

    // opt.updateRefSpeed({10, -5}, 24); // updating call - current velocity is irrelevant 
    // v_prof = opt.getRefSpeed();
    auto pts = opt.getTrajectory();
    for (size_t i = 21; i < pts.size(); i++)
    {
        opt.updateRefSpeed(pts.at(i), 4);
        v_prof = opt.getRefSpeed();
    }
    opt.plotAll();
    opt.logAll("LogFolder/");
    // ofstream alphasFile("alphas.txt");
    // if(!alphasFile.is_open())
    // {
    //     cout << "Failed to open the \"alphas.txt\" file!\n";
    //     return 42;
    // }
    // for(auto alpha:alphas)
    //     alphasFile << alpha << ";\n";
    // alphasFile.close();
    return 42;

    // Run optimization
    // Save optimization trajectory in a file
    // invoke plotting function
}