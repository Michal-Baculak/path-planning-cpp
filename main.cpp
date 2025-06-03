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
    string track_name;
    std::ifstream user_input_file("misc_input.txt");
    user_input_file >> track_name;
    user_input_file.close();
    ifstream trackFile(track_name);
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
    
    std::ofstream exp_out("LogFolder/experiment.txt");
    if(!exp_out.is_open())
    {
        std::cout << "error opening exp_out file!\n";
        return 4224;
    }
    for (size_t i = 0; i <= 20; i++)
    {
        /* code */
        GeometricTrajectoryOptimizer opt{};
        opt.getConfig().objective_function = GeometricTrajectoryOptimizer::k2L2GradObjectiveFunction; //GeometricTrajectoryOptimizer::stringToObjectiveFunction("obj_fun"); //GeometricTrajectoryOptimizer::k2LGradObjectiveFunction;
        opt.getConfig().nlopt_algorithm = nlopt::LD_SLSQP; //GeometricTrajectoryOptimizer::intToNLOPTAlgorithm(40); //LD_SLSQP = 40, LN_BOBYQA = 34
        opt.getConfig().enable_time_limit = false; //0.2s default
        opt.getConfig().time_limit = 0.5; //0.2s default
        opt.getConfig().parametrization_spacing = 3; 
        opt.getConfig().x_tol_rel = 0.000000001; 
        opt.getConfig().x_tol_abs = 0.000000000001; 
        opt.getConfig().safety_margin = 1.3; 
        opt.getConfig().k_l_weight =  0+0.05*i;
    
        opt.getVehicleModel().a_lat_max = 12.6;
        opt.getVehicleModel().a_front_max = 17; 
        opt.getVehicleModel().a_max_brake = 21.8; 
        opt.getVehicleModel().max_power = 80000;
        opt.getVehicleModel().v_max = 33.3;
        opt.getVehicleModel().mass = 190;
        opt.getVehicleModel().c_steering = 30;
    
        // opt.getConfig().objective_function = GeometricTrajectoryOptimizer::k2GradObjectiveFunction;
        auto start = std::chrono::high_resolution_clock::now(); 
        bool result = false;
        while(!result)
        {
            // cout << "running " << iteration++ << ". iteration\n"; 
            result = opt.updateTrajectory(innerCones, outerCones);
        }
        // alphas = GeometricTrajectoryOptimizer::optimize(innerCones, outerCones, base);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        cout << "Optimization time in us: " << duration.count() << "\n";
        opt.calcOptimalRefSpeed();
        cout << "Optimal lap time in seconds: " << opt.getLapTimeEst() << " for k-l weight " << opt.getConfig().k_l_weight << "\n";
        exp_out << opt.getLapTimeEst() << "\n";
        opt.logAlphas("LogFolder/alphas" + std::to_string(i) + ".txt");
        opt.plotAll();
    }
    
    
    return 42;

}