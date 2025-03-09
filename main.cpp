#include <iostream>
#include "geometric_trajectory_optimizer.hpp"
#include <sstream>
#include <string>
#include <fstream>

using namespace std;
int main()
{
    // Read cone positions from file
    ifstream trackFile("track.txt");
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
    auto base = GeometricTrajectoryOptimizer::parametrize(innerCones,outerCones, 100);

    // Save parametrization in a file
    ofstream baseFile("base.txt");
    if(!baseFile.is_open())
    {
        cout << "Error opening \"base.txt\" file!" << endl;
        return 42;
    }    

    for(auto entry:base)
    {
        baseFile << entry.at(0) << " " << entry.at(1) << " " << entry.at(2) << " " << entry.at(3) << endl;
    }
    baseFile.close();

    vector<double> alphas(base.size(), 0.0);

    vector<Point> pts = GeometricTrajectoryOptimizer::get_points(base, alphas);

    ofstream trajFile("trajectory.txt");
    
    if(!trajFile.is_open())
    {
        return 42;
    }
    for (auto pt:pts)
        trajFile << pt.x << " " << pt.y << endl;
    
    trajFile.close();
    
    // unit test profile functions;
    auto s_prof = GeometricTrajectoryOptimizer::distance_profile(base, alphas);
    auto k_prof = GeometricTrajectoryOptimizer::curvature_profile(base, alphas);
    auto k2_prof = GeometricTrajectoryOptimizer::curvature2_profile(base, alphas);

    ofstream profFile("profiles.txt");
    if(!profFile.is_open())
    {
        cout <<  "Error opening file \"profiles.txt\", aborting..." << endl;
        return 42;
    }
    profFile << "Distance profile" << endl;
    for(double s:s_prof)
        profFile << s << endl;

    profFile << "Curvature profile" << endl;
    for(double k:k_prof)
        profFile << k << endl;

    profFile << "Curvature2 profile" << endl;
    for(double k2:k2_prof)
        profFile << k2 << endl;
    profFile.close();

    // GeometricTrajectoryOptimizer::plot_demo({1,2,3,4,5}, {1,4,9,16,25});
    // GeometricTrajectoryOptimizer::plot_all(innerCones,outerCones,base,alphas);
    alphas = GeometricTrajectoryOptimizer::optimize(innerCones, outerCones, base);
    GeometricTrajectoryOptimizer::plot_all(innerCones,outerCones,base,alphas);
    return 42;
    // Run optimization
    // Save optimization trajectory in a file
    // invoke plotting function
}