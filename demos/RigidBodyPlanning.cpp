#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/base/SpaceInformation.h"
#include <ompl/geometric/planners/GSE/GSE.h>
#include "ompl/base/spaces/RealVectorStateProjections.h"

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void genSphere(Eigen::MatrixXd &sphere);
bool isStateValid(const ob::State *state);

int main()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));
    
    // set the bounds for the R^3 part of SE(3)
    space->setBounds(-2.1,2.1);
    
    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    
    // create a start state and goal state
    ob::ScopedState<> start(space),goal(space);
    start = {-2.0,-2.0,-2.0};
    goal  = {2.0,2.0,2.0};
    
    // create a problem instance, set the start and goal states
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);

    // create a GSE planner for the defined space
    auto planner(std::make_shared<og::GSE>(si,3));
    planner->setProblemDefinition(pdef);

    // Add obstacles to planner for planning
    Eigen::MatrixXd sphere(1000,3);
    genSphere(sphere); // generate a sphere obstacle
    vector<vector<double>> obs{ {0,0,0},
                            {0,-2,0},
                            {0,2,0},
                            {-2,0,0},
                            {2,0,0},
                            {0,0,-2},
                            {0,0,2},
                            {1,1,1}}; // Locations where we want to place the spherical obstacle
    for(auto row:obs){
        Eigen::VectorXd centre = Eigen::Map<Eigen::VectorXd>(row.data(),row.size());
        Eigen::MatrixXd obsi = (0.5*sphere.transpose()).colwise()+centre;
        planner->addObstacle(obsi);
    }

    // Set the Projector to extract an Eigen::Vector state from base::state 
    std::vector<uint> projectionDimensions{0,1,2};
    auto projector(std::make_shared<ob::RealVectorOrthogonalProjectionEvaluator>(space,projectionDimensions));
    planner->setProjector(projector);
    
    // Setup the planner
    planner->setup();

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(1);

    if (solved)
    {
        ob::PathPtr path = pdef->getSolutionPath();
        cout << "Found solution:" << std::endl;

        // print the path to screen
        path->as<og::PathGeometric>()->printAsMatrix(cout);
    }
    else
        cout << "No solution found" << std::endl;

    return 0;
}

// generate Uniform randomn Points on a sphere of raidius 1
// http://corysimon.github.io/articles/uniformdistn-on-sphere/ 
void genSphere(Eigen::MatrixXd &sphere){
    int N = sphere.rows();
    auto theta = 2*M_PI*(Eigen::ArrayXd::Random(N)*0.5+0.5);
    auto phi = Eigen::ArrayXd::Random(N).acos();

    Eigen::VectorXd x = phi.sin()*theta.cos();
    Eigen::VectorXd y = phi.sin()*theta.sin();
    Eigen::VectorXd z = phi.cos();

    sphere<<x,y,z;
}

bool isStateValid(const ob::State *state)
{
    // return a value that is always true but uses the argument, so we avoid compiler warnings
    return state;
}