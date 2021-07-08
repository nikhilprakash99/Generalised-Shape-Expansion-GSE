#include <iostream>
#include <fstream>
#include <string>
#include <boost/tokenizer.hpp>
#include <Eigen/Dense>

#include <omplapp/config.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/geometric/planners/GSE/GSE.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>

using namespace ompl;

base::PlannerPtr myConfiguredGSE(const base::SpaceInformationPtr &si);
void genSphere(Eigen::MatrixXd &sphere);

int main(int,char**){
    
    app::SE3RigidBodyPlanning setup;
    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/GSE/point.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/GSE/lattice.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-2);
    start->setY(-2);
    start->setZ(-2);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(10);
    goal->setY(10);
    goal->setZ(10);
    goal->rotation().setIdentity();

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 12.1);
    bounds.setHigh(1, 12.1);
    bounds.setHigh(2, 12.1);
    bounds.setLow(0, -2.1);
    bounds.setLow(1, -2.1);
    bounds.setLow(2, -2.1);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);
    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // run all planners with a uniform valid state sampler on the benchmark problem
    setup.getSpaceInformation()->setValidStateSamplerAllocator(
        [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
            return std::make_shared<base::UniformValidStateSampler>(si);
        });

    // set high because memory usage is not always estimated correctly
    double runtime_limit = 0.06, memory_limit = 100.0, run_count = 500;

    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, "GSE Benchmark");

    b.addPlannerAllocator(std::bind(&myConfiguredGSE, std::placeholders::_1));
    b.addPlanner(std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::BKPIECE1>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::LBKPIECE1>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::KPIECE1>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::SBL>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));


    b.benchmark(request);
    b.saveResultsToFile();

    return 0;
}

template<typename M>
M load_csv(const std::string path){
    std::ifstream fin(path.c_str());
    std::string line;
    std::vector<double> values;
    uint cols =0;
    while(getline(fin,line)){
        boost::tokenizer<boost::escaped_list_separator<char>> tok(line);
        for (auto str: tok)
            values.push_back(std::stod(str));
        ++cols;
    }
    return Eigen::Map<M>(values.data(),values.size()/cols,cols);
}

base::PlannerPtr myConfiguredGSE(const base::SpaceInformationPtr &si)
{
    geometric::GSE *planner = new geometric::GSE(si,3);

    Eigen::MatrixXd sphere(1000,3);
    genSphere(sphere);

    std::vector<std::vector<double>> obs{{0,0,0},
                                        {0,-2,0},
                                        {0,2,0},
                                        {-2,0,0},
                                        {2,0,0},
                                        {0,0,-2},
                                        {0,0,2},
                                        {1,1,1}};

    for(auto row:obs){
        Eigen::VectorXd centre = Eigen::Map<Eigen::VectorXd>(row.data(),row.size());
        Eigen::MatrixXd obsi = (0.5*sphere).colwise()+centre;
        planner->addObstacle(obsi);
    }

    si->setup();
    auto projector = si->getStateSpace()->getDefaultProjection();
    planner->setProjector(projector);
    
    return base::PlannerPtr(planner);
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

