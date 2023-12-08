#include <iostream>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include"raisim/World.hpp"
#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

class Robot
{
private:

public:
    raisim::World *Simulationworld;
    raisim::ArticulatedSystem *robot;
    raisim::Ground *SimGround;

    Robot(raisim::World *world, double dT, std::string urdfPath, std::string name){
        // initialize the world
        Simulationworld = world;
        Simulationworld->setTimeStep(dT);
        Simulationworld->setMaterialPairProp("rubber", "steel", 0.8, 0.8, 0.95);  //Material System,check urdfFile for more information
        // set the ground property
        SimGround = Simulationworld->addGround(0,"steel");  // Ground material
        // add robot
        robot = world->addArticulatedSystem(urdfPath);
        robot->setName(name);
    }

    virtual void initialize() = 0;  //robot initial state

};