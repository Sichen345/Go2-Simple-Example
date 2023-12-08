#include <include/BaseEnvironment/YourRobot.hpp>

// check the world time by:  robot.Simulationworld->getWorldTime();

int main(){
    std::string urdfPath = "\\home\\Usr-name\\workspace\\src\\BaseEnvironment\\rsc\\go2\\urdf\\go2_description.urdf";
    std::string robotName = "go2";
    double dT = 0.005;
    raisim::World world;
    YourRobot robot = YourRobot(&world, dT, urdfPath, robotName);
    raisim::RaisimServer server(&world);
    server.focusOn(robot.SimGround);
    server.launchServer();

    while(true){
        robot.changePosture();
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        server.integrateWorldThreadSafe();  // update the world and time safely
    }
}