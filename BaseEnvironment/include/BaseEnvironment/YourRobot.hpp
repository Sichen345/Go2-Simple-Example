#include <include/BaseEnvironment/Robot.hpp>

class YourRobot : public Robot
{
private:
    /* data */
    double angle = 0.01;
public:
    raisim::VecDyn initialJointPosition = raisim::VecDyn(19);   //robot->getGeneralizedCoordinateDim()
    raisim::VecDyn initialJointVelocity = raisim::VecDyn(18);  //robot->getDOF()
    Eigen::Matrix<double, 19, 1> jointNominalConfig;   //robot->getGeneralizedCoordinateDim()
    Eigen::Matrix<double, 18, 1> jointVelocityTarget;  //robot->getDOF()
    Eigen::Matrix<double, 18, 1> jointPgain;  //robot->getDOF()
    Eigen::Matrix<double, 18, 1> jointDgain;  //robot->getDOF()

    YourRobot(raisim::World *world, double dT, std::string urdfPath, std::string name) : Robot(world, dT, urdfPath, name){
        initialize();
    }
    void initialize() override;
    void changePosture();  // Change the angle of knees through PD controller
};
