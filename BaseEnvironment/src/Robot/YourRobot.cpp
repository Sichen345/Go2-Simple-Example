#include <include/BaseEnvironment/YourRobot.hpp>

void YourRobot::initialize(){;
    initialJointPosition.setZero();

    // base position (x,y,z)
    initialJointPosition[0] = 0.0;
    initialJointPosition[1] = 0.0;
    initialJointPosition[2] = 0.2964;

    // base rotation (q1, q2, q3, q4)
    initialJointPosition[3] = 1.0;
    initialJointPosition[4] = 0.0;
    initialJointPosition[5] = 0.0;
    initialJointPosition[6] = 0.0;

    // FR: hip calf knee
    initialJointPosition[7] = 0.0;
    initialJointPosition[8] = 0.8683;
    initialJointPosition[9] = -1.5679;

    // FL: hip calf knee
    initialJointPosition[10] = -0.0;
    initialJointPosition[11] = 0.8683;
    initialJointPosition[12] = -1.5679;

    // RR: hip calf knee
    initialJointPosition[13] = 0.0;
    initialJointPosition[14] = 0.8683;
    initialJointPosition[15] = -1.5679;

    // RL: hip calf knee
    initialJointPosition[16] = -0.0;
    initialJointPosition[17] = 0.8683;
    initialJointPosition[18] = -1.5679;

    robot->setGeneralizedCoordinate(initialJointPosition);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
}

void YourRobot::changePosture(){
    double Dangle = sin(angle);
    angle += 0.01;
    if(angle >= 1.0 ){
        angle = 0.01;
    }
	jointNominalConfig << 0, 0, 0.3, 
						  1.0, 0.0, 0.0, 0.0, 
						  0.0, 0.8683, -1.5679-Dangle,
						  -0.0, 0.8683, -1.5679-Dangle, 
						  0.0, 0.8683, -1.5679,
						  -0.0, 0.8683, -1.5679;
  	jointVelocityTarget.setZero();
	Eigen::VectorXd jointPgain(18), jointDgain(18);  //robot->getDOF()
	jointPgain.tail(12).setConstant(60.0);
	jointDgain.tail(12).setConstant(5);

	robot->setPdGains(jointPgain, jointDgain);
  	robot->setPdTarget(jointNominalConfig, jointVelocityTarget);
}