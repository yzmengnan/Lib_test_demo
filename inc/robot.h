//
// Created by 91418 on 2024/1/7.
//

#ifndef ROBOT_H
#define ROBOT_H
#include <iostream>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>

#include <string>
#include <thread>
#include <vector>
class Robot
{
public:
	/*!
	 * load the model of the robot
	 * @param filepath
	 */
	explicit Robot(const std::string& filepath);
	bool start(std::shared_ptr<std::vector<double>> j);
	~Robot() = default;
	/*!
	 * @param jointData joint space positions at degrees
	 * @return position with respect to the world frame and the rpy angles
	 */
	std::vector<double> fkine(const std::vector<double>& jointData);
	/**
	 * @return  position with respect to the world frame and the rpy angles
	 * @details only effective with the member function start pre-run
	 */
	std::vector<double> fkine();
	/*!
	 * @param jointData
	 * @return return final T
	 */
	rl::math::Transform getTransformMatrix(const std::vector<double>& jointData);
	/*!
	 *
	 * @return
	 */
	[[maybe_unused]] rl::math::Transform getTransformMatrix();
	/*!
	 * @param jointData joint space positions at degrees
	 * @return jacobian matrix with the endeffcetor frame
	 */
	rl::math::Matrix getJacobe(const std::vector<double>& jointData);
	rl::math::Matrix getInverseJacobe(const std::vector<double>& jointData);
	/*!
	 * @param jointData joint space positions at degrees
	 * @return jacobian matrix with the endeffcetor frame
	 */
	rl::math::Matrix getJacobe();
	/*!
	 * @param jointData joint space positions at degrees
	 * @return  jacobian matrix with the world frame
	 */
	rl::math::Matrix getJacob0(const std::vector<double>& jointData);
	rl::math::Matrix getInverseJacob0(const std::vector<double>& jointData);
	/*!
	 * @param jointData joint space positions at degrees
	 * @return  jacobian matrix with the world frame
	 */
	rl::math::Matrix getJacob0();

private:
	rl::mdl::XmlFactory factory;
	std::shared_ptr<rl::mdl::Model> robotModel {};
	rl::mdl::Kinematic* kinematics;
};

#endif// ROBOT_H
