//
// Created by 91418 on 2024/1/7.
//

#include "robot.h"
Robot::Robot(const std::string& filepath)
{
	this->robotModel = std ::shared_ptr<rl::mdl::Model>(this->factory.create(filepath));
}
auto Robot::fkine(const std::vector<double>& jointData) -> std::vector<double>
{
	std::vector<double> res(6, 0);
	// input data invalid
	if (jointData.size() != 6)
	{
		return res;
	}
	rl::math::Vector q(6);
	// set RL vector for joints` position
	q << jointData[0], jointData[1], jointData[2], jointData[3], jointData[4], jointData[5];
	q *= rl::math::constants::deg2rad;
	auto *tempK = dynamic_cast<rl::mdl::Kinematic*>(this->robotModel.get());
	tempK->setPosition(q);
	tempK->forwardPosition();
	rl::math::Transform t         = tempK->getOperationalPosition(0);
	rl::math::Vector3 position    = t.translation();
	rl::math::Vector3 orientation = t.rotation().eulerAngles(2, 1, 0).reverse();
	orientation *= rl::math::constants ::rad2deg;
	res = {position[0], position[1], position[2], orientation[0], orientation[1], orientation[2]};
	return res;
}
rl::math::Transform Robot::getTransformMatrix(const std::vector<double>& jointData)
{
	if (jointData.size() != 6) {
		return rl::math::Transform();
}
	rl::math::Vector temp_q(6);
	temp_q << jointData[0], jointData[1], jointData[2], jointData[3], jointData[4], jointData[5];
	temp_q *= rl::math::constants::deg2rad;
	auto *tempK = dynamic_cast<rl::mdl::Kinematic*>(this->robotModel.get());
	tempK->setPosition(temp_q);
	tempK->forwardPosition();
	return tempK->getOperationalPosition(0);
}
rl::math::Matrix Robot::getJacobe(const std::vector<double>& jointData)
{
	if (jointData.size() != 6) {
		return rl::math::Matrix();
}
	rl::math::Vector temp_q(6);
	temp_q << jointData[0], jointData[1], jointData[2], jointData[3], jointData[4], jointData[5];
	temp_q *= rl::math::constants::deg2rad;
	auto tempK = dynamic_cast<rl::mdl::Kinematic*>(this->robotModel.get());
	tempK->setPosition(temp_q);
	tempK->calculateJacobian(false);
	return tempK->getJacobian();
}
auto Robot::getJacob0(const std::vector<double>& jointData) -> rl::math::Matrix
{
	if (jointData.size() != 6) {
		return rl::math::Matrix();
}
	rl::math::Vector temp_q(6);
	temp_q << jointData[0], jointData[1], jointData[2], jointData[3], jointData[4], jointData[5];
	temp_q *= rl::math::constants::deg2rad;
	auto tempK = dynamic_cast<rl::mdl::Kinematic*>(this->robotModel.get());
	tempK->setPosition(temp_q);
	tempK->calculateJacobian();
	return tempK->getJacobian();
}
auto Robot::fkine() -> std::vector<double>
{
	this->kinematics->forwardPosition();
	auto t                        = this->kinematics->getOperationalPosition(0);
	rl::math::Vector3 position    = t.translation();
	rl::math::Vector3 orientation = t.rotation().eulerAngles(2, 1, 0);
	orientation *= rl::math::constants ::rad2deg;
	return {position[0], position[1], position[2], orientation[0], orientation[1], orientation[2]};
}
auto Robot::start(const std::shared_ptr<std::vector<double>> j) -> bool
{
	if ((*j).size() != 6)
		return false;
	kinematics = dynamic_cast<rl::mdl::Kinematic*>(this->robotModel.get());
	auto f     = [&]() {
        while (true)
        {
            rl::math::Vector q(6);
            q << (*j)[0], (*j)[1], (*j)[2], (*j)[3], (*j)[4], (*j)[5];
            q *= rl::math::constants ::deg2rad;
            this->kinematics->setPosition(q);
        }
	};
	try
	{
		std::thread t(f);
		t.detach();
	}
	catch (std::exception)
	{
		return false;
	}
	return true;
}
rl::math::Matrix Robot::getJacobe()
{
	this->kinematics->calculateJacobian(false);
	return this->kinematics->getJacobian();
}
rl::math::Matrix Robot::getJacob0()
{
	this->kinematics->calculateJacobian();
	return this->kinematics->getJacobian();
}
auto Robot::getInverseJacobe(const std::vector<double>& jointData) -> rl::math::Matrix
{
	if (jointData.size() != 6) {
		return {};
}
	rl::math::Vector temp_q(6);
	temp_q << jointData[0], jointData[1], jointData[2], jointData[3], jointData[4], jointData[5];
	temp_q *= rl::math::constants::deg2rad;
	auto *tempK = dynamic_cast<rl::mdl::Kinematic*>(this->robotModel.get());
	tempK->setPosition(temp_q);
	tempK->forwardPosition();
	tempK->calculateJacobian(false);
	// TODO do not use svd
	tempK->calculateJacobianInverse(0.5F, false);
	return tempK->getJacobianInverse();
}
auto Robot::getInverseJacob0(const std::vector<double>& jointData) -> rl::math::Matrix
{
	if (jointData.size() != 6)
	{
		return {};
	}
	rl::math::Vector temp_q(6);
	temp_q << jointData[0], jointData[1], jointData[2], jointData[3], jointData[4], jointData[5];
	temp_q *= rl::math::constants::deg2rad;
	auto* tempK = dynamic_cast<rl::mdl::Kinematic*>(this->robotModel.get());
	tempK->setPosition(temp_q);
	tempK->forwardPosition();
	tempK->calculateJacobian(true);
	tempK->calculateJacobianInverse(0.5F, false);
	return tempK->getJacobianInverse();
}
