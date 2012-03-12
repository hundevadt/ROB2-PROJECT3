/*
 * PlannerTask.cpp
 *
 *  Created on: Mar 12, 2012
 *      Author: daniel
 */

#include "PlannerTask.hpp"

PlannerTask::PlannerTask(Ptr<Device> device, Ptr<QConstraint> constraint, Ptr<Q> qStart, Ptr<Q> qGoal) :
	_device(device),
	_constraint(constraint),
	_qStart(qStart),
	_qGoal(qGoal)
{

}

Ptr<Q> PlannerTask::getQStart()
{
	return _qStart;
}

Ptr<Q> PlannerTask::getQGoal()
{
	return _qGoal;
}

Ptr<QPath> PlannerTask::getPath()
{
	return _path;
}

void PlannerTask::setPath(Ptr<QPath> path)
{
	_path = path;
}

Ptr<Device> PlannerTask::getDevice()
{
	return _device;
}

Ptr<QConstraint> PlannerTask::getConstraint()
{
	return _constraint;
}

PlannerTask::~PlannerTask() {
	// TODO Auto-generated destructor stub
}