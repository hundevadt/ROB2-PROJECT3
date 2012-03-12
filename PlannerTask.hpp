/*
 * PlannerTask.hpp
 *
 *  Created on: Mar 12, 2012
 *      Author: daniel
 */

#ifndef PLANNERTASK_HPP_
#define PLANNERTASK_HPP_

#include <rw/models/Device.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/math/Q.hpp>
#include <rw/trajectory/Path.hpp>

using namespace rw::common;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::trajectory;

class PlannerTask {
private:
	Ptr<Device> 		_device;
	Ptr<QConstraint> 	_constraint;
	Ptr<Q> 				_qStart;
	Ptr<Q> 				_qGoal;
	Ptr<QPath>			_path;

public:
						PlannerTask(
								Ptr<Device> device,
								Ptr<QConstraint>,
								Ptr<Q> qStart,
								Ptr<Q> qGoal);

	void 				setPath(Ptr<QPath> path);
	Ptr<QPath> 			getPath();

	Ptr<Q> 				getQStart();
	Ptr<Q> 				getQGoal();

	Ptr<Device> 		getDevice();
	Ptr<QConstraint> 	getConstraint();

	virtual ~PlannerTask();
};

#endif /* PLANNERTASK_HPP_ */
