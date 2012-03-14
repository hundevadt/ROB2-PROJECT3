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
	Q					_qStart;
	Q	 				_qGoal;
	QPath				_path;


	QPath				_pathSteps;



public:
						PlannerTask(
								Ptr<Device> device,
								Ptr<QConstraint>,
								Q qStart,
								Q qGoal);

						PlannerTask(
								Ptr<Device> device,
								Ptr<QConstraint>,
								QPath pathSteps);


	void 				setPath(QPath path);
	QPath				getPath();

	Q					getQStart();
	Q	 				getQGoal();

	QPath				getPathSteps();


	Ptr<Device> 		getDevice();
	Ptr<QConstraint> 	getConstraint();

	virtual ~PlannerTask();
};

#endif /* PLANNERTASK_HPP_ */
