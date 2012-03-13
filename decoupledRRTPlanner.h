/*
 * decoupledRRTPlanner.h
 *
 *  Created on: Mar 7, 2012
 *      Author: skytthe
 */

#ifndef DECOUPLEDRRTPLANNER_H_
#define DECOUPLEDRRTPLANNER_H_

#include <rws/RobWorkStudio.hpp>
#include <rw/rw.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

#include "RRTPlanner.h"

class decoupledRRTPlanner {
private:
	rws::RobWorkStudio* _robWorkStudio;
	rw::trajectory::QPath _pathQsA;
	rw::trajectory::QPath _pathQsB;

	rw::common::Ptr<rw::models::WorkCell> _workcell;
	rw::common::Ptr<rw::models::Device> _deviceA;
	rw::common::Ptr<rw::models::Device> _deviceB;
	rw::common::Ptr<rw::proximity::CollisionStrategy> _cdstrategy;
	rw::common::Ptr<rw::proximity::CollisionDetector> _collisionDetector;
	rw::common::Ptr<rw::pathplanning::QConstraint> _constraintA;
	rw::common::Ptr<rw::pathplanning::QConstraint> _constraintB;

public:
	decoupledRRTPlanner(rws::RobWorkStudio* robWorkStudio, rw::trajectory::QPath pathQsA, rw::trajectory::QPath pathQsB);
	virtual ~decoupledRRTPlanner();

	rw::trajectory::QPath plan();
//	rw::trajectory::QPath[] plan();
//	void plan();
};

#endif /* DECOUPLEDRRTPLANNER_H_ */
