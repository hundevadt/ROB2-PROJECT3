/*
 * RRTPlanner.h
 *
 *  Created on: Mar 8, 2012
 *      Author: skytthe
 */

#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include <rw/models/Device.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/trajectory/Path.hpp>
#include "RRTNode.h"


#include "PlannerTask.hpp"

using namespace rw::common;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::trajectory;

class RRTPlanner
{

public:
	RRTPlanner();
	bool edgeCollisionDetection(Ptr<RRTNode> nodeClose, Ptr<RRTNode> nodeNew, Ptr<PlannerTask> task);
	void plan(std::list<Ptr<PlannerTask> > tasks);
	virtual ~RRTPlanner();
};

#endif /* RRTPLANNER_H_ */
