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

	rw::trajectory::QPath _pathA,_pathB;

	rw::common::Ptr<rw::models::WorkCell> _workcell;
	rw::common::Ptr<rw::models::Device> _deviceA;
	rw::common::Ptr<rw::models::Device> _deviceB;

	rw::proximity::CollisionDetector::Ptr _detector;

	double _norm2A,_norm2B;
	std::vector<double> _norm2posListA, _norm2posListB;


public:
	decoupledRRTPlanner(rws::RobWorkStudio* robWorkStudio);
	virtual ~decoupledRRTPlanner();

	rw::math::Q randQ();

	std::vector<std::vector<bool> > planSspaceMap(std::list<Ptr<PlannerTask> > tasks);
	bool inCollision(rw::math::Q s,rw::trajectory::QPath pathA, rw::trajectory::QPath pathB);
	bool edgeCollisionDetection(rw::common::Ptr<RRTNode> nodeClose, rw::common::Ptr<RRTNode> nodeNew);


	void plan(std::list<Ptr<PlannerTask> > tasks);
//	rw::trajectory::QPath[] plan();
//	void plan();
};

#endif /* DECOUPLEDRRTPLANNER_H_ */
