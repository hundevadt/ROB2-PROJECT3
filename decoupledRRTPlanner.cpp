/*
 * decoupledRRTPlanner.cpp
 *
 *  Created on: Mar 7, 2012
 *      Author: skytthe
 */

#include "decoupledRRTPlanner.h"
#include "PlannerTask.hpp"



#include <rw/rw.hpp>

//collision
#include <rw/proximity/BasicFilterStrategy.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rw/math/Math.hpp>
#include <rw/math/Q.hpp>

#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/models/Models.hpp>
#include <rw/trajectory/TimedUtil.hpp>

#include <cmath>

#include <RobWorkStudio.hpp>

#include <rw/rw.hpp>


#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

#include <rw/proximity/BasicFilterStrategy.hpp>


#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rw/math/Math.hpp>
#include <rw/math/Q.hpp>

#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/models/Models.hpp>
#include <rw/trajectory/TimedUtil.hpp>

decoupledRRTPlanner::decoupledRRTPlanner(rws::RobWorkStudio* robWorkStudio, rw::trajectory::QPath pathQsA, rw::trajectory::QPath pathQsB) {
	using namespace rws;


	_robWorkStudio = robWorkStudio;
	_pathQsA = pathQsA;
	_pathQsB = pathQsB;


	_workcell = _robWorkStudio->getWorkcell();
	_deviceA = _workcell->findDevice("KukaKr16A");
	_deviceB = _workcell->findDevice("KukaKr16B");

	_cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
	rw::proximity::ProximityFilterStrategy::Ptr filter = new rw::proximity::BasicFilterStrategy(_workcell);
	filter->addRule(rw::proximity::ProximitySetupRule::makeExclude("KukaKr16A.*","KukaKr16B.*"));
	_collisionDetector = new rw::proximity::CollisionDetector(_workcell, _cdstrategy,filter);
	_constraintA = rw::pathplanning::QConstraint::make(_collisionDetector, _deviceA, _workcell->getDefaultState());
	_constraintB = rw::pathplanning::QConstraint::make(_collisionDetector, _deviceB, _workcell->getDefaultState());
}

decoupledRRTPlanner::~decoupledRRTPlanner() {

}

rw::trajectory::QPath decoupledRRTPlanner::plan()
//rw::trajectory::QPath[] decoupledRRTPlanner::plan()
//void decoupledRRTPlanner::plan()
{
	rw::trajectory::QPath pathA;
	rw::trajectory::QPath pathB;

	rw::trajectory::QPath tempPath;

	Ptr<RRTPlanner> rrtplanner = new RRTPlanner();


    Ptr<PlannerTask> tempTask;

	//create Path(s) for Robot A
	for (unsigned int t = 0; t < _pathQsA.size(); t++) {
		std::cout <<t<< "org       " << _pathQsA.at(t) << std::endl;
	}

	for (int i = 0; i < (_pathQsA.size()-1); i++) {
		tempTask = new PlannerTask(_deviceA,_constraintA,_pathQsA.at(i),_pathQsA.at(i+1));

		std::list<Ptr<PlannerTask> > tasks;
		tasks.push_back(tempTask);
		rrtplanner->plan(tasks);
		tempPath = tempTask->getPath();

		for (unsigned int j = 0; j < tempPath.size()-1; j++) {
			pathA.push_back(tempPath.at((tempPath.size()-1)-j));
		}
	}
	pathA.push_back(_pathQsA.at(_pathQsA.size()-1));
	//print RRT path

	for (unsigned int t = 0; t < pathA.size(); t++) {
		std::cout <<t<< "path          " << pathA.at(t) << std::endl;
	}

	//create Path(s) for Robot B
	for (unsigned int t = 0; t < _pathQsB.size(); t++) {
		std::cout <<t<< "org       " << _pathQsB.at(t) << std::endl;
	}

	for (int i = 0; i < (_pathQsB.size()-1); i++) {
		tempTask = new PlannerTask(_deviceB,_constraintB,_pathQsB.at(i),_pathQsB.at(i+1));

		std::list<Ptr<PlannerTask> > tasks;
		tasks.push_back(tempTask);
		rrtplanner->plan(tasks);
		tempPath = tempTask->getPath();

		for (unsigned int j = 0; j < tempPath.size()-1; j++) {
			pathB.push_back(tempPath.at((tempPath.size()-1)-j));
		}
	}
	pathB.push_back(_pathQsB.at(_pathQsB.size()-1));

	//print RRT path
	for (unsigned int t = 0; t < pathB.size(); t++) {
		std::cout <<t<< "path          " << pathB.at(t) << std::endl;
	}

	//create path in the S-space
//	rw::trajectory::QPath sPath;

//	RRVTPlanner *RRVTplanner = new RRVTPlanner(_robWorkStudio,pathA,pathB);
//	sPath = RRVTplanner->plan();

	//print path in s-space
//	for (unsigned int t = 0; t < pathB.size(); t++) {
//		std::cout <<t<< "path s:        " << sPath.at(t) << std::endl;
//	}




	// return statementCalculating path length
	return pathB;
}























