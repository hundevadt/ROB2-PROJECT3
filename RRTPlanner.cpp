/*
 * RRTPlanner.cpp
 *
 *  Created on: Mar 8, 2012
 *      Author: skytthe
 */

#include "RRTPlanner.h"
#include "RRT.h"

RRTPlanner::RRTPlanner()
{

}

Ptr<QPath> RRTPlanner::plan(std::list<PlannerTask > tasks)
{
	typedef std::list<PlannerTask>::iterator taskIterator;
	int totalQSize = 0;

	//Initialise
	for(taskIterator  task = tasks.begin();task != tasks.end();++task)
		totalQSize += task->getQStart()->size();

	Ptr<Q> lowerBound = new Q(totalQSize);
	Ptr<Q> upperBound = new Q(totalQSize);

	int indexPointer = 0;

	Device::QBox bounds;

	Ptr<Q> qStart 	= new Q(totalQSize);
	Ptr<Q> qGoal 	= new Q(totalQSize);

	for(taskIterator  task = tasks.begin();task != tasks.end();++task)
	{
		bounds = task->getDevice()->getBounds();
		lowerBound->setSubPart(indexPointer,bounds.first);
		upperBound->setSubPart(indexPointer,bounds.second);

		qStart->setSubPart(indexPointer,*task->getQStart());
		qGoal->setSubPart(indexPointer,*task->getQGoal());

		indexPointer += bounds.first.size();
	}

	std::cout << Math::ranQ(bounds) << std::endl;



}


RRTPlanner::~RRTPlanner() {

}
