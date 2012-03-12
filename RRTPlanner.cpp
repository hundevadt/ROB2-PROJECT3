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

void RRTPlanner::plan(std::list<Ptr<PlannerTask> > tasks)
{
	typedef std::list<Ptr<PlannerTask> >::iterator taskIterator;
	int totalQSize = 0;

	for(taskIterator task = tasks.begin();task != tasks.end();task++)
		totalQSize += (*task)->getQStart().size();

	Ptr<Q> lowerBound = new Q(totalQSize);
	Ptr<Q> upperBound = new Q(totalQSize);

	Ptr<Q> qStart 	= new Q(totalQSize);
	Ptr<Q> qGoal 	= new Q(totalQSize);

	int indexPointer = 0;
	for(taskIterator  task = tasks.begin();task != tasks.end();task++)
	{
		Device::QBox bound = (*task)->getDevice()->getBounds();
		lowerBound->setSubPart(indexPointer,bound.first);
		upperBound->setSubPart(indexPointer,bound.second);

		qStart->setSubPart(indexPointer,(*task)->getQStart());
		qGoal->setSubPart(indexPointer,(*task)->getQGoal());

		indexPointer += bound.first.size();
	}


	double epsilon = 0.1;
	Ptr<RRTNode> nodeStart = new RRTNode(*qStart,NULL);
	Ptr<RRTNode> nodeGoal;
	Ptr<RRT> tree = new RRT(nodeStart);

	int maxAttemps = 100000;
	int attemps = 0;
	bool reached = false;
	while(!reached && attemps < maxAttemps)
	{
		Q qRandom = Math::ranQ(*lowerBound,*upperBound);
		Ptr<RRTNode> nodeNear = tree->getClosestNode(qRandom);
		Q qNear = nodeNear->getValue();
		Q qDirection = qRandom - qNear;
		Q qStep = epsilon*qDirection/qDirection.norm2();


		Q qNew = qNear + qStep;

		bool collision = false;
		indexPointer = 0;
		for(taskIterator  task = tasks.begin();task != tasks.end();task++)
		{
			int DOF = (*task)->getDevice()->getDOF();

			if((*task)->getConstraint()->inCollision(qNew.getSubPart(indexPointer,DOF)))
				collision = true;

			indexPointer += DOF;
		}

		while(!collision && !reached)
		{
			Ptr<RRTNode> nodeNew = new RRTNode(qNew,nodeNear);
			tree->addNodeToTree(nodeNew);

			Q qBridge = qNew - *qGoal;

			if(qBridge.norm2() < epsilon)
			{
				reached = true;
				nodeGoal = new RRTNode(*qGoal,nodeNew);
				tree->addNodeToTree(nodeGoal);
				break;
			}

			qDirection = *qGoal - qNew;
			qStep = epsilon*qDirection/qDirection.norm2();
			qNew += qStep;

			indexPointer = 0;
			for(taskIterator  task = tasks.begin();task != tasks.end();task++)
			{
				int DOF = (*task)->getDevice()->getDOF();

				if((*task)->getConstraint()->inCollision(qNew.getSubPart(indexPointer,DOF)))
					collision = true;

				indexPointer += DOF;
			}
		}

		attemps++;
		if(attemps%100 == 0)
			std::cout << attemps << std::endl;
	}

	std::cout << tree->getListOfNodes().size() << std::endl;


	if(reached)
	{
		indexPointer = 0;
		for(taskIterator  task = tasks.begin();task != tasks.end();task++)
		{
			int DOF = (*task)->getDevice()->getDOF();

			QPath path;
			Ptr<RRTNode> tempNode = nodeGoal;
			do
			{
				path.push_back(tempNode->getValue().getSubPart(indexPointer,DOF));
				tempNode = tempNode->getParrent();
			}while(tempNode != NULL);

			(*task)->setPath(path);
			indexPointer += DOF;
		}

	}
	else
	{
		std::cout << "Path not found in " << maxAttemps << " attemps" << std::endl;
	}

}


RRTPlanner::~RRTPlanner() {

}
