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


bool RRTPlanner::edgeCollisionDetection(rw::common::Ptr<RRTNode> nodeClose, rw::common::Ptr<RRTNode> nodeNew,Ptr<PlannerTask> task)
{
	//Use a resolution of epsilon to test edge
	const double eps = 0.01;

	//Initialize end point of edge
	rw::math::Q qStart = nodeNew->getValue();
	rw::math::Q qEnd = nodeClose->getValue();

	//Initialize vector from start to end
	const rw::math::Q qDelta = qEnd - qStart;

	//Calculate edge "length" in joint space
	const double normDeltaQ = qDelta.norm2();

	//Use binary search for edge collision detection
	const int n = ceil(normDeltaQ/eps);

	const int levels = Math::ceilLog2(n);
	const double extendedLength = pow(2,levels)*eps;

	//Extend edge to get optimal edge intervals
	const Q qExtended = (qDelta/qDelta.norm2())*extendedLength;

	Q qStep,qTemp;
	int steps;

	for(int i = 1;i <= levels;i++)
	{
		steps = pow(2,i-1);
		qStep = qExtended/steps;
		for(int j = 1 ; j<= steps;j++)
		{
			qTemp = qStart + (j - 1/2)*qStep;

			//Only do collision check if config in the edge
			if( ((j - 1/2)*qStep).norm2() <= normDeltaQ )
				//Return false if any configuration along the edge is in collision
				if(task->getConstraint()->inCollision(qTemp))
					return true;
		}
	}
	return false;
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

//				if ( edgeCollisionDetection( nodeNew , nodeNear , *task) )
//					collision = true;

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
