/*
 * decoupledRRTPlanner.cpp
 *
 *  Created on: Mar 7, 2012
 *      Author: skytthe, hundevant and madsen
 */

#include "decoupledRRTPlanner.h"
#include "PlannerTask.hpp"
#include "RRTPlanner2.h"
#include "RRT.h"
#include "RRTNode.h"

#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>

#include <rw/rw.hpp>

//collision
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

const double connectEpsilon = 0.001, expandEpsilon = .02;


decoupledRRTPlanner::decoupledRRTPlanner(rws::RobWorkStudio* robWorkStudio) :
	_robWorkStudio(robWorkStudio)
{
	_workcell = _robWorkStudio->getWorkcell();
	_deviceA = _workcell->findDevice("KukaKr16A");
	_deviceB = _workcell->findDevice("KukaKr16B");

	using namespace rw::proximity;
	using namespace rwlibs::proximitystrategies;

	rw::proximity::ProximityFilterStrategy::Ptr filter = new rw::proximity::BasicFilterStrategy(_workcell);

	filter->addRule(rw::proximity::ProximitySetupRule::makeExclude("*","*"));
	filter->addRule(rw::proximity::ProximitySetupRule::makeInclude("KukaKr16A.*","KukaKr16B.*"));

	rw::proximity::CollisionStrategy::Ptr cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
	_detector = new rw::proximity::CollisionDetector(_workcell, cdstrategy,filter);

}

decoupledRRTPlanner::~decoupledRRTPlanner() {

}


std::vector<std::vector<bool> > decoupledRRTPlanner::planSspaceMap(std::list<Ptr<PlannerTask> > tasks)
{

	typedef std::list<Ptr<PlannerTask> >::iterator taskIterator;
	std::vector< std::vector<bool> > sMatrix(101, std::vector<bool>(101));



	Ptr<RRTPlanner> rrtplanner = new RRTPlanner();
	rw::trajectory::QPath tempPathQs;
	rw::trajectory::QPath tempPath;
	rw::trajectory::QPath currentRRTPath;
	std::vector<rw::trajectory::QPath> allRRTPaths;
	Ptr<PlannerTask> tempTask;

	for(taskIterator task = tasks.begin();task != tasks.end();task++)
	{
		tempPathQs = (*task)->getPathSteps();
		for (int i = 0; i < (tempPathQs.size()-1); i++) {
			tempTask = new PlannerTask((*task)->getDevice(),(*task)->getConstraint(),tempPathQs.at(i),tempPathQs.at(i+1));

			std::list<Ptr<PlannerTask> > tasks;
			tasks.push_back(tempTask);
			rrtplanner->plan(tasks);
			tempPath = tempTask->getPath();

			for (unsigned int j = 0; j < tempPath.size()-1; j++) {
				currentRRTPath.push_back(tempPath.at((tempPath.size()-1)-j));
			}
		}
		currentRRTPath.push_back(tempPathQs.at(tempPathQs.size()-1));
		allRRTPaths.push_back(currentRRTPath);
		currentRRTPath.clear();
	}


	//planning in the S-space (only for 2 robots

	_norm2A = 0.;
	_norm2B = 0.;

	rw::trajectory::QPath pathA;
	pathA = allRRTPaths.at(0);
	_pathA = allRRTPaths.at(0);
	rw::trajectory::QPath pathB;
	pathB = allRRTPaths.at(1);
	_pathB = allRRTPaths.at(1);

	rw::math::Q qTemp;

	//norm2 for pathA
	for (int i = 0; i < (pathA.size()-1); ++i) {
		qTemp = pathA.at(i)-pathA.at(i+1);
		_norm2posListA.push_back(_norm2A);
		_norm2A = _norm2A + qTemp.norm2();

	}
	_norm2posListA.push_back(_norm2A);


	//norm2 for pathB
	for (int i = 0; i < (pathB.size()-1); ++i) {
		qTemp = pathB.at(i)-pathB.at(i+1);
		_norm2posListB.push_back(_norm2B);
		_norm2B = _norm2B + qTemp.norm2();
	}
	_norm2posListB.push_back(_norm2B);

	double posS1,posS2;
	posS1 = 0.*_norm2A;
   	posS2 = 0.*_norm2B;

   	int qNumA=0,qNumB=0;
   	while(qNumA<(_norm2posListA.size()-2) && posS1>_norm2posListA.at(qNumA+1))
   	{
   		qNumA++;
   	}
   	while(qNumB<(_norm2posListB.size()-2) && posS2>_norm2posListB.at(qNumB+1))
   	{
   		qNumB++;
   	}

	std::size_t qprintSize = 2;

	bool temp;
	std::cout << "sSpace = [ \n";
	for (int ii = 0; ii < 1001; ii++)
	{
		for (int jj = 0; jj < 1001; jj++)
		{
			rw::math::Q *itrQ = new rw::math::Q(qprintSize,(((double)(1000-ii))/1000),(((double)jj)/1000));
			temp = inCollision(*itrQ,pathA,pathB);
			std::cout << temp;
			sMatrix[ii][jj] = temp;
		}
		std::cout << "; \n";
	}
	std::cout << "]";

	return sMatrix;

}


bool decoupledRRTPlanner::plan(std::list<Ptr<PlannerTask> > tasks)
//rw::trajectory::QPath[] decoupledRRTPlanner::plan()
//void decoupledRRTPlanner::plan()
{

	typedef std::list<Ptr<PlannerTask> >::iterator taskIterator;

	Ptr<RRTPlanner2> rrtplanner = new RRTPlanner2();
	rw::trajectory::QPath tempPathQs;
	rw::trajectory::QPath tempPath;
	rw::trajectory::QPath currentRRTPath;
	std::vector<rw::trajectory::QPath> allRRTPaths;
	Ptr<PlannerTask> tempTask;

	for(taskIterator task = tasks.begin();task != tasks.end();task++)
	{
		tempPathQs = (*task)->getPathSteps();
		for (int i = 0; i < (tempPathQs.size()-1); i++) {
			tempTask = new PlannerTask((*task)->getDevice(),(*task)->getConstraint(),tempPathQs.at(i),tempPathQs.at(i+1));
//			std::cout << "path: " << i << " qinit " << tempPathQs.at(i) << " qgoal " <<tempPathQs.at(i+1) << std::endl;
			rrtplanner->plan(tempTask);
			tempPath = tempTask->getPath();

			for (unsigned int j = 0; j < tempPath.size()-1; j++) {
				currentRRTPath.push_back(tempPath.at(j));
//				std::cout << "path steps: " << tempPath.at((tempPath.size()-1)-j) << std::endl;
			}
		}
		currentRRTPath.push_back(tempPathQs.at(tempPathQs.size()-1));
		allRRTPaths.push_back(currentRRTPath);
		currentRRTPath.clear();
	}


	// print the non velocity tuned path
//	for (unsigned int i = 0; i < allRRTPaths.size(); i++)
//	{
//		std::cout << "path " << i << std::endl;
//		for (unsigned int j = 0; j < allRRTPaths.at(i).size(); j++) {
//			std::cout<< " " << i << ":" << j << allRRTPaths.at(i).at(j) << std::endl;
//		}
//	}

	//planning in the S-space (only for 2 robots
	_norm2A = 0.;
	_norm2B = 0.;

	rw::trajectory::QPath pathA;
	pathA = allRRTPaths.at(0);
	_pathA = allRRTPaths.at(0);
	rw::trajectory::QPath pathB;
	pathB = allRRTPaths.at(1);
	_pathB = allRRTPaths.at(1);

	rw::math::Q qTemp;

	//norm2 for pathA
	for (int i = 0; i < (pathA.size()-1); ++i) {
		qTemp = pathA.at(i)-pathA.at(i+1);
		_norm2posListA.push_back(_norm2A);
		_norm2A = _norm2A + qTemp.norm2();

	}
	_norm2posListA.push_back(_norm2A);



	//norm2 for pathB
	for (int i = 0; i < (pathB.size()-1); ++i) {
		qTemp = pathB.at(i)-pathB.at(i+1);
		_norm2posListB.push_back(_norm2B);
//		std::cout << i << "  " << qTemp.norm2() << "  - " << _norm2B << std::endl;
		_norm2B = _norm2B + qTemp.norm2();
	}
	_norm2posListB.push_back(_norm2B);

	double posS1,posS2;
	posS1 = 0.*_norm2A;
   	posS2 = 0.*_norm2B;

   	int qNumA=0,qNumB=0;
   	while(qNumA<(_norm2posListA.size()-2) && posS1>_norm2posListA.at(qNumA+1))
   	{
   		qNumA++;
   	}
   	while(qNumB<(_norm2posListB.size()-2) && posS2>_norm2posListB.at(qNumB+1))
   	{
   		qNumB++;
   	}


//	int itr = 0;
//	for(taskIterator  task = tasks.begin();task != tasks.end();task++)
//	{
//		(*task)->setPath(allRRTPaths.at(itr));
//		itr++;
//	}

//	std::cout << "\n s map ------------------------------------------------------------------------------" << std::endl;
//
//	std::size_t qprintSize = 2;
//	bool temp;
//	std::cout << "sSpace = [ \n";
//	for (int ii = 0; ii < 101; ii++)
//	{
//		for (int jj = 0; jj < 101; jj++)
//		{
//			rw::math::Q *itrQ = new rw::math::Q(qprintSize,(((double)(ii))/100),(((double)jj)/100));
//			temp = inCollision(*itrQ,pathA,pathB);
//			std::cout << temp;
//		}
//		std::cout << "; \n";
//	}
//	std::cout << "]";
//
//
//	std::cout << "s nodes ------------------------------------------------------------------------------" << std::endl;


	rw::trajectory::QPath sPath;

	std::size_t qSize = 2;
	rw::math::Q *sInit = new rw::math::Q(qSize,0.0,0.0);
	rw::math::Q *sGoal = new rw::math::Q(qSize,1.0,1.0);

	Ptr<PlannerTask> sTask = new PlannerTask(NULL,NULL,*sInit,*sGoal);

//	std::cout << "0" << std::endl;

	Ptr<RRT> treeA = new RRT(new RRTNode(*sInit,NULL));
	Ptr<RRT> treeB = new RRT(new RRTNode(*sGoal,NULL));
	Ptr<RRT> tempTree;


	Q qRandom,qClose,qDirection,qStep,qNew;
	Ptr<RRTNode> nodeClose,nodeNew;

//	std::cout << "1" << std::endl;

	for(int i = 0;i<1000;i++)
	{
		qRandom = randQ();
//		std::cout << "2.1" << std::endl;

		nodeClose = treeA->getClosestNode(qRandom);
//		std::cout << "2.2" << std::endl;

		qClose = nodeClose->getValue();
//		std::cout << "2.2" << std::endl;

		qDirection = qRandom - qClose;
//		std::cout << "2.3" << std::endl;

		qStep = expandEpsilon*qDirection/qDirection.norm2();
	//	std::cout << "2.4" << std::endl;
		qNew = qClose + qStep;

	//	std::cout << "2" << std::endl;

		if(!edgeCollisionDetection(qClose,qNew,sTask))
		{
		//	std::cout << "3.1" << std::endl;

			nodeNew = new RRTNode(qNew, nodeClose);
			if(connect(treeB,nodeNew,sTask))
			{
	//			std::cout << "3.2" << std::endl;
				break;
			}
		}

	//	std::cout << "3" << std::endl;

//		if(i%100 == 0) std::cout << i << std::endl;

		//Tree swap
		tempTree = treeA; treeA = treeB; treeB = tempTree;

		if (i == 999)
			return false;
	}


	sPath = sTask->getPath();




	//print path in s-space
//	std::cout << "COLLISION CHECK HERE START" << std::endl;
//	for (unsigned int t = 0; t < sPath.size(); t++) {
//		if (inCollision(sPath.at(t),pathA,pathB))
//		{
//			std::cout <<t<< " path s:" << sPath.at(t) << "            in collision" << std::endl;
//		}else {
//			std::cout <<t<< " path s:" << sPath.at(t) << "            not in collision" << std::endl;
//		}
//	}
//	std::cout << "COLLISION CHECK HERE END" << std::endl;



	double norm2sPrint = 0;
	//norm2 for pathA
	for (int i = 0; i < (sPath.size()-1); ++i) {
		qTemp = sPath.at(i)-sPath.at(i+1);
		norm2sPrint = norm2sPrint + qTemp.norm2();
	}



	rw::trajectory::QPath sPathSteps;
	rw::math::Q sDirection,sStep,sTemp;
	int numberOfSteps;

	for(int i = 0; i < (sPath.size()-1); i++){

		sDirection = sPath.at(i+1) - sPath.at(i);
		sStep = 0.01*sDirection/sDirection.norm2();
		numberOfSteps = floor(sDirection.norm2()/sStep.norm2());
		sTemp = sPath.at(i);

		for (int j = 0; j < numberOfSteps; j++) {
			sPathSteps.push_back(sTemp);
			sTemp += sStep;
		}
	}
	sPathSteps.push_back(sPath.at(sPath.size()-1));


	//print path in s-space
//	std::cout << "COLLISION CHECK2 HERE START" << std::endl;
//	for (unsigned int t = 0; t < sPathSteps.size(); t++) {
//		std::cout <<t<< " steppath s:        " << sPathSteps.at(t) << std::endl;
//		if (inCollision(sPathSteps.at(t),pathA,pathB))
//		{
//			std::cout << "            in collision" << std::endl;
//		}else {
//			std::cout << "            not in collision" << std::endl;
//		}
//	}
//	std::cout << "COLLISION CHECK2 HERE END" << std::endl;




	//create final path
	rw::trajectory::QPath finalPathA,finalPathB;
	std::vector<rw::trajectory::QPath> allFinalPaths;
   	rw::math::Q qA,qB;


//	for (int k = 0; k < sPathSteps.size(); ++k) {
	for (int k = 0; k < sPath.size(); ++k) {
		rw::math::Q thisS;
//		thisS = sPathSteps.at(k);
		thisS = sPath.at(k);
		double posS1,posS2;
		posS1 = (thisS[0])*_norm2A;
	   	posS2 = (thisS[1])*_norm2B;

	   	int qNumA=0,qNumB=0;
	   	while(qNumA<(_norm2posListA.size()-2) && posS1>_norm2posListA.at(qNumA+1))
	   	{
	   		qNumA++;
	   	}
	   	while(qNumB<(_norm2posListB.size()-2) && posS2>_norm2posListB.at(qNumB+1))
	   	{
	   		qNumB++;
	   	}

	   	//gen qA
	   	rw::math::Q qA;
	   	qA = pathA.at(qNumA)+
	   			(((posS1)-_norm2posListA.at(qNumA))/((_norm2posListA.at(qNumA+1))-(_norm2posListA.at(qNumA))))
	   			*(pathA.at(qNumA+1)-pathA.at(qNumA));

	   	//gen qB
	   	rw::math::Q qB;
	   	qB = pathB.at(qNumB)+
	   			(((posS2)-_norm2posListB.at(qNumB))/((_norm2posListB.at(qNumB+1))-(_norm2posListB.at(qNumB))))
	   			*(pathB.at(qNumB+1)-pathB.at(qNumB));

	   	finalPathA.push_back(qA);
	   	finalPathB.push_back(qB);
	}

	allFinalPaths.push_back(finalPathA);
	allFinalPaths.push_back(finalPathB);


	int itr = 0;
	for(taskIterator  task = tasks.begin();task != tasks.end();task++)
	{
		(*task)->setPath(allFinalPaths.at(itr));
		itr++;
	}

	return true;



//	std::cout << "\n s path ------------------------------------------------------------------------------" << std::endl;
//
//	for (unsigned int j = 0; j < sPathSteps.size(); j++) {
//		std::cout<< sPathSteps.at(j);
//	}



}



rw::math::Q decoupledRRTPlanner::randQ()
{
	double randS1,randS2;

	randS1 = rw::math::Math::ran(0.0,1.0);
	randS2 = rw::math::Math::ran(0.0,1.0);

	std::size_t qSize = 2;
	rw::math::Q *rndQ = new rw::math::Q(qSize,randS1,randS2);

	return *rndQ;
}

bool decoupledRRTPlanner::inCollision(rw::math::Q s,rw::trajectory::QPath pathA, rw::trajectory::QPath pathB)
{

	if(s[0]<0 || s[0]>1 || s[1]<0 || s[1]>1)
	{
		std::cout << "ERROR" << std::cout;
		return true;
	}

	double posS1,posS2;
	posS1 = (s[0])*_norm2A;
   	posS2 = (s[1])*_norm2B;

   	int qNumA=0,qNumB=0;
   	while(qNumA<(_norm2posListA.size()-2) && posS1>_norm2posListA.at(qNumA+1))
   	{
   		qNumA++;
   	}
   	while(qNumB<(_norm2posListB.size()-2) && posS2>_norm2posListB.at(qNumB+1))
   	{
   		qNumB++;
   	}

   	//gen qA
   	rw::math::Q qA;
   	qA = pathA.at(qNumA)+
   			(((posS1) - _norm2posListA.at(qNumA))/((_norm2posListA.at(qNumA+1))-(_norm2posListA.at(qNumA))))
   			*(pathA.at(qNumA+1)-pathA.at(qNumA));

   	//gen qB
   	rw::math::Q qB;
   	qB = pathB.at(qNumB)+
   			(((posS2)-_norm2posListB.at(qNumB))/((_norm2posListB.at(qNumB+1))-(_norm2posListB.at(qNumB))))
   			*(pathB.at(qNumB+1)-pathB.at(qNumB));


	rw::kinematics::State state = _robWorkStudio->getWorkCell()->getDefaultState();

	_deviceA->setQ(qA,state);
	_deviceB->setQ(qB,state);

	//return true if in collision
    return _detector->inCollision(state);

}

bool decoupledRRTPlanner::connect(Ptr<RRT> tree, Ptr<RRTNode> node,Ptr<PlannerTask> task)
{
	Ptr<RRTNode> nodeInOppositeTree = tree->getClosestNode(node->getValue());
	Q qClose = nodeInOppositeTree->getValue();
	Q qDirection = qClose - node->getValue();
	Q qStep = connectEpsilon*qDirection/qDirection.norm2();

	Q qNext = node->getValue();
	Q qDistanceToGoal;

	bool reached = false;

	while(!reached && !inCollision((qNext += qStep),_pathA,_pathB))
	{
		qDistanceToGoal = qClose - qNext;
		if(qDistanceToGoal.norm2() < connectEpsilon)
			reached = true;
	}

	Ptr<RRTNode> nodeNew;

	//Subtract step in collision
	if(!reached)
		nodeNew = new RRTNode(qNext - qStep,node);
	else
	{
		nodeNew = new RRTNode(qNext,node);
		task->setPath(getPath(nodeNew,nodeInOppositeTree,task));
	}

	//Only add new node if new configuration
	if(qNext != node->getValue())
		nodeNew = new RRTNode(qNext,node);

	return reached;
}

QPath decoupledRRTPlanner::getPath(Ptr<RRTNode> node1, Ptr<RRTNode> node2, Ptr<PlannerTask> task)
{
	typedef std::list<Q >::iterator qIterator;

	std::list<Q> path;
	Ptr<RRTNode> tempNode;

	tempNode = node1;
	do
	{
		path.push_back(tempNode->getValue());
		tempNode = tempNode->getParrent();
	}while(tempNode != NULL);

	tempNode = node2;
	do
	{
		path.push_front(tempNode->getValue());
		tempNode = tempNode->getParrent();
	} while(tempNode != NULL);

	if(path.front() == task->getQGoal())
		path.reverse();

	QPath qPath;
	for(qIterator q = path.begin();q != path.end();q++)
		qPath.push_back(*q);

	return qPath;
}
bool decoupledRRTPlanner::edgeCollisionDetection(Q qStart, Q qEnd, Ptr<PlannerTask> task)
{
//	std::cout << "4" << std::endl;

	//Use a resolution of epsilon to test edge
	const double eps = 0.001;

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
				if(inCollision(qTemp,_pathA,_pathB))
					return true;
		}
	}
	return false;
}
