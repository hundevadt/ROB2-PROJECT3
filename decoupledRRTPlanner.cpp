/*
 * decoupledRRTPlanner.cpp
 *
 *  Created on: Mar 7, 2012
 *      Author: skytthe, hundevant and madsen
 */

#include "decoupledRRTPlanner.h"
#include "PlannerTask.hpp"
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

decoupledRRTPlanner::decoupledRRTPlanner(rws::RobWorkStudio* robWorkStudio) :
	_robWorkStudio(robWorkStudio)
{
	_workcell = _robWorkStudio->getWorkcell();
	_deviceA = _workcell->findDevice("KukaKr16A");
	_deviceB = _workcell->findDevice("KukaKr16B");
}

decoupledRRTPlanner::~decoupledRRTPlanner() {

}

void decoupledRRTPlanner::plan(std::list<Ptr<PlannerTask> > tasks)
//rw::trajectory::QPath[] decoupledRRTPlanner::plan()
//void decoupledRRTPlanner::plan()
{

	typedef std::list<Ptr<PlannerTask> >::iterator taskIterator;

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


	// print the non velocity tuned path
	for (unsigned int i = 0; i < allRRTPaths.size(); i++)
	{
		std::cout << "path " << i << std::endl;
		for (unsigned int j = 0; j < allRRTPaths.at(i).size(); j++) {
			std::cout<< " " << i << ":" << j << allRRTPaths.at(i).at(j) << std::endl;
		}
	}


	//planning in the S-space (only for 2 robots

	_norm2A = 0.;
	_norm2B = 0.;

	rw::trajectory::QPath pathA;
	pathA = allRRTPaths.at(0);
	rw::trajectory::QPath pathB;
	pathB = allRRTPaths.at(1);

	rw::math::Q qTemp;

	//norm2 for pathA
	for (int i = 0; i < (pathA.size()-1); ++i) {
		qTemp = pathA.at(i)-pathA.at(i+1);
		_norm2posListA.push_back(_norm2A);
//		std::cout << i << "  " << qTemp.norm2() << "  - " << _norm2A << std::endl;
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




	std::cout << "-----------------------------------------------------------------------------------------" << std::endl;
	std::cout << "-----------------------------------------------------------------------------------------" << std::endl;
	std::cout << "-----------------------------------------------------------------------------------------" << std::endl;


	std::size_t qprintSize = 2;

	std::cout << "sSpace = [ \n";
	for (int ii = 0; ii < 101; ii++)
	{
		for (int jj = 0; jj < 101; jj++)
		{
			rw::math::Q *itrQ = new rw::math::Q(qprintSize,(((double)ii)/100),(((double)jj)/100));
			std::cout << inCollision(*itrQ,pathA,pathB);
		}
		std::cout << "; \n";
	}
	std::cout << "]";

	std::cout << "-----------------------------------------------------------------------------------------" << std::endl;
	std::cout << "-----------------------------------------------------------------------------------------" << std::endl;
	std::cout << "-----------------------------------------------------------------------------------------" << std::endl;





	rw::trajectory::QPath sPath;

	std::size_t qSize = 2;
	rw::math::Q *sInit = new rw::math::Q(qSize,0.0,0.0);
	rw::math::Q *sGoal = new rw::math::Q(qSize,1.0,1.0);

	double epsilon = 0.05;
	Ptr<RRTNode> nodeStart = new RRTNode(*sInit,NULL);
	Ptr<RRTNode> nodeGoal;
	Ptr<RRT> tree = new RRT(nodeStart);

	int maxAttemps = 100000;
	int attemps = 0;
	bool reached = false;
	while(!reached && attemps < maxAttemps)
	{
		Q qRandom = randQ();

		Ptr<RRTNode> nodeNear = tree->getClosestNode(qRandom);
		Q qNear = nodeNear->getValue();
		Q qDirection = qRandom - qNear;
		Q qStep = epsilon*qDirection/qDirection.norm2();


		Q qNew = qNear + qStep;

		bool collision = inCollision(qNew,pathA,pathB);

		while(!collision && !reached)
		{
			Ptr<RRTNode> nodeNew = new RRTNode(qNew,nodeNear);
			tree->addNodeToTree(nodeNew);

			Q qBridge = qNew - *sGoal;

			if(qBridge.norm2() < epsilon)
			{
				reached = true;
				nodeGoal = new RRTNode(*sGoal,nodeNew);
				tree->addNodeToTree(nodeGoal);
				break;
			}

			qDirection = *sGoal - qNew;
			qStep = epsilon*qDirection/qDirection.norm2();
			qNew += qStep;

			collision = inCollision(qNew,pathA,pathB);
		}

		attemps++;
		if(attemps%100 == 0)
			std::cout << attemps << std::endl;
	}

	std::cout << "Number of nodes in s-space-tree: " << tree->getListOfNodes().size() << std::endl;
	std::cout << "Number of attemps in s-space-tree: " << attemps << std::endl;

	if(reached)
	{
			Ptr<RRTNode> tempNode = nodeGoal;
			do
			{
				sPath.push_back(tempNode->getValue());
				tempNode = tempNode->getParrent();
			}while(tempNode != NULL);
	}
	else
	{
		std::cout << "Path not found in " << maxAttemps << " attemps" << std::endl;
	}








	//print path in s-space
	std::cout << "COLLISION CHECK HERE START" << std::endl;
	for (unsigned int t = 0; t < sPath.size(); t++) {
		if (inCollision(sPath.at(t),pathA,pathB))
		{
			std::cout <<t<< " path s:" << sPath.at(t) << "            in collision" << std::endl;
		}else {
			std::cout <<t<< " path s:" << sPath.at(t) << "            not in collision" << std::endl;
		}
	}
	std::cout << "COLLISION CHECK HERE END" << std::endl;



	double norm2sPrint = 0;
	//norm2 for pathA
	for (int i = 0; i < (sPath.size()-1); ++i) {
		qTemp = sPath.at(i)-sPath.at(i+1);
		norm2sPrint = norm2sPrint + qTemp.norm2();
	}
	std::cout << " sPath l:   " << norm2sPrint << " sPath size:   " << sPath.size() << std::endl;



	rw::trajectory::QPath sPathSteps;
	rw::math::Q sDirection,sStep,sTemp;
	int numberOfSteps;

	for(int i = 0; i < (sPath.size()-1); i++){

		sDirection = sPath.at(i+1) - sPath.at(i);
		sStep = 0.01*sDirection/sDirection.norm2();
		numberOfSteps = floor(sDirection.norm2()/sStep.norm2());
		sTemp = sPath.at(i);
		std::cout << i <<" :: nrStep: "<< numberOfSteps <<" step:" << sStep <<"sstep l: " << sStep.norm2() << std::endl;

		for (int j = 0; j < numberOfSteps; j++) {
			std::cout << j << std::endl;
			sPathSteps.push_back(sTemp);
			sTemp += sStep;
		}
	}
	sPathSteps.push_back(sPath.at(sPath.size()-1));


	//print path in s-space
	std::cout << "COLLISION CHECK2 HERE START" << std::endl;
	for (unsigned int t = 0; t < sPathSteps.size(); t++) {
		std::cout <<t<< " steppath s:        " << sPathSteps.at(t) << std::endl;
		if (inCollision(sPathSteps.at(t),pathA,pathB))
		{
			std::cout << "            in collision" << std::endl;
		}else {
			std::cout << "            not in collision" << std::endl;
		}
	}
	std::cout << "COLLISION CHECK2 HERE END" << std::endl;



//	for (unsigned int j = 0; j < sPathSteps.size(); j++) {
//		std::cout<< "s:" << j << " ; " << sPathSteps.at(j) << std::endl;
//	}

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


//	rw::trajectory::QPath pathA;
//	pathA = allRRTPaths.at(0);
//	rw::trajectory::QPath pathB;
//	pathB = allRRTPaths.at(1);


	int itr = 0;
	for(taskIterator  task = tasks.begin();task != tasks.end();task++)
	{
		(*task)->setPath(allFinalPaths.at(itr));
		itr++;
	}



	std::cout << "Done !!!" << std::endl;


//
//	std::size_t qprintSize = 2;
//
//	std::cout << "sSpace = [ \n";
//	for (int ii = 0; ii < 101; ii++)
//	{
//		for (int jj = 0; jj < 101; jj++)
//		{
//			rw::math::Q *itrQ = new rw::math::Q(qprintSize,(((double)ii)/100),(((double)jj)/100));
//			std::cout << inCollision(*itrQ,pathA,pathB);
//		}
//		std::cout << "; \n";
//	}
//	std::cout << "]";


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
   			(((posS1)-_norm2posListA.at(qNumA))/((_norm2posListA.at(qNumA+1))-(_norm2posListA.at(qNumA))))
   			*(pathA.at(qNumA+1)-pathA.at(qNumA));

   	//gen qB
   	rw::math::Q qB;
   	qB = pathB.at(qNumB)+
   			(((posS2)-_norm2posListB.at(qNumB))/((_norm2posListB.at(qNumB+1))-(_norm2posListB.at(qNumB))))
   			*(pathB.at(qNumB+1)-pathB.at(qNumB));

using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;

	rw::kinematics::State state = _robWorkStudio->getWorkCell()->getDefaultState();

	_deviceA->setQ(qA,state);
	_deviceB->setQ(qB,state);


	static rw::proximity::CollisionStrategy::Ptr cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
	static CollisionDetector detector(_workcell, cdstrategy);

	//return true if in collision
    return detector.inCollision(state);

    return false;
}



//Terminal Path 3
//Loaded Plugin RWSImageLoaderPlugin
//AN OPENGL ERROR: 1280
//Workcell WorkCell[PickPlacePlanner]successfully loaded.
//0org      Q[6]{-0.49, 1.133, 0.424, 3.142, -0.014, -2.061}
//27
//23
//1org      Q[6]{0.668, 0.535, -0.703, 3.766, -1.252, -0.954}
//27
//23
//2org      Q[6]{0, 0.026, 0.812, 3.149, -0.733, -1.57}
//28
//22
//0org      Q[6]{0.49, 1.133, 0.424, -3.142, -0.014, 2.061}
//28
//23
//1org      Q[6]{-0.668, 0.535, -0.703, -3.766, -1.252, 0.954}
//27
//22
//2org      Q[6]{0, 0.026, 0.812, -3.149, -0.733, 1.57}
//27
//22
//Number of nodes in s-space-tree: 77
//Number of attemps in s-space-tree: 13
//COLLISION CHECK HERE START
//0 path s:Q[2]{1, 1}            not in collision
//1 path s:Q[2]{0.974183, 0.984268}            not in collision
//2 path s:Q[2]{0.364546, 0.554631}            not in collision
//3 path s:Q[2]{0.395647, 0.515481}            not in collision
//4 path s:Q[2]{0.429868, 0.479026}            not in collision
//5 path s:Q[2]{0.46468, 0.443136}            not in collision
//6 path s:Q[2]{0, 0}            not in collision
//COLLISION CHECK HERE END
// sPath l:   1.56816 sPath size:   7
//0 :: nrStep: 3 step:Q[2]{-0.00853948, -0.00520358}sstep l: 0.01
//0
//1
//2
//1 :: nrStep: 74 step:Q[2]{-0.00817406, -0.00576061}sstep l: 0.01
//0
//1
//2
//3
//4
//5
//6
//7
//8
//9
//10
//11
//12
//13
//14
//15
//16
//17
//18
//19
//20
//21
//22
//23
//24
//25
//26
//27
//28
//29
//30
//31
//32
//33
//34
//35
//36
//37
//38
//39
//40
//41
//42
//43
//44
//45
//46
//47
//48
//49
//50
//51
//52
//53
//54
//55
//56
//57
//58
//59
//60
//61
//62
//63
//64
//65
//66
//67
//68
//69
//70
//71
//72
//73
//2 :: nrStep: 5 step:Q[2]{0.00622037, -0.00782987}sstep l: 0.01
//0
//1
//2
//3
//4
//3 :: nrStep: 5 step:Q[2]{0.00684408, -0.00729099}sstep l: 0.01
//0
//1
//2
//3
//4
//4 :: nrStep: 5 step:Q[2]{0.00696252, -0.00717797}sstep l: 0.01
//0
//1
//2
//3
//4
//5 :: nrStep: 64 step:Q[2]{-0.00723684, -0.00690132}sstep l: 0.01
//0
//1
//2
//3
//4
//5
//6
//7
//8
//9
//10
//11
//12
//13
//14
//15
//16
//17
//18
//19
//20
//21
//22
//23
//24
//25
//26
//27
//28
//29
//30
//31
//32
//33
//34
//35
//36
//37
//38
//39
//40
//41
//42
//43
//44
//45
//46
//47
//48
//49
//50
//51
//52
//53
//54
//55
//56
//57
//58
//59
//60
//61
//62
//63
//COLLISION CHECK2 HERE START
//0 steppath s:        Q[2]{1, 1}
//            not in collision
//1 steppath s:        Q[2]{0.991461, 0.994796}
//            not in collision
//2 steppath s:        Q[2]{0.982921, 0.989593}
//            not in collision
//3 steppath s:        Q[2]{0.974183, 0.984268}
//            not in collision
//4 steppath s:        Q[2]{0.966009, 0.978507}
//            not in collision
//5 steppath s:        Q[2]{0.957835, 0.972747}
//            not in collision
//6 steppath s:        Q[2]{0.949661, 0.966986}
//            in collision
//7 steppath s:        Q[2]{0.941486, 0.961226}
//            in collision
//8 steppath s:        Q[2]{0.933312, 0.955465}
//            in collision
//9 steppath s:        Q[2]{0.925138, 0.949704}
//            not in collision
//10 steppath s:        Q[2]{0.916964, 0.943944}
//            not in collision
//11 steppath s:        Q[2]{0.90879, 0.938183}
//            not in collision
//12 steppath s:        Q[2]{0.900616, 0.932423}
//            not in collision
//13 steppath s:        Q[2]{0.892442, 0.926662}
//            not in collision
//14 steppath s:        Q[2]{0.884268, 0.920901}
//            not in collision
//15 steppath s:        Q[2]{0.876094, 0.915141}
//            not in collision
//16 steppath s:        Q[2]{0.86792, 0.90938}
//            in collision
//17 steppath s:        Q[2]{0.859746, 0.90362}
//            in collision
//18 steppath s:        Q[2]{0.851572, 0.897859}
//            not in collision
//19 steppath s:        Q[2]{0.843398, 0.892098}
//            not in collision
//20 steppath s:        Q[2]{0.835224, 0.886338}
//            not in collision
//21 steppath s:        Q[2]{0.82705, 0.880577}
//            not in collision
//22 steppath s:        Q[2]{0.818876, 0.874816}
//            not in collision
//23 steppath s:        Q[2]{0.810701, 0.869056}
//            not in collision
//24 steppath s:        Q[2]{0.802527, 0.863295}
//            not in collision
//25 steppath s:        Q[2]{0.794353, 0.857535}
//            not in collision
//26 steppath s:        Q[2]{0.786179, 0.851774}
//            not in collision
//27 steppath s:        Q[2]{0.778005, 0.846013}
//            not in collision
//28 steppath s:        Q[2]{0.769831, 0.840253}
//            not in collision
//29 steppath s:        Q[2]{0.761657, 0.834492}
//            not in collision
//30 steppath s:        Q[2]{0.753483, 0.828732}
//            not in collision
//31 steppath s:        Q[2]{0.745309, 0.822971}
//            not in collision
//32 steppath s:        Q[2]{0.737135, 0.81721}
//            not in collision
//33 steppath s:        Q[2]{0.728961, 0.81145}
//            not in collision
//34 steppath s:        Q[2]{0.720787, 0.805689}
//            not in collision
//35 steppath s:        Q[2]{0.712613, 0.799928}
//            not in collision
//36 steppath s:        Q[2]{0.704439, 0.794168}
//            not in collision
//37 steppath s:        Q[2]{0.696265, 0.788407}
//            not in collision
//38 steppath s:        Q[2]{0.68809, 0.782647}
//            in collision
//39 steppath s:        Q[2]{0.679916, 0.776886}
//            in collision
//40 steppath s:        Q[2]{0.671742, 0.771125}
//            not in collision
//41 steppath s:        Q[2]{0.663568, 0.765365}
//            not in collision
//42 steppath s:        Q[2]{0.655394, 0.759604}
//            not in collision
//43 steppath s:        Q[2]{0.64722, 0.753844}
//            not in collision
//44 steppath s:        Q[2]{0.639046, 0.748083}
//            not in collision
//45 steppath s:        Q[2]{0.630872, 0.742322}
//            not in collision
//46 steppath s:        Q[2]{0.622698, 0.736562}
//            not in collision
//47 steppath s:        Q[2]{0.614524, 0.730801}
//            not in collision
//48 steppath s:        Q[2]{0.60635, 0.72504}
//            not in collision
//49 steppath s:        Q[2]{0.598176, 0.71928}
//            not in collision
//50 steppath s:        Q[2]{0.590002, 0.713519}
//            not in collision
//51 steppath s:        Q[2]{0.581828, 0.707759}
//            not in collision
//52 steppath s:        Q[2]{0.573654, 0.701998}
//            not in collision
//53 steppath s:        Q[2]{0.56548, 0.696237}
//            not in collision
//54 steppath s:        Q[2]{0.557305, 0.690477}
//            not in collision
//55 steppath s:        Q[2]{0.549131, 0.684716}
//            not in collision
//56 steppath s:        Q[2]{0.540957, 0.678956}
//            not in collision
//57 steppath s:        Q[2]{0.532783, 0.673195}
//            not in collision
//58 steppath s:        Q[2]{0.524609, 0.667434}
//            not in collision
//59 steppath s:        Q[2]{0.516435, 0.661674}
//            not in collision
//60 steppath s:        Q[2]{0.508261, 0.655913}
//            not in collision
//61 steppath s:        Q[2]{0.500087, 0.650153}
//            not in collision
//62 steppath s:        Q[2]{0.491913, 0.644392}
//            not in collision
//63 steppath s:        Q[2]{0.483739, 0.638631}
//            not in collision
//64 steppath s:        Q[2]{0.475565, 0.632871}
//            not in collision
//65 steppath s:        Q[2]{0.467391, 0.62711}
//            not in collision
//66 steppath s:        Q[2]{0.459217, 0.621349}
//            not in collision
//67 steppath s:        Q[2]{0.451043, 0.615589}
//            in collision
//68 steppath s:        Q[2]{0.442869, 0.609828}
//            in collision
//69 steppath s:        Q[2]{0.434694, 0.604068}
//            in collision
//70 steppath s:        Q[2]{0.42652, 0.598307}
//            not in collision
//71 steppath s:        Q[2]{0.418346, 0.592546}
//            not in collision
//72 steppath s:        Q[2]{0.410172, 0.586786}
//            not in collision
//73 steppath s:        Q[2]{0.401998, 0.581025}
//            not in collision
//74 steppath s:        Q[2]{0.393824, 0.575265}
//            not in collision
//75 steppath s:        Q[2]{0.38565, 0.569504}
//            not in collision
//76 steppath s:        Q[2]{0.377476, 0.563743}
//            not in collision
//77 steppath s:        Q[2]{0.364546, 0.554631}
//            not in collision
//78 steppath s:        Q[2]{0.370766, 0.546801}
//            not in collision
//79 steppath s:        Q[2]{0.376986, 0.538971}
//            not in collision
//80 steppath s:        Q[2]{0.383207, 0.531141}
//            not in collision
//81 steppath s:        Q[2]{0.389427, 0.523311}
//            not in collision
//82 steppath s:        Q[2]{0.395647, 0.515481}
//            not in collision
//83 steppath s:        Q[2]{0.402492, 0.50819}
//            not in collision
//84 steppath s:        Q[2]{0.409336, 0.500899}
//            not in collision
//85 steppath s:        Q[2]{0.41618, 0.493608}
//            not in collision
//86 steppath s:        Q[2]{0.423024, 0.486317}
//            not in collision
//87 steppath s:        Q[2]{0.429868, 0.479026}
//            not in collision
//88 steppath s:        Q[2]{0.43683, 0.471848}
//            not in collision
//89 steppath s:        Q[2]{0.443793, 0.46467}
//            not in collision
//90 steppath s:        Q[2]{0.450755, 0.457492}
//            in collision
//91 steppath s:        Q[2]{0.457718, 0.450314}
//            in collision
//92 steppath s:        Q[2]{0.46468, 0.443136}
//            not in collision
//93 steppath s:        Q[2]{0.457444, 0.436235}
//            not in collision
//94 steppath s:        Q[2]{0.450207, 0.429334}
//            not in collision
//95 steppath s:        Q[2]{0.44297, 0.422433}
//            not in collision
//96 steppath s:        Q[2]{0.435733, 0.415531}
//            not in collision
//97 steppath s:        Q[2]{0.428496, 0.40863}
//            not in collision
//98 steppath s:        Q[2]{0.421259, 0.401729}
//            in collision
//99 steppath s:        Q[2]{0.414023, 0.394827}
//            in collision
//100 steppath s:        Q[2]{0.406786, 0.387926}
//            in collision
//101 steppath s:        Q[2]{0.399549, 0.381025}
//            in collision
//102 steppath s:        Q[2]{0.392312, 0.374123}
//            not in collision
//103 steppath s:        Q[2]{0.385075, 0.367222}
//            not in collision
//104 steppath s:        Q[2]{0.377838, 0.360321}
//            not in collision
//105 steppath s:        Q[2]{0.370602, 0.353419}
//            not in collision
//106 steppath s:        Q[2]{0.363365, 0.346518}
//            not in collision
//107 steppath s:        Q[2]{0.356128, 0.339617}
//            not in collision
//108 steppath s:        Q[2]{0.348891, 0.332715}
//            not in collision
//109 steppath s:        Q[2]{0.341654, 0.325814}
//            not in collision
//110 steppath s:        Q[2]{0.334417, 0.318913}
//            not in collision
//111 steppath s:        Q[2]{0.327181, 0.312011}
//            not in collision
//112 steppath s:        Q[2]{0.319944, 0.30511}
//            not in collision
//113 steppath s:        Q[2]{0.312707, 0.298209}
//            not in collision
//114 steppath s:        Q[2]{0.30547, 0.291307}
//            not in collision
//115 steppath s:        Q[2]{0.298233, 0.284406}
//            not in collision
//116 steppath s:        Q[2]{0.290996, 0.277505}
//            not in collision
//117 steppath s:        Q[2]{0.283759, 0.270604}
//            not in collision
//118 steppath s:        Q[2]{0.276523, 0.263702}
//            not in collision
//119 steppath s:        Q[2]{0.269286, 0.256801}
//            not in collision
//120 steppath s:        Q[2]{0.262049, 0.2499}
//            not in collision
//121 steppath s:        Q[2]{0.254812, 0.242998}
//            not in collision
//122 steppath s:        Q[2]{0.247575, 0.236097}
//            not in collision
//123 steppath s:        Q[2]{0.240338, 0.229196}
//            not in collision
//124 steppath s:        Q[2]{0.233102, 0.222294}
//            in collision
//125 steppath s:        Q[2]{0.225865, 0.215393}
//            in collision
//126 steppath s:        Q[2]{0.218628, 0.208492}
//            in collision
//127 steppath s:        Q[2]{0.211391, 0.20159}
//            in collision
//128 steppath s:        Q[2]{0.204154, 0.194689}
//            in collision
//129 steppath s:        Q[2]{0.196917, 0.187788}
//            in collision
//130 steppath s:        Q[2]{0.189681, 0.180886}
//            in collision
//131 steppath s:        Q[2]{0.182444, 0.173985}
//            in collision
//132 steppath s:        Q[2]{0.175207, 0.167084}
//            in collision
//133 steppath s:        Q[2]{0.16797, 0.160182}
//            in collision
//134 steppath s:        Q[2]{0.160733, 0.153281}
//            in collision
//135 steppath s:        Q[2]{0.153496, 0.14638}
//            in collision
//136 steppath s:        Q[2]{0.14626, 0.139479}
//            in collision
//137 steppath s:        Q[2]{0.139023, 0.132577}
//            in collision
//138 steppath s:        Q[2]{0.131786, 0.125676}
//            in collision
//139 steppath s:        Q[2]{0.124549, 0.118775}
//            in collision
//140 steppath s:        Q[2]{0.117312, 0.111873}
//            in collision
//141 steppath s:        Q[2]{0.110075, 0.104972}
//            in collision
//142 steppath s:        Q[2]{0.102839, 0.0980706}
//            in collision
//143 steppath s:        Q[2]{0.0956017, 0.0911693}
//            in collision
//144 steppath s:        Q[2]{0.0883649, 0.084268}
//            in collision
//145 steppath s:        Q[2]{0.081128, 0.0773667}
//            in collision
//146 steppath s:        Q[2]{0.0738912, 0.0704654}
//            in collision
//147 steppath s:        Q[2]{0.0666543, 0.063564}
//            in collision
//148 steppath s:        Q[2]{0.0594175, 0.0566627}
//            in collision
//149 steppath s:        Q[2]{0.0521807, 0.0497614}
//            in collision
//150 steppath s:        Q[2]{0.0449438, 0.0428601}
//            in collision
//151 steppath s:        Q[2]{0.037707, 0.0359588}
//            in collision
//152 steppath s:        Q[2]{0.0304702, 0.0290575}
//            in collision
//153 steppath s:        Q[2]{0.0232333, 0.0221561}
//            in collision
//154 steppath s:        Q[2]{0.0159965, 0.0152548}
//            not in collision
//155 steppath s:        Q[2]{0.00875964, 0.00835351}
//            not in collision
//156 steppath s:        Q[2]{0, 0}
//            not in collision
//COLLISION CHECK2 HERE END
//Done !!!
//Done !!!
//sSpace = [
//00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
//00001000000000000000000000000000000000001000000000000000000000000000000000000000000000000000000000000;
//00111000000000000000000000000000000000011100000000000000000000000000000000011100000000000000000000000;
//00111100000000000000000000000000000000111100000000000000000000000000000000011110000000000000000000000;
//00111110000000000000000000000000111000111110000000000000000000000000111100111110000000000000000000000;
//00011110000000000000000000000011111000001111000000000000000000000001111000001111000000000000000000000;
//00000111000000000000000000000111100000000111100000000000000000000011100000000011100000000000000000000;
//00000011100000000000000011101110000000000011110000000000000001101110000000000001110000000000000001110;
//00000001110000000000000111110000000000000001111000000000000111111000000000000000111000000000000011111;
//00000000111000000000001111000000000000000000011000000000001111000000000000000000011100000000000111100;
//00000000011100000000011110000000000000000000001110000000011110000000000000000000001110000000011110000;
//00000000001110000001111000000000000000000000001111000000111100000000000000000000000111000000111100000;
//00000000000111000011110000000000000000000000000111100011111000000000000000000000000011100001111000000;
//00000000000011111111100000000000000000000000000011111111110000000000000000000000000001111111110000000;
//00000000000001111111000000000000000000000000000001111111000000000000000000000000000000111111100000000;
//00000000000001111100000000000000000000000000000000111110000000000000000000000000000000111110000000000;
//00000000000001111110000000000000000000000000000000111111000000000000000000000000000000111111000000000;
//00000000000001111111000000000000000000000000000001111111100000000000000000000000000000111111100000000;
//00000000000011111111100000000000000000000000000001111111100000000000000000000000000001111111110000000;
//00000000000111101111110000000000000000000000000011100111110000000000000000000000000011110111111000000;
//00000000000111000011111000000000000000000000000111100011111000000000000000000000000011100001111100000;
//00000000001110000001111100000000000000000000001111000001111100000000000000000000000111000000111110000;
//00000000011110000000111110000000000000000000011110000000111110000000000000000000001110000000011110000;
//00000000111100000000011100000000000000000000111100000000011110000000000000000000011110000000001110000;
//00000001111000000000001000000000000000000000111000000000001100000000000000000000111100000000000100000;
//00000001110000000000000000000000000000000001110000000000000000000000000000000000111000000000000000000;
//00000001100000000000000000000000000000000001110000000000000000000000000000000000110000000000000000000;
//00000001100000000000000000000000000000000000100000000000000000000000000000000000110000000000000000000;
//00000001100000000000000000000000000000000001100000000000000000000000000000000000100000000000000000000;
//00000011000000000000000000000000000000000001100000000000000000000000000000000001100000000000000000000;
//00000011000000000000000000000000000000000011000000000000000000000000000000000001100000000000000000000;
//00000110000000000000000000000000000000000111000000000000000000000000000000000011000000000000000000000;
//00001110000000000000000000000000000000000110000000000000000000000000000000000111000000000000000000000;
//00001100000000000000000000000000000000001110000000000000000000000000000000000110000000000000000000000;
//00001100000000000000000000000000000000000100000000000000000000000000000000000110000000000000000000000;
//00001000000000000000000000000000000000000000000000000000000000000000000000000100000000000000000000000;
//00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
//00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
//00011000000000000000000000000000000000011100000000000000000000000000000000001100000000000000000000000;
//00111100000000000000000000000000000000111100000000000000000000000000000000011110000000000000000000000;
//01111100000000000000000000000000010000111110000000000000000000000000011000111110000000000000000000000;
//00111110000000000000000000000001111000011110000000000000000000000001111100011111000000000000000000000;
//00001111000000000000000000000011110000001111000000000000000000000011110000000111100000000000000000000;
//00000111100000000000000001101111000000000011100000000000000001100111000000000011110000000000000000110;
//00000001110000000000000111111100000000000001110000000000000011111100000000000001111000000000000011111;
//00000000111000000000001111100000000000000000111000000000000111110000000000000000011000000000000111110;
//00000000011100000000011110000000000000000000011100000000011111000000000000000000001110000000001111000;
//00000000001110000000111100000000000000000000001110000000111100000000000000000000000111000000011110000;
//00000000000111000001111000000000000000000000000111000001111000000000000000000000000111100001111100000;
//00000000000111100111110000000000000000000000000011110011110000000000000000000000000011110011111000000;
//00000000000011111111000000000000000000000000000001111111100000000000000000000000000001111111100000000;
//00000000000001111110000000000000000000000000000001111111000000000000000000000000000000111111000000000;
//00000000000001111110000000000000000000000000000000111110000000000000000000000000000000111111000000000;
//00000000000001111111000000000000000000000000000000111111000000000000000000000000000000111111100000000;
//00000000000001111111100000000000000000000000000001111111100000000000000000000000000000111111100000000;
//00000000000011101111100000000000000000000000000011111111110000000000000000000000000001110111110000000;
//00000000000111100111110000000000000000000000000111100011111000000000000000000000000011100011111000000;
//00000000001111000001111000000000000000000000000111000001111100000000000000000000000111100001111100000;
//00000000011110000000111100000000000000000000001110000000111110000000000000000000001111000000111110000;
//00000000111100000000111100000000000000000000011100000000011110000000000000000000011110000000011110000;
//00000001111000000000011100000000000000000000111100000000001100000000000000000000111100000000001100000;
//00000001110000000000000000000000000000000001111000000000000000000000000000000000111000000000000000000;
//00000001110000000000000000000000000000000001110000000000000000000000000000000000110000000000000000000;
//00000000100000000000000000000000000000000000110000000000000000000000000000000000010000000000000000000;
//00000001100000000000000000000000000000000000100000000000000000000000000000000000110000000000000000000;
//00000001000000000000000000000000000000000001100000000000000000000000000000000001100000000000000000000;
//00000011000000000000000000000000000000000011000000000000000000000000000000000001100000000000000000000;
//00000111000000000000000000000000000000000111000000000000000000000000000000000011000000000000000000000;
//00000110000000000000000000000000000000000111000000000000000000000000000000000011000000000000000000000;
//00001110000000000000000000000000000000000110000000000000000000000000000000000110000000000000000000000;
//00001100000000000000000000000000000000000110000000000000000000000000000000000110000000000000000000000;
//00001000000000000000000000000000000000000100000000000000000000000000000000000100000000000000000000000;
//00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
//00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
//00001000000000000000000000000000000000001000000000000000000000000000000000000100000000000000000000000;
//00111100000000000000000000000000000000011100000000000000000000000000000000011100000000000000000000000;
//00111100000000000000000000000000000000111100000000000000000000000000000000011110000000000000000000000;
//00111110000000000000000000000000111000111110000000000000000000000000111100111111000000000000000000000;
//00001110000000000000000000000011110000001111000000000000000000000001111000001111000000000000000000000;
//00000111000000000000000000000111100000000111100000000000000000000011100000000011100000000000000000000;
//00000011100000000000000011111110000000000011110000000000000011111110000000000001110000000000000001111;
//00000001110000000000000111110000000000000000111000000000000111111000000000000000111000000000000111111;
//00000000111000000000011111000000000000000000011100000000001111000000000000000000011100000000001111100;
//00000000011100000000111100000000000000000000001110000000011110000000000000000000001110000000011110000;
//00000000001110000001111000000000000000000000000111000000111100000000000000000000000111000000111100000;
//00000000000111000011110000000000000000000000000011100011111000000000000000000000000011100001111000000;
//00000000000011111111100000000000000000000000000011111111100000000000000000000000000001111111110000000;
//00000000000001111111000000000000000000000000000001111111000000000000000000000000000000111111100000000;
//00000000000001111100000000000000000000000000000000111110000000000000000000000000000000111110000000000;
//00000000000001111110000000000000000000000000000000111111000000000000000000000000000000111111000000000;
//00000000000001111111000000000000000000000000000001111111100000000000000000000000000000111111100000000;
//00000000000011101111100000000000000000000000000001111111100000000000000000000000000001111111110000000;
//00000000000111100111110000000000000000000000000011100111110000000000000000000000000011110111111000000;
//00000000000111000011111000000000000000000000000111100011111100000000000000000000000111100001111100000;
//00000000001110000001111100000000000000000000001111000000111100000000000000000000001111000000111110000;
//00000000011100000000111100000000000000000000011110000000111110000000000000000000001110000000011110000;
//00000000111100000000011100000000000000000000111100000000011110000000000000000000011100000000001110000;
//00000001111000000000001000000000000000000001111000000000000000000000000000000000111100000000000100000;
//00000001110000000000000000000000000000000001110000000000000000000000000000000000111000000000000000000;
//00000001100000000000000000000000000000000001110000000000000000000000000000000000110000000000000000000;
//00000001100000000000000000000000000000000000100000000000000000000000000000000000110000000000000000000;
//]7   A:B   7
//Q[6]{0, 0.026, 0.812, 3.149, -0.733, -1.57}   A:B   Q[6]{0, 0.026, 0.812, -3.149, -0.733, 1.57}
//Q[6]{0.14063, 0.134212, 0.488862, 3.2778, -0.842921, -1.43832}   A:B   Q[6]{-0.0861828, 0.0933234, 0.615436, -3.2269, -0.797897, 1.48841}
//Q[6]{-0.488995, 1.13248, 0.423022, 3.14254, -0.0150743, -2.06004}   A:B   Q[6]{-0.444756, 0.365273, -0.197929, -3.55917, -1.0787, 1.1598}
//Q[6]{-0.24829, 1.00826, 0.188836, 3.27183, -0.27232, -1.82981}   A:B   Q[6]{-0.660299, 0.529145, -0.685578, -3.75887, -1.24602, 0.961099}
//Q[6]{0.0165492, 0.871585, -0.0688324, 3.41409, -0.555359, -1.57649}   A:B   Q[6]{-0.397232, 0.675133, -0.439783, -3.6191, -0.961823, 1.21379}
//Q[6]{0.285972, 0.732544, -0.330959, 3.55881, -0.843296, -1.31879}   A:B   Q[6]{-0.119408, 0.818507, -0.169302, -3.4697, -0.665026, 1.47908}
//Q[6]{-0.49, 1.133, 0.424, 3.142, -0.014, -2.061}   A:B   Q[6]{0.49, 1.133, 0.424, -3.142, -0.014, 2.061}
