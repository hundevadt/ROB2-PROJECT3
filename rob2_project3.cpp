//First project commit test

#include "rob2_project3.hpp"

#include <QPushButton>
#include <RobWorkStudio.hpp>

#include <rw/rw.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>

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

#include <iostream>
#include <fstream>
#include <time.h>

#include "RRTPlanner.h"

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;


SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginName", QIcon(":/pa_icon.png"))
{
	QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    int row = 0;

    _btn0 = new QPushButton("Scene collision check");
    pLayout->addWidget(_btn0, row++, 0);
    connect(_btn0, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn2 = new QPushButton("Robot collision check");
    pLayout->addWidget(_btn2, row++, 0);
    connect(_btn2, SIGNAL(clicked()), this, SLOT(clickEvent()));


    _pathComboBox = new QComboBox();
    pLayout->addWidget(_pathComboBox,row++,0);
    connect(_pathComboBox, SIGNAL(activated(int)), this, SLOT(clickEvent()));
    _pathComboBox->addItem("Path 0");
    _pathComboBox->addItem("Path 1");
    _pathComboBox->addItem("Path 2");
    _pathComboBox->addItem("Path 3");
    _pathComboBox->addItem("Path 4");
    _pathComboBox->addItem("Path 5");

    _btn1 = new QPushButton("Plan Path decoupled");
    pLayout->addWidget(_btn1, row++, 0);
    connect(_btn1, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn3 = new QPushButton("Plan Path centralized");
    pLayout->addWidget(_btn3, row++, 0);
    connect(_btn3, SIGNAL(clicked()), this, SLOT(clickEvent()));


    pLayout->setRowStretch(row,1);
}

SamplePlugin::~SamplePlugin(){ /* deallocate used memory */ }
void SamplePlugin::open(WorkCell* workcell){ /* do something when workcell is openned */}
void SamplePlugin::close() { /* do something when the workcell is closed */}

void SamplePlugin::initialize() {
    /* do something when plugin is initialized */
	_robWorkStudio = getRobWorkStudio();
	_robWorkStudio->stateChangedEvent().add(
            boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
	// load scene when initializing plugin
	loadScene("MultiKukaKr16/Scene.wc.xml");

	//initializing paths
		// q0 is first int the list
	std::size_t qSize = 6;




	//path 0
		//Robot A
	rw::math::Q *q_robotA0_1 = new rw::math::Q(qSize,-.5,0.,0.,0.,0.,0.);
	rw::math::Q *q_robotA0_2 = new rw::math::Q(qSize,.5,0.,0.,0.,0.,0.);
		//Robot B
	rw::math::Q *q_robotB0_1 = new rw::math::Q(qSize,-.5,0.,0.,0.,0.,0.);
	rw::math::Q *q_robotB0_2 = new rw::math::Q(qSize,.5,0.,0.,0.,0.,0.);

	_pathA0.push_back(*q_robotA0_1);
	_pathA0.push_back(*q_robotA0_2);

	_pathB0.push_back(*q_robotB0_1);
	_pathB0.push_back(*q_robotB0_2);





	//path 1
		//Robot A
	rw::math::Q *q_robotA1_1 = new rw::math::Q(qSize,1.,1.1,-.2,.5,.3,2.8);
	rw::math::Q *q_robotA1_2 = new rw::math::Q(qSize,.3,0.,-1,.4,.4,0.);
		//Robot B
	rw::math::Q *q_robotB1_1 = new rw::math::Q(qSize,1,-.8,-.4,-3.1,.7,-.3);
	rw::math::Q *q_robotB1_2 = new rw::math::Q(qSize,.689,.9,.9,0.,0.,0.);

	_pathA1.push_back(*q_robotA1_1);
	_pathA1.push_back(*q_robotA1_2);

	_pathB1.push_back(*q_robotB1_1);
	_pathB1.push_back(*q_robotB1_2);

	//path 2
		//Robot A
	rw::math::Q *q_robotA2_1 = new rw::math::Q(qSize,.4,.4,-.1,1.,1.,1.);
	rw::math::Q *q_robotA2_2 = new rw::math::Q(qSize,-.4,-.4,.1,0.,0.,0.);
		//Robot B
	rw::math::Q *q_robotB2_1 = new rw::math::Q(qSize,.4,-.4,.1,0.,0.,0.);
	rw::math::Q *q_robotB2_2 = new rw::math::Q(qSize,-.4,.4,-.1,1.,1.,1.);

	_pathA2.push_back(*q_robotA2_1);
	_pathA2.push_back(*q_robotA2_2);

	_pathB2.push_back(*q_robotB2_1);
	_pathB2.push_back(*q_robotB2_2);

	//path 3
		//Robot A
	rw::math::Q *q_robotA3_1 = new rw::math::Q(qSize,.333,.85,-.454,3.142,-1.175,-1.238);
	rw::math::Q *q_robotA3_2 = new rw::math::Q(qSize,-.571,.199,.609,3.142,-.763,-2.142);
		//Robot B
	rw::math::Q *q_robotB3_1 = new rw::math::Q(qSize,.571,.199,.609,-3.142,-.763,2.142);
	rw::math::Q *q_robotB3_2 = new rw::math::Q(qSize,-.333,.85,-.454,-3.142,-1.175,1.238);

	_pathA3.push_back(*q_robotA3_1);
	_pathA3.push_back(*q_robotA3_2);

	_pathB3.push_back(*q_robotB3_1);
	_pathB3.push_back(*q_robotB3_2);

	//path 4
		//Robot A
	rw::math::Q *q_robotA4_1 = new rw::math::Q(qSize,-.49,1.133,.424,3.142,-.014,-2.061);
	rw::math::Q *q_robotA4_2 = new rw::math::Q(qSize,.668,.535,-.703,3.766,-1.252,-.954);
	rw::math::Q *q_robotA4_3 = new rw::math::Q(qSize,0.,.026,.812,3.149,-.733,-1.57);
		//Robot B
	rw::math::Q *q_robotB4_1 = new rw::math::Q(qSize,.49,1.133,.424,-3.142,-.014,2.061);
	rw::math::Q *q_robotB4_2 = new rw::math::Q(qSize,-.668,.535,-.703,-3.766,-1.252,.954);
	rw::math::Q *q_robotB4_3 = new rw::math::Q(qSize,0.,.026,.812,-3.149,-.733,1.57);

	_pathA4.push_back(*q_robotA4_1);
	_pathA4.push_back(*q_robotA4_2);
	_pathA4.push_back(*q_robotA4_3);

	_pathB4.push_back(*q_robotB4_1);
	_pathB4.push_back(*q_robotB4_2);
	_pathB4.push_back(*q_robotB4_3);

	//path 5
		//Robot A
	rw::math::Q *q_robotA5_1 = new rw::math::Q(qSize,0,.026,.812,3.149,-.733,-1.57);
	rw::math::Q *q_robotA5_2 = new rw::math::Q(qSize,-.257,1.234,-.157,-3.142,-.494,1.316);
	rw::math::Q *q_robotA5_3 = new rw::math::Q(qSize,.668,.535,-.703,3.766,-1.252,-.954);
	rw::math::Q *q_robotA5_4 = new rw::math::Q(qSize,0,.026,.812,3.149,-.733,-1.57);
		//Robot B
	rw::math::Q *q_robotB5_1 = new rw::math::Q(qSize,0,.026,.812,3.149,-.733,-1.57);
	rw::math::Q *q_robotB5_2 = new rw::math::Q(qSize,.257,1.234,-.157,3.142,-.494,-1.316);
	rw::math::Q *q_robotB5_3 = new rw::math::Q(qSize,-.668,.535,-.703,-3.766,-1.252,.954);
	rw::math::Q *q_robotB5_4 = new rw::math::Q(qSize,0,.026,.812,3.149,-.733,-1.57);

	_pathA5.push_back(*q_robotA5_1);
	_pathA5.push_back(*q_robotA5_2);
	_pathA5.push_back(*q_robotA5_3);
	_pathA5.push_back(*q_robotA5_4);

	_pathB5.push_back(*q_robotB5_1);
	_pathB5.push_back(*q_robotB5_2);
	_pathB5.push_back(*q_robotB5_3);
	_pathB5.push_back(*q_robotB5_4);

	_currentPathA = _pathA1;
	_currentPathB = _pathB1;
}

void SamplePlugin::stateChangedListener(const State& state) {
    log().info() << "State changed!";
}

void SamplePlugin::loadScene(std::string scene)
{
	WorkCell::Ptr workCell = WorkCellLoader::load(scene);
	_robWorkStudio->setWorkcell(workCell);
	std::cout	<< "Workcell " << *workCell << "successfully loaded." << std::endl;
}

void SamplePlugin::collisionCheck()
{
	WorkCell::Ptr workCell = _robWorkStudio->getWorkCell();

	rw::models::Device::Ptr device = _robWorkStudio->getWorkcell()->getDevices().at(0);

	std::cout << _robWorkStudio->getWorkCell()->getDevices().at(0)->getName() << std::endl;
	std::cout << _robWorkStudio->getWorkCell()->getDevices().at(1)->getName() << std::endl;

	CollisionStrategy::Ptr cdstrategy = ProximityStrategyFactory::makeCollisionStrategy("PQP");

	ProximityFilterStrategy::Ptr filter = new BasicFilterStrategy(workCell);
	filter->addRule(ProximitySetupRule::makeExclude("KukaKr16A.*","KukaKr16B.*"));

	CollisionDetector::Ptr collisionDetector = new CollisionDetector(workCell, cdstrategy,filter);

	QConstraint::Ptr constraint = QConstraint::make(
			collisionDetector, device, _robWorkStudio->getState());

	if(constraint->inCollision(device->getQ(_robWorkStudio->getState())))
		std::cout << "in collision" << std::endl;
	else
		std::cout << "not in collision" << std::endl;
}

void SamplePlugin::selectPath(int pathNumber){
	switch(pathNumber){
	case 0 :
		_currentPathA = _pathA0;
		_currentPathB = _pathB0;
//    		std::cout << "1" << "  " << _currentPath << std::endl;
		break;
	case 1 :
		_currentPathA = _pathA1;
		_currentPathB = _pathB1;
//    		std::cout << "2" << "  " << _currentPath << std::endl;
		break;
	case 2 :
		_currentPathA = _pathA2;
		_currentPathB = _pathB2;
//    		std::cout << "3" << "  " << _currentPath << std::endl;
		break;
	case 3 :
		_currentPathA = _pathA3;
		_currentPathB = _pathB3;
//    		std::cout << "4" << "  " << _currentPath << std::endl;
		break;
	case 4 :
		_currentPathA = _pathA4;
		_currentPathB = _pathB4;
//    		std::cout << "5" << "  " << _currentPath << std::endl;
	case 5 :
		_currentPathA = _pathA5;
		_currentPathB = _pathB5;
//    		std::cout << "5" << "  " << _currentPath << std::endl;

		break;
	default :
		break;
	}

	//show path
	rw::models::Device::Ptr deviceA;
	rw::models::Device::Ptr deviceB;
	rw::kinematics::State state;

	state = _robWorkStudio->getState();


	deviceA = _robWorkStudio->getWorkcell()->getDevices().at(0);
	deviceB = _robWorkStudio->getWorkcell()->getDevices().at(1);


	_robWorkStudio->setTimedStatePath(TimedUtil::makeTimedStatePath(
			*_robWorkStudio->getWorkcell(),rw::models::Models::getStatePath(*deviceA, _currentPathA, state)));

	state = _robWorkStudio->getState();

	_robWorkStudio->setTimedStatePath(TimedUtil::makeTimedStatePath(
			*_robWorkStudio->getWorkcell(),rw::models::Models::getStatePath(*deviceB, _currentPathB, state)));
}


void SamplePlugin::planPath() {

}

void SamplePlugin::robotCC(){


	std::size_t qSize = 6;

	Ptr<Device> deviceA = _robWorkStudio->getWorkCell()->getDevices()[0];
	Ptr<Device> deviceB = _robWorkStudio->getWorkCell()->getDevices()[1];

	rw::math::Q *qA = new rw::math::Q(qSize,0,0.,0.,0.,0.,0.);
	rw::math::Q *qB = new rw::math::Q(qSize,0.,0.,0.,0.,0.,0.);


	kinematics::State state = _robWorkStudio->getWorkCell()->getDefaultState();


	deviceA->setQ(*qA,state);
	deviceB->setQ(*qB,state);



using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;

    CollisionDetector detector(
    		_robWorkStudio->getWorkCell(), ProximityStrategyYaobi::make());

    const bool collision =
        detector.inCollision(state);

    std::cout
        << "Workcell "
        << _robWorkStudio->getWorkCell()
        << " is in collision in its initial state: "
        << collision
        << std::endl;
}

void SamplePlugin::clickEvent() {
    QObject *obj = sender();
    if(obj == _btn0){
        collisionCheck();
    } else if(obj == _btn2){
    	robotCC();
    } else if(obj == _btn3){
    	centralizedPlan();
    } else if(obj == _btn1){
    	decoupledPlan();
    } else if(obj == _pathComboBox){
    	selectPath(_pathComboBox->currentIndex());
    }
}


void SamplePlugin::centralizedPlan()
{
//   log().info() << "Button 0 pressed!\n";
//   std::cout    << "Button 0 pressed!" << std::endl;

	rw::common::Ptr<RRTPlanner> planner = new RRTPlanner();
	std::list<Ptr<PlannerTask> > tasks;

	Ptr<WorkCell> workcell = _robWorkStudio->getWorkCell();
	Ptr<Device> device1 = workcell->getDevices()[0];
	Ptr<Device> device2 = workcell->getDevices()[1];

	rw::proximity::CollisionStrategy::Ptr cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");

	CollisionDetector::Ptr collisionDetector = new CollisionDetector(workcell, cdstrategy);

	rw::pathplanning::QConstraint::Ptr constraint1 = rw::pathplanning::QConstraint::make(
			collisionDetector, device1, workcell->getDefaultState());

	rw::pathplanning::QConstraint::Ptr constraint2 = rw::pathplanning::QConstraint::make(
			collisionDetector, device2, workcell->getDefaultState());

	Math::seed();

	Ptr<QSampler> cFree1 = QSampler::makeConstrained(QSampler::makeUniform(device1), constraint1);
	Ptr<QSampler> cFree2 = QSampler::makeConstrained(QSampler::makeUniform(device2), constraint2);

   Ptr<PlannerTask> task = new PlannerTask(device1,constraint1,cFree1->sample(),cFree1->sample());
   Ptr<PlannerTask> task2 = new PlannerTask(device2,constraint2,cFree2->sample(),cFree2->sample());

   tasks.push_back(task);
   tasks.push_back(task2);


   planner->plan(tasks);

   std::cout << task->getPath().size() << std::endl;
   for(int i = 0; i < task->getPath().size() ; i++)
   	std::cout << task->getPath().at(i) << std::endl;

   TimedStatePath timedStatePath;

   double time = 0.0;

   kinematics::State state = _robWorkStudio->getWorkCell()->getDefaultState();
   for(int i = 0; i < task->getPath().size() ; i++)
   {
   	device1->setQ(task->getPath().at(i),state);
   	device2->setQ(task2->getPath().at(i),state);
   	if(i != 0)
   		time += ((Q)(task->getPath().at(i) - task->getPath().at(i-1))).norm2();

   	timedStatePath.push_back(*(new TimedState(time,state)));
   }

   _robWorkStudio->setTimedStatePath(timedStatePath);
}

void SamplePlugin::decoupledPlan()
{

	rw::common::Ptr<decoupledRRTPlanner> decoupledPlanner = new decoupledRRTPlanner(_robWorkStudio);
	std::list<Ptr<PlannerTask> > tasks;

	Ptr<WorkCell> workcell = _robWorkStudio->getWorkCell();
	Ptr<Device> deviceA = workcell->getDevices()[0];
	Ptr<Device> deviceB = workcell->getDevices()[1];

	rw::proximity::CollisionStrategy::Ptr cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");

	rw::proximity::ProximityFilterStrategy::Ptr filter = new rw::proximity::BasicFilterStrategy(workcell);
	filter->addRule(rw::proximity::ProximitySetupRule::makeExclude("KukaKr16A.*","KukaKr16B.*"));

	CollisionDetector::Ptr collisionDetector = new CollisionDetector(workcell, cdstrategy,filter);

	rw::pathplanning::QConstraint::Ptr constraintA = rw::pathplanning::QConstraint::make(
			collisionDetector, deviceA, workcell->getDefaultState());

	rw::pathplanning::QConstraint::Ptr constraintB = rw::pathplanning::QConstraint::make(
			collisionDetector, deviceB, workcell->getDefaultState());

	Ptr<PlannerTask> taskA = new PlannerTask(deviceA,constraintA,_currentPathA);
	Ptr<PlannerTask> taskB = new PlannerTask(deviceB,constraintB,_currentPathB);

   tasks.push_back(taskA);
   tasks.push_back(taskB);

   decoupledPlanner->plan(tasks);


//   std::cout << taskA->getPath().size() << "   A:B   " << taskB->getPath().size() << std::endl;
//   for(int i = 0; i < taskA->getPath().size() ; i++)
//   {
//	   std::cout << taskA->getPath().at(i) << "   A:B   " << taskB->getPath().at(i) << std::endl;
//   }

   TimedStatePath timedStatePath;

   double time = 0.0;

   kinematics::State state = _robWorkStudio->getWorkCell()->getDefaultState();
   for(int i = 0; i < taskA->getPath().size() ; i++)
   {
   	deviceA->setQ(taskA->getPath().at(i),state);
//   	deviceB->setQ(taskB->getPath().at(i),state);
   	if(i != 0)
   		time += ((Q)(taskA->getPath().at(i) - taskA->getPath().at(i-1))).norm2();

   	timedStatePath.push_back(*(new TimedState(time,state)));
   }

   _robWorkStudio->setTimedStatePath(timedStatePath);




	//old implementation
//
//	std::cout << "plan path" << std::endl;
//    decoupledRRTPlanner *planner = new decoupledRRTPlanner(_robWorkStudio,_currentPathA, _currentPathB);
////  rw::trajectory::QPath[] resultPaths = planner->plan();
//    rw::trajectory::QPath path;
//    path = planner->plan();
//
//
//	//show path
//	rw::models::Device::Ptr deviceA;
//	rw::models::Device::Ptr deviceB;
//	rw::kinematics::State state;
//
//	state = _robWorkStudio->getState();
//
//
//	deviceA = _robWorkStudio->getWorkcell()->getDevices().at(0);
//	deviceB = _robWorkStudio->getWorkcell()->getDevices().at(1);
//
//
//	_robWorkStudio->setTimedStatePath(TimedUtil::makeTimedStatePath(
//			*_robWorkStudio->getWorkcell(),rw::models::Models::getStatePath(*deviceB, path, state)));

}


Q_EXPORT_PLUGIN(SamplePlugin);
