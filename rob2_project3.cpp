//First project commit test b

#include "rob2_project3.hpp"

#include <QPushButton>
#include <RobWorkStudio.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include "RRTPlanner.h"
#include "PlannerTask.hpp"

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

    _btn0 = new QPushButton("Button0");
    pLayout->addWidget(_btn0, row++, 0);
    connect(_btn0, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn1 = new QPushButton("Button1");
    pLayout->addWidget(_btn1, row++, 0);
    connect(_btn1, SIGNAL(clicked()), this, SLOT(clickEvent()));


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



void SamplePlugin::clickEvent() {

	using namespace rw;
	using namespace common;
	using namespace models;
	using namespace pathplanning;

    QObject *obj = sender();
    if(obj == _btn0)
    {
        log().info() << "Button 0 pressed!\n";
        std::cout    << "Button 0 pressed!" << std::endl;
        rw::common::Ptr<RRTPlanner> planner = new RRTPlanner();
        std::list<PlannerTask> tasks;

        Ptr<WorkCell> workcell = _robWorkStudio->getWorkCell();
        Ptr<Device> device = workcell->getDevices()[0];

    	rw::proximity::CollisionStrategy::Ptr cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
    	CollisionDetector::Ptr collisionDetector = new CollisionDetector(workcell, cdstrategy);

    	rw::pathplanning::QConstraint::Ptr constraint = rw::pathplanning::QConstraint::make(
    			collisionDetector, device, workcell->getDefaultState());

    	Ptr<QSampler> cFree = QSampler::makeConstrained(QSampler::makeUniform(device), constraint);

        PlannerTask* task = new PlannerTask(device,constraint,cFree->sample(),cFree->sample());
        PlannerTask* task2 = new PlannerTask(device,constraint,cFree->sample(),cFree->sample());

        tasks.push_back(*task);
        tasks.push_back(*task2);


        planner->plan(tasks);
    } else if(obj == _btn1)
    {
        log().info() << "Button 1 pressed!\n";
        std::cout    << "Button 1 pressed!" << std::endl;
    }
}

Q_EXPORT_PLUGIN(SamplePlugin);
