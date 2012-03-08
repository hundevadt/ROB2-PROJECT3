#include "rob2_project3.hpp"

#include <QPushButton>
#include <RobWorkStudio.hpp>

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
    QObject *obj = sender();
    if(obj == _btn0){
        log().info() << "Button 0 pressed!\n";
        std::cout    << "Button 0 pressed!" << std::endl;
    } else if(obj == _btn1){
        log().info() << "Button 1 pressed!\n";
        std::cout    << "Button 1 pressed!" << std::endl;
    }
}

Q_EXPORT_PLUGIN(SamplePlugin);
