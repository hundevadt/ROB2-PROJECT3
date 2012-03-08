#ifndef rob2_project3_HPP
#define rob2_project3_HPP

#include <rws/RobWorkStudio.hpp>
#include <rw/rw.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

#include "centralizedRRTPlanner.h"
#include "decoupledRRTPlanner.h"

class SamplePlugin: public rws::RobWorkStudioPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
    SamplePlugin();
	virtual ~SamplePlugin();

	// functions inherited from RobworkStudioPlugin, are typically used but can be optional
    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();

    void loadScene(std::string scene);

private slots:
    void clickEvent();

    void stateChangedListener(const rw::kinematics::State& state);
private:
    rws::RobWorkStudio* _robWorkStudio;

    QPushButton* _btn0,*_btn1;
};

#endif /*SAMPLEPLUGIN_HPP*/
