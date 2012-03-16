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
    void selectPath(int pathNumber);
	void planPath();
    void collisionCheck();
    void robotCC();

    void centralizedPlan();
    bool decoupledPlan();


private slots:
    void clickEvent();

    void stateChangedListener(const rw::kinematics::State& state);
private:
    rws::RobWorkStudio* _robWorkStudio;

    QPushButton* _btn0,*_btn1,*_btn2,*_btn3;
    QComboBox *_pathComboBox;

    rw::trajectory::QPath  _currentPathA, _resultPathA, _pathA0, _pathA1, _pathA2, _pathA3, _pathA4, _pathA5;
    rw::trajectory::QPath  _currentPathB, _resultPathB, _pathB0, _pathB1, _pathB2, _pathB3, _pathB4, _pathB5;
};

#endif /*SAMPLEPLUGIN_HPP*/
