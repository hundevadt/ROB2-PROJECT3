/*
 * decoupledRRTPlanner.h
 *
 *  Created on: Mar 7, 2012
 *      Author: skytthe
 */

#ifndef DECOUPLEDRRTPLANNER_H_
#define DECOUPLEDRRTPLANNER_H_

class decoupledRRTPlanner {
public:
	decoupledRRTPlanner();
	virtual ~decoupledRRTPlanner();

	rw::trajectory::QPath plan(rw::math::Q qAInit, rw::math::Q qAGoal,
			rw::math::Q qBInit, rw::math::Q qBGoal);
};

#endif /* DECOUPLEDRRTPLANNER_H_ */
