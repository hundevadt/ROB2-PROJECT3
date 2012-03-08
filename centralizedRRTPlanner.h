/*
 * centralizedRRTPlanner.h
 *
 *  Created on: Mar 7, 2012
 *      Author: skytthe
 */

#ifndef CENTRALIZEDRRTPLANNER_H_
#define CENTRALIZEDRRTPLANNER_H_

class centralizedRRTPlanner {
public:
	centralizedRRTPlanner();
	virtual ~centralizedRRTPlanner();

	rw::trajectory::QPath plan(rw::math::Q qAInit, rw::math::Q qAGoal,
			rw::math::Q qBInit, rw::math::Q qBGoal);
};

#endif /* CENTRALIZEDRRTPLANNER_H_ */
