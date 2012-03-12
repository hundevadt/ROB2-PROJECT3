/*
 * RRT.h
 *
 *  Created on: Feb 23, 2012
 *      Author: daniel test
 */

#ifndef RRT_H_
#define RRT_H_

#include "RRTNode.h"
#include <rw/math.hpp>
#include <rw/common.hpp>
#include <limits>

#include <list>

class RRTNode;

class RRT {
private:
	std::list<rw::common::Ptr<RRTNode> > _tree;
public:
	RRT();
	RRT(rw::common::Ptr<RRTNode> node);
	virtual ~RRT();

	bool addNodeToTree(rw::common::Ptr<RRTNode>);
	rw::common::Ptr<RRTNode> getClosestNode(rw::math::Q node);

	std::list<rw::common::Ptr<RRTNode> > getListOfNodes();

};

#endif /* RRT_H_ */
