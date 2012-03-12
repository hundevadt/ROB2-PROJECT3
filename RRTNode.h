/*
 * RRTNode.h
 *
 *  Created on: Feb 23, 2012
 *      Author: daniel
 */


#ifndef RRTNODE_H_
#define RRTNODE_H_

#include <rw/math.hpp>
#include <rw/math/Q.hpp>
#include "RRT.h"

class RRT;
class RRTNode {
private:

	rw::common::Ptr<RRT> _tree;
	rw::common::Ptr<RRTNode> _parrent;
	rw::math::Q _value;
public:
	RRTNode();
	RRTNode(rw::math::Q value, rw::common::Ptr<RRTNode> parrent);

    rw::common::Ptr<RRTNode> getParrent() const;
    rw::math::Q getValue() const;

    void setParrent(rw::common::Ptr<RRTNode> parrent);
    void setValue(rw::math::Q value);

    rw::common::Ptr<RRT> getTree();

    void addTreeToNode(rw::common::Ptr<RRT> tree);
};

#endif /* RRTNODE_H_ */
