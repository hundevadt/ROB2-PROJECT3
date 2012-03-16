/*
 * RRT.cpp
 *
 *  Created on: Feb 23, 2012
 *      Author: daniel
 */

#include <rw/math/Q.hpp>

#include "RRT.h"


RRT::RRT(){
}

RRT::RRT(rw::common::Ptr<RRTNode> node){
	addNodeToTree(node);
}

RRT::~RRT() {

}

rw::common::Ptr<RRTNode> RRT::getClosestNode(rw::math::Q node) {

	rw::common::Ptr<RRTNode> closestNode = NULL;
	rw::math::Q tempQ;

	double distanceToNode = std::numeric_limits<double>::max();

	std::list<rw::common::Ptr<RRTNode> >::iterator itt;
	for (itt = _tree.begin(); itt != _tree.end(); itt++) {

		tempQ = (*itt)->getValue();

		if((tempQ - node).norm2() < distanceToNode){
			distanceToNode = (tempQ - node).norm2();
			closestNode = *itt;
		}
	}

	return closestNode;
}

std::list<rw::common::Ptr<RRTNode> > RRT::getListOfNodes(){
	return _tree;
}

bool RRT::addNodeToTree(rw::common::Ptr<RRTNode> newNode) {
	_tree.push_back(newNode);

//	std::cout << newNode->getValue();

	newNode->addTreeToNode(this);

	return !(_tree.empty());
}
