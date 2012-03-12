/*
 * RRTNode.cpp
 *
 *  Created on: Feb 23, 2012
 *      Author: Daniel
 */

#include "RRTNode.h"

RRTNode::RRTNode() :
	_parrent(NULL)
{

}

RRTNode::RRTNode(rw::math::Q value, rw::common::Ptr<RRTNode> parrent) :
	_value(value),
	_parrent(parrent)
{

}


rw::common::Ptr<RRTNode> RRTNode::getParrent() const
{
    return _parrent;
}

void RRTNode::addTreeToNode(rw::common::Ptr<RRT> tree){
	_tree = tree;
}

rw::common::Ptr<RRT> RRTNode::getTree(){
	return  _tree;
}

rw::math::Q RRTNode::getValue() const
{
    return _value;
}

void RRTNode::setParrent(rw::common::Ptr<RRTNode> parrent)
{
    this->_parrent = parrent;
}

void RRTNode::setValue(rw::math::Q value)
{
    this->_value = value;
}
