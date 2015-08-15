#include "MotionState.h"


MotionState::MotionState():
inited(false),
degreeR(-100),
degreeT(-100)
{
}


MotionState::~MotionState()
{
}

std::ostream& operator<<(std::ostream& out, const MotionState& ms) {
	out << "MotionState[" << ms.idxImg[0] << "-" << ms.idxImg[1] << "]" << std::endl;
	out << "matR: " << ms.matR << std::endl;
	out << "matT: " << ms.matT << std::endl;
	out << "sclae: " << ms.scale << " ; " << 1.65f / ms.scale << std::endl;
	out << "pair: " << ms.mapPairPoints.size() << std::endl;
	return out;
}

