#include "MotionState.h"


MotionState::MotionState():
inited(false),
errType(0)/*,
degreeR(-100),
degreeT(-100)*/
{
}


MotionState::~MotionState()
{
}

std::ostream& operator<<(std::ostream& out, const MotionState& ms) {
	out << "MotionState[" << ms.idxImg[0] << "-" << ms.idxImg[1] << "]" << std::endl;
	out << "matR: " << ms.matR << std::endl;
	out << "matT: " << ms.matT << std::endl;
	out << "scale: " << ms.scale << " ; " << 1.65f / ms.scale << std::endl;
	out << "pair: " << ms.mapPairPoints.size() << std::endl;
	return out;
}

double MotionState::getDegree(const std::string& str) {
	double ret = 0.0f;
	
	switch (str[0]) {
	case 'R':case 'r': {
				 //计算 matRix吧[0,0,1]^T旋转了多少度
				 ret = Utils::getRodriguesRotation(matR * Const::mat31_001, cv::Mat());
	}	break;
	case 'T':case 't': {
				 ret = Utils::getRodriguesRotation(matT, cv::Mat());
	}   break;
	default: 
		throw std::exception("错误调用", 1);
	}
	return ret;
}