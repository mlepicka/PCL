/*!
 * \file
 * \brief
 * \author Mort
 */

#include <memory>
#include <string>

#include "CvGaussianNoise.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CvGaussianNoise {

CvGaussianNoise::CvGaussianNoise(const std::string & name) :
		Base::Component(name)  {

}

CvGaussianNoise::~CvGaussianNoise() {
}

void CvGaussianNoise::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register handlers

}

bool CvGaussianNoise::onInit() {

	return true;
}

bool CvGaussianNoise::onFinish() {
	return true;
}

bool CvGaussianNoise::onStop() {
	return true;
}

bool CvGaussianNoise::onStart() {
	return true;
}



} //: namespace CvGaussianNoise
} //: namespace Processors
