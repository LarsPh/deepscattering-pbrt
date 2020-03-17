/* Expensions on pbrt made by Zhaorong Wang */

/*
	pbrt source code is Copyright(c) 1998-2016
						Matt Pharr, Greg Humphreys, and Wenzel Jakob.

	This file is part of pbrt.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are
	met:

	- Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.

	- Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
	IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
	TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
	PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
	HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_MANDELBULBDE_H
#define PBRT_SHAPES_MANDELBULBDE_H

// shapes/mandelbulbde.h*

#include "distanceestimator.h"

namespace pbrt {
class MandelbulbDE : public DistanceEstimator 
{
  public:
    // MandelbulbDE Public Methods
    MandelbulbDE(const Transform *ObjectToWorld,
             const Transform *WorldToObject, bool reverseOrientation,
             int maxIters, Float hitEpsilon,
             Float rayEpsilonMultiplier, Float normalEpsilon,
             int fractalIters, int mandelbulbPower)
        : DistanceEstimator(ObjectToWorld, WorldToObject, reverseOrientation,
                            maxIters, hitEpsilon, rayEpsilonMultiplier, normalEpsilon),
        fractalIters(fractalIters),
        mandelbulbPower(mandelbulbPower) {}
    Bounds3f ObjectBound() const;
    Float Area() const;
    Float Evaluate(const Point3f &p) const;

  private:
    const int fractalIters;
    const int mandelbulbPower;
};

std::shared_ptr<Shape> CreateMandelbulbDEShape(const Transform *o2w,
                                                    const Transform *w2o,
                                                    bool reverseOrientation,
                                                    const ParamSet &params);

}

#endif  // PBRT_SHAPES_MANDELBULBDE_H