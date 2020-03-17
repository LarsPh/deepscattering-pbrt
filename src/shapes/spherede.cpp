/* Extension on pbrt made by Zhaorong Wang */

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

// shapes/distanceestimator.cpp*
#include "efloat.h"
#include "paramset.h"
#include "sampling.h"
#include "shapes/spherede.h"
#include "stats.h"

namespace pbrt {
	Bounds3f SphereDE::ObjectBound() const {
		return Bounds3f(Point3f(-radius, -radius, -radius),
						Point3f(radius, radius, radius));
	}

	Float SphereDE::Area() const {
		return Float(4.0 * Pi * radius * radius);
	}

	Float SphereDE::Evaluate(const Point3f &p) const {
            return (Float)abs(Vector3f(p).Length() - radius);
	}

std::shared_ptr<Shape> CreateSphereDEShape(
            const Transform *o2w, const Transform *w2o, bool reverseOrientation,
            const ParamSet &params) {
            int maxIters = params.FindOneInt("maxiters", 1000);
            Float hitEpsilon = params.FindOneFloat("hitepsilon", 10e-5f);
            Float rayEpsilonMultiplier =
                params.FindOneFloat("rayepsilonmultiplier", 1000.0);
            Float normalEpsilon = params.FindOneFloat("normalepsilon", 1e-6f);
            Float radius = params.FindOneFloat("radius", 1);
            return std::make_shared<SphereDE>(
                o2w, w2o, reverseOrientation, maxIters, hitEpsilon,
                rayEpsilonMultiplier, normalEpsilon, radius);
        }
}  // namespace pbrt