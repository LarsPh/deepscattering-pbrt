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
#include "shapes/distanceestimator.h"
#include "efloat.h"
#include "paramset.h"
#include "sampling.h"
#include "stats.h"

namespace pbrt {

// DistanceEstimator Method Definitions

Vector3f DistanceEstimator::CalculateNormal(
    const Point3f &pos, float eps, const Vector3f &defaultNormal) const {
    const Vector3f v1 = Vector3f(1.0, -1.0, -1.0);
    const Vector3f v2 = Vector3f(-1.0, -1.0, 1.0);
    const Vector3f v3 = Vector3f(-1.0, 1.0, -1.0);
    const Vector3f v4 = Vector3f(1.0, 1.0, 1.0);

    const Vector3f normal =
        v1 * Evaluate(pos + v1 * eps) + v2 * Evaluate(pos + v2 * eps) +
        v3 * Evaluate(pos + v3 * eps) + v4 * Evaluate(pos + v4 * eps);
    const Float length = normal.Length();
	
    return length > 0 ? (normal / length) : defaultNormal;
}

bool DistanceEstimator::Intersect(const Ray &r, Float *tHit,
                                  SurfaceInteraction *isect,
                                  bool testAlphaTexture) const {
    Point3f o = r.o;
    Vector3f d = r.d / r.d.Length();
    Float t = r.d.Length();
    Float distance = Evaluate(o + t * d);
    int iters = 0;

    while (true) {
        ++iters;
        // update t
        t += distance;
		// check
        if (t >= r.tMax || iters > maxIters) return false;
        // Did it hit?
        distance = Evaluate(o + t * d);
        if (distance < hitEpsilon * t) break;
    }

    // Update _tHit_
    if (tHit != nullptr) *tHit = t;

	// Update _isect_
    // Compute sphere hit position and pErr
    Point3f pHit = (Point3f)o + t * d;
    Vector3f pErr = (Vector3f)pHit * rayEpsilonMultiplier;
	// Compute normal and decomposite it
    Vector3f normal = CalculateNormal(pHit, normalEpsilon, -d);
    Vector3f dpdu, dpdv;
    CoordinateSystem(normal, &dpdu, &dpdv);
	
    // Initialize _SurfaceInteraction_ from parametric information
    if (isect != nullptr)
        *isect = SurfaceInteraction(pHit, pErr, Point2f(0,0), -d,
                               dpdu, dpdv,
							   Normal3f(0, 0, 0), Normal3f(0, 0, 0),   // degenerated uv related derivatives
							   r.time, this); 
    return true;
}

bool DistanceEstimator::IntersectP(const Ray &r, bool testAlphaTexture) const {
    return Intersect(r, nullptr, nullptr, testAlphaTexture);
}

Interaction DistanceEstimator::Sample(const Point2f &u, Float *pdf) const {
    LOG(FATAL) << "DistanceEstimator::Sample not implemented.";
    return Interaction();
}

}  // namespace pbrt
