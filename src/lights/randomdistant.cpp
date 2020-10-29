
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

// lights/randomdistant.cpp*
#include "lights/randomdistant.h"
#include "paramset.h"
#include "sampling.h"
#include "stats.h"

namespace pbrt {

// RandomDistantLight Method Definitions
RandomDistantLight::RandomDistantLight(const Transform &LightToWorld, const Spectrum &L)
    : Light((int)LightFlags::DeltaDirection, LightToWorld, MediumInterface()),
      L(L) {}

void RandomDistantLight::Shuffle(RNG &rng) {
    // WZR: randomlized direction
    w = UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
}

void RandomDistantLight::getDSInfo(Vector3f *wlight) { *wlight = w; }

Spectrum RandomDistantLight::Sample_Li(const Interaction &ref, const Point2f &u,
                                 Vector3f *wi, Float *pdf,
                                 VisibilityTester *vis) const {
    ProfilePhase _(Prof::LightSample);
    *wi = w;
    *pdf = 1;
    Point3f pOutside = ref.p + w * (2 * worldRadius);
    *vis =
        VisibilityTester(ref, Interaction(pOutside, ref.time, mediumInterface));
    return L;
}

Spectrum RandomDistantLight::Power() const {
    return L * Pi * worldRadius * worldRadius;
}

Float RandomDistantLight::Pdf_Li(const Interaction &, const Vector3f &) const {
    return 0.f;
}

Spectrum RandomDistantLight::Sample_Le(const Point2f &u1, const Point2f &u2,
                                 Float time, Ray *ray, Normal3f *nLight,
                                 Float *pdfPos, Float *pdfDir) const {
    ProfilePhase _(Prof::LightSample);
    // WZR: 
    std::cout << "Does not function for bdpt and sppm integrators "
                 "because the randomlized light direction"
              << std::endl;
    exit(1);
    return L;
}

void RandomDistantLight::Pdf_Le(const Ray &, const Normal3f &, Float *pdfPos,
                          Float *pdfDir) const {
    ProfilePhase _(Prof::LightPdf);
    *pdfPos = 1 / (Pi * worldRadius * worldRadius);
    *pdfDir = 0;
}

std::shared_ptr<RandomDistantLight> CreateRandomDistantLight(const Transform &light2world,
                                                 const ParamSet &paramSet) {
    Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    return std::make_shared<RandomDistantLight>(light2world, L * sc);
}

}  // namespace pbrt
