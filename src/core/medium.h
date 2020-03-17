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

#ifndef PBRT_CORE_MEDIUM_H
#define PBRT_CORE_MEDIUM_H

// core/medium.h*
#include <memory>
#include <unordered_map>

#include "geometry.h"
#include "pbrt.h"
#include "spectrum.h"

// WZR:
#include "ext/interpolation.h"

namespace pbrt {
typedef std::unordered_map<int, Float> unoMap;

// Media Declarations
class PhaseFunction {
  public:
    // PhaseFunction Interface
    virtual ~PhaseFunction();
    virtual Float p(const Vector3f &wo, const Vector3f &wi) const = 0;
    virtual Float Sample_p(const Vector3f &wo, Vector3f *wi,
                           const Point2f &u) const = 0;
    virtual std::string ToString() const = 0;
};

inline std::ostream &operator<<(std::ostream &os, const PhaseFunction &p) {
    os << p.ToString();
    return os;
}

bool GetMediumScatteringProperties(const std::string &name, Spectrum *sigma_a,
                                   Spectrum *sigma_s);

// WZR: Maths Inline Functions
inline Float ToDegrees(Float rad) { return rad / 180.0 * Pi; }

inline Float ToRad(Float degrees) { return degrees * 180.0 / Pi; }

// Media Inline Functions
inline Float PhaseHG(Float cosTheta, Float g) {
    Float denom = 1 + g * g + 2 * g * cosTheta;
    return Inv4Pi * (1 - g * g) / (denom * std::sqrt(denom));
}

inline Float BetterAcos(Float cos) {
    if (cos <= -1.0) return Pi;
    if (cos >= 1.0) return 0;
    return acos(cos);
}

// Medium Declarations
class Medium {
  public:
    // Medium Interface
    virtual ~Medium() {}
    virtual Spectrum Tr(const Ray &ray, Sampler &sampler) const = 0;
    virtual Spectrum Sample(const Ray &ray, Sampler &sampler,
                            MemoryArena &arena,
                            MediumInteraction *mi) const = 0;
};

// HenyeyGreenstein Declarations
class HenyeyGreenstein : public PhaseFunction {
  public:
    // HenyeyGreenstein Public Methods
    HenyeyGreenstein(Float g) : g(g) {}
    Float p(const Vector3f &wo, const Vector3f &wi) const;
    Float Sample_p(const Vector3f &wo, Vector3f *wi,
                   const Point2f &sample) const;
    std::string ToString() const {
        return StringPrintf("[ HenyeyGreenstein g: %f ]", g);
    }

  private:
    const Float g;
};

// WZR: CloudMie Declarations

// Comparing in precision 2

/*
struct RoundedHash {
    std::size_t operator()(const Float &f) const {
        return std::hash<float>()(round(f * 100) / 100);
    }
};

struct RoundedEqual {
    bool operator()(const Float &lhs, const Float &rhs) const {
        return round(lhs * 100) / 100 == round(rhs * 100) / 100;
    }
};
*/

class CloudMie : public PhaseFunction {
  public:
    // CloudMie Public Methods
    Float p(const Vector3f &wo, const Vector3f &wi) const;
    Float Sample_p(const Vector3f &wo, Vector3f *wi,
                   const Point2f &sample) const;
    std::string ToString() const {
        return StringPrintf("[ Mie parameters: TODO ]");
    }
    static void createCerp();
    //static void initializeCDF();
    //static unoMap inversedCDF4;
    //static unoMap inversedCDF8;
    //static unoMap inversedCDF9;
    //static unoMap inversedCDFe;

  private:
    
    static alglib::spline1dinterpolant CerpPhase;
    static alglib::spline1dinterpolant CerpInv;
    static alglib::real_1d_array MiePhase;
    static alglib::real_1d_array CDF;
    static alglib::real_1d_array Theta;

    Float phaseMie(Float degree) const;
    /*Float evaluate(Float u0, Float precision, unoMap &inversedCDF, Float r1,
                   Float r2) const;*/
};

// MediumInterface Declarations
struct MediumInterface {
    MediumInterface() : inside(nullptr), outside(nullptr) {}
    // MediumInterface Public Methods
    MediumInterface(const Medium *medium) : inside(medium), outside(medium) {}
    MediumInterface(const Medium *inside, const Medium *outside)
        : inside(inside), outside(outside) {}
    bool IsMediumTransition() const { return inside != outside; }
    const Medium *inside, *outside;
};
}  // namespace pbrt

#endif  // PBRT_CORE_MEDIUM_H
