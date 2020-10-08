
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

#ifndef PBRT_MEDIA_GRID_H
#define PBRT_MEDIA_GRID_H

// media/grid.h*
#include "medium.h"
#include "transform.h"
#include "stats.h"

namespace pbrt {

STAT_MEMORY_COUNTER("Memory/Volume density grid", densityBytes);

// GridDensityMedium Declarations
class GridDensityMedium : public Medium {
  public:
    // GridDensityMedium Public Methods
    GridDensityMedium(const Spectrum &sigma_a, const Spectrum &sigma_s, Float g,
                      int nx, int ny, int nz, const Transform &mediumToWorld,
                      const Float *d)
        : sigma_a(sigma_a),
          sigma_s(sigma_s),
          g(g),
          nx(nx),
          ny(ny),
          nz(nz),
          WorldToMedium(Inverse(mediumToWorld)),
          density(new Float[nx * ny * nz]) {
        densityBytes += nx * ny * nz * sizeof(Float);
        memcpy((Float *)density.get(), d, sizeof(Float) * nx * ny * nz);
        // Precompute values for Monte Carlo sampling of _GridDensityMedium_
        Float maxDensity = 0;
        Float densitySum = 0;
        // WZR: set average rho_t to correct value
        int nonzeroVexels = 0;
        for (int i = 0; i < nx * ny * nz; ++i) {
            maxDensity = std::max(maxDensity, density[i]);
            densitySum += density[i];
            if (density[i] != 0) ++nonzeroVexels;
        }
        // std::cout << nonzeroVexels << std::endl;
        invMaxDensity = 1 / maxDensity;
        // 2 for the forward peak of Mie phase funtion
        Float f = 1. /  (densitySum / nonzeroVexels) * 0.43;
        this->sigma_a *= f;
        this->sigma_s *= f;
        sigma_t = (sigma_a + sigma_s)[0];
        if (Spectrum(sigma_t) != sigma_a + sigma_s)
            Error(
                "GridDensityMedium requires a spectrally uniform attenuation "
                "coefficient!");
        for (int i = 0; i < nx * ny * nz; ++i)
            maxDensity = std::max(maxDensity, density[i]);
        invMaxDensity = 1 / maxDensity;
    }

    Float Density(const Point3f &p) const;
    Float D(const Point3i &p) const {
        Bounds3i sampleBounds(Point3i(0, 0, 0), Point3i(nx, ny, nz));
        if (!InsideExclusive(p, sampleBounds)) return 0;
        return density[(p.z * ny + p.y) * nx + p.x];
    }
    Spectrum Sample(const Ray &ray, Sampler &sampler, MemoryArena &arena,
                    MediumInteraction *mi) const;
    Spectrum Tr(const Ray &ray, Sampler &sampler) const;
    // WZR: for calling Density outside Tr
    Point3f World2Medium(Point3f p) { return WorldToMedium(p); }
    Point3f Medium2World(Point3f p) { return Inverse(WorldToMedium)(p); }

    // WZR: make invMaxDensity public for recording normalized data 
    Float invMaxDensity;

  private:
    // GridDensityMedium Private Data
    Spectrum sigma_a, sigma_s;
    const Float g;
    const int nx, ny, nz;
    const Transform WorldToMedium;
    std::unique_ptr<Float[]> density;
    Float sigma_t;
};

}  // namespace pbrt

#endif  // PBRT_MEDIA_GRID_H
