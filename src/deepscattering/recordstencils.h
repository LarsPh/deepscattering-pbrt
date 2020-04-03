// WZR:

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_DEEPSCATTERING_RECORDSTENCILS_H
#define PBRT_DEEPSCATTERING_RECORDSTENCILS_H

// deepscattering/recordstencils.h*
#include "pbrt.h"
#include "transform.h"
#include "media/grid.h"

namespace pbrt {
class RecordStencils {
  public:
    RecordStencils(GridDensityMedium* m, Point3f p, Vector3f wo, Vector3f wi, Float u, int K = 10);
    void record(Float *data);

  private:
    GridDensityMedium* medium;
    int k;
    Float unit;
    Transform stencilsCoordinate;
    Point3f Stencils2World(Point3f p) { return stencilsCoordinate(p); }
    // for logging
    Point3f location;
    Vector3f wLight;
    Vector3f wView;
};
}  // namespace pbrt

#endif  // PBRT_DEEPSCATTERING_RECORDSTENCILS_H