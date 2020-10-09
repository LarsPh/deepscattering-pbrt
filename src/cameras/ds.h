//WZR:

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CAMERAS_REVERSEDENVIRONMENT_H
#define PBRT_CAMERAS_REVERSEDENVIRONMENT_H

// cameras/ds.h*
#include "camera.h"
#include "film.h"

namespace pbrt {

    // EnvironmentCamera Declarations
    class DSCamera : public Camera {
    public:
        // EnvironmentCamera Public Methods
        DSCamera(const AnimatedTransform& CameraToWorld,
                                Float shutterOpen,
                                Float shutterClose, Film* film,
                                const Medium* medium)
          : Camera(CameraToWorld, shutterOpen, shutterClose, film, medium) {}
        Float GenerateRay(const CameraSample& sample, Ray*) const;
        void Shuffle(const Scene&, std::unique_ptr<Sampler>);
        void getDSInfo(GridDensityMedium* medium, Point3f* p, Vector3f* wo);

     private:
        Point3f p;
        Vector3f dir;
        Medium* newMedium;
    };

    DSCamera* CreateDSCamera(const ParamSet& params,
        const AnimatedTransform& cam2world,
        Film* film, const Medium* medium);
    }  // namespace pbrt

#endif  // PBRT_CAMERAS_DS_H
