//WZR:

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CAMERAS_REVERSEDENVIRONMENT_H
#define PBRT_CAMERAS_REVERSEDENVIRONMENT_H

// cameras/reveredenvironment.h*
#include "camera.h"
#include "film.h"

namespace pbrt {

    // EnvironmentCamera Declarations
    class ReversedEnvironmentCamera : public Camera {
    public:
        // EnvironmentCamera Public Methods
        ReversedEnvironmentCamera(const AnimatedTransform& CameraToWorld,
                                Float r, Float shutterOpen,
                                Float shutterClose, Film* film,
                                const Medium* medium)
          : Camera(CameraToWorld, shutterOpen, shutterClose, film, medium), radius(r){}
        Float GenerateRay(const CameraSample& sample, Ray*) const;

     private:
        Float radius;
    };

    ReversedEnvironmentCamera* CreateReversedEnvironmentCamera(const ParamSet& params,
        const AnimatedTransform& cam2world,
        Film* film, const Medium* medium);
    }  // namespace pbrt

#endif  // PBRT_CAMERAS_REVERSEDENVIRONMENT_H
