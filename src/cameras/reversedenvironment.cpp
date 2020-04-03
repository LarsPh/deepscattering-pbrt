//WZR:


 // cameras/reversedenvironment.cpp*
#include "cameras/reversedenvironment.h"
#include "paramset.h"
#include "sampler.h"
#include "stats.h"

namespace pbrt {

    // EnvironmentCamera Method Definitions
    Float ReversedEnvironmentCamera::GenerateRay(const CameraSample& sample,
        Ray* ray) const {
        ProfilePhase prof(Prof::GenerateCameraRay);
        // Compute environment camera ray direction
        Float theta = Pi * sample.pFilm.y / film->fullResolution.y;
        Float phi = 2 * Pi * sample.pFilm.x / film->fullResolution.x;
        // reversed direction
        Vector3f dir = -Vector3f(std::sin(theta) * std::cos(phi), std::cos(theta),
            std::sin(theta) * std::sin(phi));
        Point3f p = Point3f(0, 0, 0) + Normalize(-dir) * radius;
        *ray = Ray(p, dir, Infinity, Lerp(sample.time, shutterOpen, shutterClose));
        ray->medium = medium;
        *ray = CameraToWorld(*ray);
        return 1;
    }

    ReversedEnvironmentCamera* CreateReversedEnvironmentCamera(const ParamSet& params,
        const AnimatedTransform& cam2world,
        Film* film, const Medium* medium) {
        // Extract common camera parameters from _ParamSet_
        Float radius = params.FindOneFloat("radius", 10.f); //set defalut to?
        Float shutteropen = params.FindOneFloat("shutteropen", 0.f);
        Float shutterclose = params.FindOneFloat("shutterclose", 1.f);
        if (shutterclose < shutteropen) {
            Warning("Shutter close time [%f] < shutter open [%f].  Swapping them.",
                shutterclose, shutteropen);
            std::swap(shutterclose, shutteropen);
        }
        Float lensradius = params.FindOneFloat("lensradius", 0.f);
        Float focaldistance = params.FindOneFloat("focaldistance", 1e30f);
        Float frame = params.FindOneFloat(
            "frameaspectratio",
            Float(film->fullResolution.x) / Float(film->fullResolution.y));
        Bounds2f screen;
        if (frame > 1.f) {
            screen.pMin.x = -frame;
            screen.pMax.x = frame;
            screen.pMin.y = -1.f;
            screen.pMax.y = 1.f;
        }
        else {
            screen.pMin.x = -1.f;
            screen.pMax.x = 1.f;
            screen.pMin.y = -1.f / frame;
            screen.pMax.y = 1.f / frame;
        }
        int swi;
        const Float* sw = params.FindFloat("screenwindow", &swi);
        if (sw) {
            if (swi == 4) {
                screen.pMin.x = sw[0];
                screen.pMax.x = sw[1];
                screen.pMin.y = sw[2];
                screen.pMax.y = sw[3];
            }
            else
                Error("\"screenwindow\" should have four values");
        }
        (void)lensradius;     // don't need this
        (void)focaldistance;  // don't need this

        return new ReversedEnvironmentCamera(cam2world, radius, shutteropen, shutterclose, film,
            medium);
    }

}  // namespace pbrt
