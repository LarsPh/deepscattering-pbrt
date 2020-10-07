//WZR:


 // cameras/reversedenvironment.cpp*
#include "cameras/ds.h"
#include "paramset.h"
#include "sampler.h"
#include "stats.h"
// WZR
#include "sampling.h"
#include "scene.h"

namespace pbrt {

    // EnvironmentCamera Method Definitions
    Float DSCamera::GenerateRay(const CameraSample& sample,
        Ray* ray) const {
        ProfilePhase prof(Prof::GenerateCameraRay);
        // Compute randomlized camera ray direction
        Point3f center;
        Float radius;
        sceneBounds.BoundingSphere(&center, &radius);

        RNG rng;
        Vector3f dir =
            UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
        Vector3f up(rng.UniformFloat(), rng.UniformFloat(), rng.UniformFloat());
        
        Point3f look = center + dir;
        Transform worldToSphere = LookAt(center, look, up);
        Transform sphereToWorld = Inverse(worldToSphere);

        Point2f bias2 =
            ConcentricSampleDisk({rng.UniformFloat(), rng.UniformFloat()});
        Point3f bias3(bias2.x, bias2.y, 0.);
        

        Point3f p = sphereToWorld(worldToSphere(center) + bias3) +
                    Normalize(-dir) * (radius + FLT_EPSILON);
        
        *ray = Ray(p, dir, Infinity, Lerp(sample.time, shutterOpen, shutterClose));
        ray->medium = medium;
        *ray = CameraToWorld(*ray);
        return 1;
    }

    void DSCamera::Preprocess(const Scene& scene) {
        sceneBounds = scene.WorldBound();
    }

    DSCamera* CreateDSCamera(const ParamSet& params,
        const AnimatedTransform& cam2world,
        Film* film, const Medium* medium) {
        // Extract common camera parameters from _ParamSet_
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

        return new DSCamera(cam2world, shutteropen, shutterclose, film,
            medium);
    }

}  // namespace pbrt
