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

    // DSCamera Method Definitions
void DSCamera::Shuffle(const Scene& scene, std::unique_ptr<Sampler> sampler) {
    while (true) {
        // Compute randomlized point x direction and position
        Point3f center;
        Float radius;
        Bounds3f sceneBounds = scene.WorldBound();
        sceneBounds.BoundingSphere(&center, &radius);

        RNG rng;
        dir = UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
        Vector3f up(rng.UniformFloat(), rng.UniformFloat(), rng.UniformFloat());

        Point3f look = center + dir;
        Transform worldToSphere = LookAt(center, look, up);
        Transform sphereToWorld = Inverse(worldToSphere);

        Point2f bias2 =
            ConcentricSampleDisk({rng.UniformFloat(), rng.UniformFloat()});
        Point3f bias3(bias2.x, bias2.y, 0.);

        Point3f pX = sphereToWorld(worldToSphere(center) + bias3) +
                     Normalize(-dir) * (radius + FLT_EPSILON);
        // Free flight sample xi position
        MemoryArena arena;
        Ray ray(pX, dir, Infinity);

        SurfaceInteraction isect;
        // regenerate when generated ray doesn't intersect with scene
        if (!scene.Intersect(ray, &isect)) continue;
        assert(!ray.medium);
        assert(!isect.bsdf);
        ray = isect.SpawnRay(ray.d);
        assert(ray.medium);
        MediumInteraction mi;
        ray.medium->Sample(ray, *sampler, arena, &mi);
        if (!mi.IsValid()) continue;

        *newMedium = *ray.medium;
        p = mi.p;
        // store densities at p and angle
        // TODO
        break;
    }
}
void DSCamera::getDSInfo(GridDensityMedium* medium, Point3f* p, Vector3f* wo) {
    medium = (GridDensityMedium*)newMedium;
    *p = DSCamera::p;
    *wo = dir;
}

    Float DSCamera::GenerateRay(const CameraSample& sample,
        Ray* ray) const {
        ProfilePhase prof(Prof::GenerateCameraRay);
        
        *ray = Ray(p, dir, Infinity, Lerp(sample.time, shutterOpen, shutterClose));
        assert(newMedium);
        // use updated medium after sampling in Shuffle instead of using medium of camera position
        ray->medium = newMedium;
        *ray = CameraToWorld(*ray);
        return 1;
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
