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
void DSCamera::Shuffle(const Scene& scene, RNG &rng, MediumInteraction *fst_mi, MemoryArena *arena) {
    Point3f center;
    Float radius;
    Bounds3f sceneBounds = scene.WorldBound();
    sceneBounds.BoundingSphere(&center, &radius);

    while (true) {
        // Compute randomlized point x direction and position

        dir = UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
        Vector3f up(rng.UniformFloat(), rng.UniformFloat(), rng.UniformFloat());

        Point3f look = center + dir;
        Transform worldToSphere = LookAt(center, look, up);
        Transform sphereToWorld = Inverse(worldToSphere);

        Point2f bias2 =
            ConcentricSampleDisk({rng.UniformFloat(), rng.UniformFloat()});
        Point3f bias3(bias2.x, bias2.y, 0.);

        Point3f pX = sphereToWorld(worldToSphere(center) + bias3) +
                     Normalize(-dir) * (radius + 0.01);
        // std::cout << std::endl << "px: " << pX << "\t";
        // if ((pX - center).Length() < radius)
        //    std::cout << "inside: " << (pX - center).Length() - radius;
        // else
        //    std::cout << "outside bias: " << (pX - center).Length() - radius << "\t";
       
        // Free flight sample xi position
        // MemoryArena arena;
        Ray ray(pX, dir, Infinity);

        SurfaceInteraction isect;
        // regenerate when generated ray doesn't intersect with scene
        if (!scene.Intersect(ray, &isect)) {
            // std::cout << std::endl << "miss surface!" << std::endl;			
            continue;
        }
        if (ray.medium != nullptr)
            std::cout << std::endl << "Problematic!" << std::endl;
        if (isect.bsdf != nullptr)
            std::cout << std::endl << "Problematic!" << std::endl;

        // std::cout << "isect.p: " << isect.p << "\t" << "dis pX-isect.p: " << (pX - isect.p).Length() << "\t"; 

        ray = isect.SpawnRay(ray.d);
        // std::cout << "new ray o: " << ray.o << "\t"
        //           << "new ray d: " << ray.d << "\t";
        if (ray.medium == nullptr)
            std::cout << std::endl << "Problematic!" << std::endl;
        ray.medium->Sample_u(ray, rng, *arena, fst_mi);
        // problem here
        if (!fst_mi->IsValid()) {
            // std::cout << std::endl << "miss medium!" << std::endl;
            ++missCount;
            continue;
        }
        ++hitCount;
        newMedium = ray.medium;
        p = fst_mi->p;
        // std::cout << "mi.p: " << p << "\t"
        //           << "dis isec.p-mi.p: " << (p - isect.p).Length() << "\t";       
        break;
    }
}
void DSCamera::printInfo() {
    std::cout << std::endl
              << missCount << " of all " << hitCount + missCount
              << " sample rays didn't hit the medium" << std::endl;
}
void DSCamera::getDSInfo(GridDensityMedium** medium, Point3f* p, Vector3f* wo) {
    *medium = (GridDensityMedium *)newMedium;
    *p = DSCamera::p;
    *wo = dir;
}

    Float DSCamera::GenerateRay(const CameraSample& sample,
        Ray* ray) const {
        ProfilePhase prof(Prof::GenerateCameraRay);
        // std::cout << "GenerateRay::p: " << p << "\t";
        *ray = Ray(p, dir, Infinity, Lerp(sample.time, shutterOpen, shutterClose));
        if (newMedium == nullptr)
            std::cout << "Problematic!";
        // use updated medium after sampling in Shuffle instead of using medium of camera position
        ray->medium = newMedium;
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
