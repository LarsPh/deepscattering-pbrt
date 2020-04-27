//WZR:

#include "recordstencils.h"

namespace pbrt {
RecordStencils::RecordStencils(GridDensityMedium* m, Point3f p, Vector3f wo,
                               Vector3f wi, Float u, int K)
    : medium(m), unit(u), k(K), wLight(wi), wView(wo), location(p) {
    Vector3f z = Normalize(wLight);  // light ray
    Vector3f y = Normalize(Cross(z, wView));
    Vector3f x = Normalize(Cross(z, y));
    Matrix4x4 matrix(x[0], y[0], z[0], p[0], x[1], y[1], z[1], p[1], x[2], y[2],
                     z[2], p[2], 0.f, 0.f, 0.f, 1.f);
    stencilsCoordinate = Transform(matrix);
}
void RecordStencils::record(Float* data) {    
        Point3f p;
        for (int K = 0; K < k; K++) {
            Float unitK = unit * pow(2, K);
            Float dx = 2.f * unitK / 4.f;
            Float dy = 2.f * unitK / 4.f;
            Float dz = 4.f * unitK / 8.f;
            Point3f a(-unitK, -unitK, -unitK);
            Point3f b(unitK, unitK, 3 * unitK);
            for (int m = 0; m < 9; m++)               
                for (int j = 0; j < 5; j++)
                    for (int i = 0; i < 5; i++)
                     {
                        p = a + Vector3f(i * dx, j * dy, m * dz);
                        Point3f pMedium = medium->World2Medium(Stencils2World(p));
                        Float density =
                            medium->Density(pMedium) * medium->invMaxDensity;

                        /*if (i < 3 && j < 3 && m < 3)
                            LOG(INFO)
                                << "WZR: wLight " << wLight << "wView " << wView
                                << "location " << location
                                << "WZR: left bottom 3x3 stencil "
                                   "positions(stencil coordinate) "
                                << p << " (world coordinate) "
                                << Stencils2World(p) << " density " << density;
                        */
                        //record density
                        data[K*225+5*5*m+5*j+i] = density;
                    }
            Vector3f delta = Abs(p - b);
            CHECK(delta.x < 1e-3 && delta.y < 1e-3 && delta.z < 1e-3)
                << "incorrect stencil positions";
        }    
        // Float gamma = BetterAcos(Dot(wLight, wView));
        Float gamma = Dot(Normalize(wLight), Normalize(wView));
        data[2250] = gamma;
    }

}  // namespace pbrt