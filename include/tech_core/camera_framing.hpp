#pragma once
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace tech_core {
// Bound visible body geometry, excluding world/ground and collision proxies.
// The same rule frames robots with different link names, reach and placement.
inline void frame_model(const mjModel* model, const mjData* data, mjvCamera& camera) {
    std::array<double,3> lo,hi;
    lo.fill(std::numeric_limits<double>::infinity());
    hi.fill(-std::numeric_limits<double>::infinity());
    for(int g=0;g<model->ngeom;++g) {
        if(model->geom_bodyid[g]==0 || model->geom_type[g]==mjGEOM_PLANE ||
           model->geom_group[g]>2 || model->geom_rgba[4*g+3]==0) continue;
        const auto* a=model->geom_aabb+6*g;
        const auto* r=data->geom_xmat+9*g;
        for(int row=0;row<3;++row) {
            double centre=data->geom_xpos[3*g+row],extent=0;
            for(int col=0;col<3;++col) {
                centre+=r[3*row+col]*a[col];
                extent+=std::abs(r[3*row+col])*a[col+3];
            }
            lo[row]=std::min(lo[row],centre-extent);hi[row]=std::max(hi[row],centre+extent);
        }
    }
    if(!std::isfinite(lo[0])) return;
    double diagonal=0;
    for(int row=0;row<3;++row) {
        camera.lookat[row]=.5*(lo[row]+hi[row]);
        diagonal+=std::pow(hi[row]-lo[row],2);
    }
    camera.type=mjCAMERA_FREE;
    camera.distance=std::max(.3,1.5*std::sqrt(diagonal));
}
} // namespace tech_core
