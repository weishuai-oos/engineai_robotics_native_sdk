#ifndef ENGINEAI_ROBOTICS_SIMULATION_MUJOCO_ELASTIC_BAND_H_
#define ENGINEAI_ROBOTICS_SIMULATION_MUJOCO_ELASTIC_BAND_H_

#include <mujoco/mujoco.h>

void ResolveElasticBandBody(const mjModel* m);
void ElasticBandPassiveCallback(const mjModel* m, mjData* d);
void AdjustElasticBandRestLength(mjtNum delta);
void ToggleElasticBand();

#endif  // ENGINEAI_ROBOTICS_SIMULATION_MUJOCO_ELASTIC_BAND_H_
