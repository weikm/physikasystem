#pragma once

namespace VPE {
void UpdateParticleBufferCuda(size_t particle_count,
                              int split_num,
                              float pos_offset_x,
                              float pos_offset_y,
                              float pos_offset_z,
                              float *dst_buffer_d,
                              float *position_d,
                              float *radius_d);

void CopyDoubleToFloatCuda(double *src, int src_pitch, float *dst, int count_x, int count_y);
}  // namespace VPE