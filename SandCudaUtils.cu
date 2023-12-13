#include "SandCudaUtils.h"
namespace VPE {
__global__ void Kernel_UpdateParticleBuffer(size_t particle_count,
                                            int split_num,
                                            float pos_offset_x,
                                            float pos_offset_y,
                                            float pos_offset_z,
                                            float *dst_buffer_d,
                                            float *position_d,
                                            float *radius_d) {
    size_t index = threadIdx.x + (blockIdx.x * blockDim.x);

    if (index >= particle_count) {
        return;
    }

    float *pos = position_d + index * 3;
    for (int i = 0; i < split_num; i++) {
        int target_index = index * split_num + i;
        float r = radius_d[target_index];
        float *dst = dst_buffer_d + 4 * target_index;
        dst[0] = static_cast<float>(pos[0] + pos_offset_x);
        // offset by radius to make rendering result less flat
        dst[1] = static_cast<float>(pos[1] * 0.5 + pos_offset_y + r);
        dst[2] = static_cast<float>(pos[2] + pos_offset_z);
        dst[3] = r;
    }
}

void UpdateParticleBufferCuda(size_t particle_count,
                              int split_num,
                              float pos_offset_x,
                              float pos_offset_y,
                              float pos_offset_z,
                              float *dst_buffer_d,
                              float *position_d,
                              float *radius_d) {
    Kernel_UpdateParticleBuffer <<< (particle_count + 511) / 512, 512>>>(
        particle_count,
        split_num,
        pos_offset_x, pos_offset_y, pos_offset_z,
        dst_buffer_d,
        position_d,
        radius_d);
}

__global__ void Kernal_ConvertDoubleToFloat(double *src,
                                            int src_pitch,
                                            float *dst,
                                            int count_x, int count_y) {
    size_t x = threadIdx.x + (blockIdx.x * blockDim.x);
    size_t y = threadIdx.y + (blockIdx.y * blockDim.y);

    if (x >= count_x || y >= count_y) {
        return;
    }
    dst[x + y * count_x] = static_cast<float>(src[x + y * src_pitch]);
}

void CopyDoubleToFloatCuda(double *src, int src_pitch, float *dst, int count_x, int count_y) {
    dim3 dimBlock(16, 16);
    dim3 dimGrid;
    dimGrid.x = (count_x + dimBlock.x - 1) / dimBlock.x;
    dimGrid.y = (count_y + dimBlock.y - 1) / dimBlock.y;
    Kernal_ConvertDoubleToFloat<<<dimGrid, dimBlock>>>(src, src_pitch, dst, count_x, count_y);
}
}  // namespace VPE