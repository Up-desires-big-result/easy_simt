// easy_simt_kernel.cu
// 演示 kernel：共享内存交换 + 数据相关分支分化
//   1. 全局内存 -> 共享内存写入, __syncthreads, 再读邻居线程的数据(交换)
//   2. 按数据正负走不同长度的计算路径 => warp 内真实分支分化
//   3. N=1000 不能被块大小整除 => 最后一个块还有边界判断分化
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cuda_runtime.h>

#define N     1000          // 故意不是 32 的整数倍
#define BLOCK 32            // easy_simt 硬件口径: 32 线程/块

#define CUDA_CHECK(call)                                             \
    do {                                                             \
        cudaError_t err = (call);                                    \
        if (err != cudaSuccess) {                                    \
            fprintf(stderr, "CUDA error %s:%d: %s\n", __FILE__,      \
                    __LINE__, cudaGetErrorString(err));              \
            exit(EXIT_FAILURE);                                      \
        }                                                            \
    } while (0)

__global__ void shmem_diverge_kernel(const float* __restrict__ in,
                                     float* __restrict__ out, int n) {
    __shared__ float tile[BLOCK];          // 共享内存

    int gid = blockIdx.x * blockDim.x + threadIdx.x;

    // 边界保护: 最后一个块里部分线程越界 => 分化点 1
    float v = 0.0f;
    if (gid < n) v = in[gid];

    tile[threadIdx.x] = v;                 // 共享内存写
    __syncthreads();

    // 读"邻居"的数据: 线程 tid 读 tid^0x10 位置 => 线程间交换, 共享内存读
    float x = tile[threadIdx.x ^ 0x10];

    // 数据相关分支: 正数走 8 次 FMA 长路径, 负数走短路径 => 分化点 2
    float r;
    if (x > 0.0f) {
        r = x;
        #pragma unroll
        for (int i = 0; i < 8; ++i)
            r = r * 1.0001f + 0.0001f;   // 乘法+加法分开, 编译时需 -fmad=false
    } else {
        r = -x;
    }

    if (gid < n) out[gid] = r;
}

// CPU 参考实现
static void cpu_reference(const float* in, float* ref, int n) {
    // 与 kernel 相同的交换规则: 块内位置异或 0x10
    for (int b = 0; b * BLOCK < n; ++b) {
        int base = b * BLOCK;
        float tmp[BLOCK];
        for (int t = 0; t < BLOCK; ++t) {
            int gid = base + t;
            tmp[t] = (gid < n) ? in[gid] : 0.0f;
        }
        for (int t = 0; t < BLOCK; ++t) {
            int gid = base + t;
            if (gid >= n) break;
            float x = tmp[t ^ 0x10];
            float r;
            if (x > 0.0f) {
                r = x;
                for (int i = 0; i < 8; ++i)
                    r = r * 1.0001f + 0.0001f;   // 与 kernel 保持一致: 乘加分离
            } else {
                r = -x;
            }
            ref[gid] = r;
        }
    }
}

int main() {
    const size_t bytes = N * sizeof(float);
    float* h_in  = (float*)malloc(bytes);
    float* h_out = (float*)malloc(bytes);
    float* h_ref = (float*)malloc(bytes);

    // 数据正负交错, 保证每个 warp 内正负都有 => 分化必然发生
    for (int i = 0; i < N; ++i)
        h_in[i] = (float)((i % 7) - 3) * 100.0f;

    float *d_in, *d_out;
    CUDA_CHECK(cudaMalloc(&d_in, bytes));
    CUDA_CHECK(cudaMalloc(&d_out, bytes));
    CUDA_CHECK(cudaMemcpy(d_in, h_in, bytes, cudaMemcpyHostToDevice));

    int grid = (N + BLOCK - 1) / BLOCK;    // 32 个块
    shmem_diverge_kernel<<<grid, BLOCK>>>(d_in, d_out, N);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    CUDA_CHECK(cudaMemcpy(h_out, d_out, bytes, cudaMemcpyDeviceToHost));

    cpu_reference(h_in, h_ref, N);
    float max_err = 0.0f;
    for (int i = 0; i < N; ++i) {
        float err = fabsf(h_out[i] - h_ref[i]);
        if (err > max_err) max_err = err;
    }

    printf("== shmem_diverge (N=%d, block=%d, grid=%d) ==\n", N, BLOCK, grid);
    printf("max_abs_error = %.3e\n", max_err);
    printf("RESULT        = %s\n", max_err < 1e-3f ? "PASS" : "FAIL");
    printf("out[0]=%.4f out[128]=%.4f out[999]=%.4f\n",
           h_out[0], h_out[128], h_out[999]);

    cudaFree(d_in); cudaFree(d_out);
    free(h_in); free(h_out); free(h_ref);
    return 0;
}
