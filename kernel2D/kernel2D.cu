/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "kernel2D.h"

namespace DynaMap{

    kernel2D::kernel2D(){}
    kernel2D::~kernel2D(){}

    __global__ 
    void generateGaussianKernel1D(float *gaussianKernel, float delta, int radius) {
        int x = threadIdx.x - radius;
        gaussianKernel[threadIdx.x] = __expf(-(x * x) / (2 * delta * delta));
    }
    void kernel2D::generateFilterkernel1D(gaussianKernal& kernel){
        int threads_per_block = kernel.kernelRadius;
        int thread_blocks = 1;
        // std::cout << "<<<kernel_generateGaussian>>> threadBlocks: "<< thread_blocks << ", threadPerBlock: " << threads_per_block << std::endl;
        generateGaussianKernel1D <<< thread_blocks, threads_per_block >>>(kernel.kernel, kernel.kernelDelta, kernel.kernelRadius);
        cudaDeviceSynchronize();
        if(cudaGetLastError())std::cout << cudaGetErrorString(cudaGetLastError()) << std::endl;

    }
    __global__ 
    void generateGaussianKernel2D(float *gaussianKernel, float delta, int radius) {
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        int size = radius * radius;
        for (int idx = index; idx < size; idx += stride){
            int v = idx / radius;
            int u = idx * radius + v;
            int u_idx = (v - radius) * (v - radius);
            int v_idx = (u - radius) * (u - radius);
            gaussianKernel[idx] = (__expf(-(u_idx) / (2 * delta * delta))) * (__expf(-(v_idx) / (2 * delta * delta)));
        }
    }
    void kernel2D::generateFilterkernel2D(gaussianKernal& kernel){
        int threads_per_block = 32;
        int thread_blocks =(kernel.kernelRadius * kernel.kernelRadius + threads_per_block - 1) / threads_per_block;
        std::cout << "<<<kernel_generateGaussianKernel2D>>> threadBlocks: "<< thread_blocks << ", threadPerBlock: " << threads_per_block << std::endl;
        generateGaussianKernel2D <<< thread_blocks, threads_per_block >>>(kernel.kernel, kernel.kernelDelta, kernel.kernelRadius);
        cudaDeviceSynchronize();
        if(cudaGetLastError())std::cout << cudaGetErrorString(cudaGetLastError()) << std::endl;
    }
    cv::Mat kernel2D::testgenerateGaussianKernel2D(gaussianKernal& kernel){
        cv::Mat kernelImage(kernel.kernelRadius*2, kernel.kernelRadius*2, CV_32FC1);
		cudaMallocManaged(&kernel.kernel, sizeof(float) * (kernel.kernelRadius)*(kernel.kernelRadius)); 
        this->generateFilterkernel2D(kernel);
        for (int i = 0; i < kernel.kernelRadius; i++) {
            for (int j = 0; j < kernel.kernelRadius; j++) {
                kernelImage.at<float>(i, j) = kernel.kernel[i * kernel.kernelRadius + j];
            }
        }
        for(int cnt=0; cnt< (kernel.kernelRadius)*(kernel.kernelRadius);cnt++)
        std::cout << "cnt " << cnt << ": " <<kernel.kernel[cnt] << std::endl;
        cudaFree(kernel.kernel);
        return kernelImage;
    }

}  // namespace DynaMap