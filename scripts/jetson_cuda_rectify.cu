// Rectify packed BGR frames with OpenCV CV_32FC2 maps on a Jetson GPU.
#include <cuda_runtime.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

void check(cudaError_t status, const char* operation) {
    if (status != cudaSuccess) {
        throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
    }
}

__device__ uchar3 pixel(const uchar3* image, int width, int height, int x, int y) {
    if (x < 0 || x >= width || y < 0 || y >= height) return make_uchar3(0, 0, 0);
    return image[y * width + x];
}

__global__ void rectify(const uchar3* input, const float2* map, uchar3* output,
                        int width, int height) {
    const int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= width * height) return;
    const float2 source = map[index];
    if (!(source.x > -1 && source.x < width && source.y > -1 && source.y < height)) {
        output[index] = make_uchar3(0, 0, 0);
        return;
    }
    // OpenCV INTER_LINEAR quantizes each fractional coordinate to 1/32.
    const int qx = __float2int_rn(source.x * 32);
    const int qy = __float2int_rn(source.y * 32);
    const int x = qx >> 5;
    const int y = qy >> 5;
    const int dx = qx - 32 * x;
    const int dy = qy - 32 * y;
    const int wa = (32 - dx) * (32 - dy);
    const int wb = dx * (32 - dy);
    const int wc = (32 - dx) * dy;
    const int wd = dx * dy;
    const uchar3 a = pixel(input, width, height, x, y);
    const uchar3 b = pixel(input, width, height, x + 1, y);
    const uchar3 c = pixel(input, width, height, x, y + 1);
    const uchar3 d = pixel(input, width, height, x + 1, y + 1);
    output[index] = make_uchar3((a.x * wa + b.x * wb + c.x * wc + d.x * wd + 512) >> 10,
                                (a.y * wa + b.y * wb + c.y * wc + d.y * wd + 512) >> 10,
                                (a.z * wa + b.z * wb + c.z * wc + d.z * wd + 512) >> 10);
}

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "Usage: jetson_cuda_rectify WIDTH HEIGHT MAP_F32_XY INPUT_BGR OUTPUT_BGR\n";
        return 2;
    }
    try {
        const int width = std::stoi(argv[1]);
        const int height = std::stoi(argv[2]);
        if (width <= 0 || height <= 0 || width > 4096 || height > 4096) {
            throw std::runtime_error("Invalid image size");
        }
        const int pixels = width * height;
        const size_t frame_bytes = pixels * sizeof(uchar3);
        const size_t map_bytes = pixels * sizeof(float2);
        std::ifstream map_file(argv[3], std::ios::binary);
        std::ifstream frames(argv[4], std::ios::binary);
        if (!map_file || !frames) throw std::runtime_error("Cannot open map or input video");
        std::vector<float2> map(pixels);
        if (!map_file.read(reinterpret_cast<char*>(map.data()), map_bytes)
            || map_file.peek() != std::char_traits<char>::eof()) {
            throw std::runtime_error("Map must contain one float32 x/y pair per output pixel");
        }
        std::ofstream output(argv[5], std::ios::binary);
        if (!output) throw std::runtime_error("Cannot open output");
        std::vector<uchar3> image(pixels), result(pixels);
        float2* device_map = nullptr;
        uchar3* device_input = nullptr;
        uchar3* device_output = nullptr;
        check(cudaMalloc(&device_map, map_bytes), "allocate map");
        check(cudaMalloc(&device_input, frame_bytes), "allocate input");
        check(cudaMalloc(&device_output, frame_bytes), "allocate output");
        check(cudaMemcpy(device_map, map.data(), map_bytes, cudaMemcpyHostToDevice), "upload map");
        cudaEvent_t start, finish;
        check(cudaEventCreate(&start), "create start event");
        check(cudaEventCreate(&finish), "create end event");
        cudaDeviceProp gpu{};
        check(cudaGetDeviceProperties(&gpu, 0), "identify GPU");
        int count = 0;
        double kernel_ms = 0;
        double frame_ms = 0;
        while (frames.read(reinterpret_cast<char*>(image.data()), frame_bytes)) {
            const auto begin = std::chrono::steady_clock::now();
            check(cudaMemcpy(device_input, image.data(), frame_bytes, cudaMemcpyHostToDevice),
                  "upload frame");
            check(cudaEventRecord(start), "start kernel timer");
            rectify<<<(pixels + 255) / 256, 256>>>(device_input, device_map, device_output, width, height);
            check(cudaGetLastError(), "launch rectification");
            check(cudaEventRecord(finish), "stop kernel timer");
            check(cudaMemcpy(result.data(), device_output, frame_bytes, cudaMemcpyDeviceToHost),
                  "download rectified frame");
            frame_ms += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - begin).count();
            float elapsed = 0;
            check(cudaEventElapsedTime(&elapsed, start, finish), "read kernel timing");
            kernel_ms += elapsed;
            if (!output.write(reinterpret_cast<const char*>(result.data()), frame_bytes)) {
                throw std::runtime_error("Cannot write output frame");
            }
            ++count;
        }
        if (!frames.eof() || frames.gcount() != 0 || count == 0) {
            throw std::runtime_error("Input must contain complete BGR frames");
        }
        output.close();
        if (!output) throw std::runtime_error("Cannot finish output");
        cudaEventDestroy(start);
        cudaEventDestroy(finish);
        cudaFree(device_output);
        cudaFree(device_input);
        cudaFree(device_map);
        std::cout << std::fixed << std::setprecision(3)
                  << "GPU=" << gpu.name << " frames=" << count
                  << " kernel_ms_per_frame=" << kernel_ms / count
                  << " copy_and_kernel_ms_per_frame=" << frame_ms / count << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
