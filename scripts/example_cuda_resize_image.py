import time
import pycuda.autoinit
import numpy as np
import pycuda.driver as cuda
from pycuda.compiler import SourceModule

mod = SourceModule("""
__global__ void resize_images(unsigned char *input, unsigned char *output, int width, int height, int new_width, int new_height, float scale_factor, int num_images) {
    int img_idx = blockIdx.z;
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    int idy = threadIdx.y + blockIdx.y * blockDim.y;
    if (img_idx < num_images && idx < new_width && idy < new_height) {
        int x = int(idx / scale_factor);
        int y = int(idy / scale_factor);
        int input_idx = img_idx * (width * height * 3) + (y * width + x) * 3;
        int output_idx = img_idx * (new_width * new_height * 3) + (idy * new_width + idx) * 3;
        output[output_idx] = input[input_idx];
        output[output_idx + 1] = input[input_idx + 1];
        output[output_idx + 2] = input[input_idx + 2];
    }
}
""")

resize_images_kernel = mod.get_function("resize_images")


def resize_images_gpu(images, scale_factor):
    num_images, height, width, _ = images.shape
    new_width = int(width * scale_factor)
    new_height = int(height * scale_factor)
    input_gpu = cuda.mem_alloc(images.nbytes)
    output_gpu = cuda.mem_alloc(num_images * new_width * new_height * 3)

    cuda.memcpy_htod(input_gpu, images)

    block_size = (16, 16, 1)
    grid_size = (
        int((new_width + block_size[0] - 1) / block_size[0]), int((new_height + block_size[1] - 1) / block_size[1]),
        num_images)
    resize_images_kernel(input_gpu, output_gpu, np.int32(width), np.int32(height), np.int32(new_width),
                         np.int32(new_height), np.float32(1.0 / scale_factor), np.int32(num_images), block=block_size,
                         grid=grid_size)

    output_images = np.empty((num_images, new_height, new_width, 3), dtype=np.uint8)
    cuda.memcpy_dtoh(output_images, output_gpu)

    input_gpu.free()
    output_gpu.free()

    return output_images


def process_images(image_list, scale_factors):
    images = np.array(image_list)
    resized_images_list = [{factor: resize_images_gpu(images, 1.0 / factor) for factor in scale_factors}]
    return resized_images_list


# 模拟接收图像流
def receive_images(num_images, width, height):
    # 生成一些随机图像作为示例
    images = [np.random.randint(0, 256, (height, width, 3), dtype=np.uint8) for _ in range(num_images)]
    return images


# 主函数
def main():
    # 每秒接收30张图片
    num_images = 30
    width, height = 928, 800
    scale_factors = [2, 4, 8, 16]

    images = receive_images(num_images, width, height)

    start_time = time.time()
    resized_images_list = process_images(images, scale_factors)
    end_time = time.time()

    elapsed_time = end_time - start_time
    print(f"Processing time: {elapsed_time:.3f} seconds")

    # 打印每个缩放因子的第一个图像的形状以验证结果
    for factor in scale_factors:
        print(f"Scale factor 1/{factor}: {resized_images_list[0][factor][0].shape}")


if __name__ == "__main__":
    main()
