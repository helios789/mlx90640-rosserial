#include "mbed.h"
#include "MLX90640_rosserial.h"


int main()
{
    mlx90640_rosserial mlxHandler;
    mlxHandler.publish();

    return 0;
}