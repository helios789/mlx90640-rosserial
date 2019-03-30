#pragma once
#include "mbed.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define TA_SHIFT 8
//the default shift for a MLX90640 device in open air

class mlx90640
{
public:
    static float mlx90640To[768];   // 24 x 32 temperature array
    paramsMLX90640 mlx90640device;        


public:
    // constructor
    mlx90640()
    {
        MLX90640_I2CInit();
        MLX90640_I2CFreqSet(400);

        int status;
        uint16_t eeMLX90640[832];
        status = MLX90640_DumpEE(0x33, eeMLX90640);
        
        if (status != 0)
        {
            // TODO : 오류 예외처리 구현
        }
        status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640device);
        
        if (status != 0)
        {
            // TODO : 오류 예외처리 구현
        }
    }

    void getTemperatureFromSensor()
    {
        for (int x = 0 ; x < 2 ; x++) //Read both subpages
        {
            uint16_t mlx90640Frame[834];
            int status = MLX90640_GetFrameData(0x33, mlx90640Frame);
            
            if (status < 0)
            {
                // TODO : 오류 예외처리 구현
            }
        
            float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640device);
            float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640device);
        
            float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
            float emissivity = 0.95;
        
            MLX90640_CalculateTo(mlx90640Frame, &mlx90640device, emissivity, tr, mlx90640To);
        }
    }
}