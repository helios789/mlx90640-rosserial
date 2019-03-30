#pragma once
#include "MLX90640.h"
#include <ros.h>
#include "rtos.h"
#include <std_msgs/Float32MultiArray.h>

// rosserial msg 의 제한된 메시지 길이 때문에
// 24 x 32 = 768 개의 mlx90640 픽셀을 4부분으로 나눠서 192 픽셀씩 rosserial로 전송
const u_int16_t MLX90640_MAX_SIZE = 768;
const u_int16_t MSG_NUM = 4;
const u_int16_t MSG_LEN = MLX90640_MAX_SIZE / MSG_NUM;
const char* MSG_TOPIC_NAME[MSG_NUM] = {"mlx90640-part-1", "mlx90640-part-2", "mlx90640-part-3", "mlx90640-part-4"};
Mutex mutex;

using namespace ros;

class mlx90640_rosserial
{
private:
    mlx90640 _mlx90640;

    NodeHandle nh;
    Publisher *publisher[MSG_NUM];
    std_msgs::Float32MultiArray msg[MSG_NUM];

public:
    mlx90640_rosserial()
    {
        nh.initNode();
        nh.getHardware()->setBaud(115200);
        
        for(int i = 0; i < MSG_NUM; i++)
        {
            // rosseial message setting
            msg[i].data_length = MSG_LEN;
            msg[i].data = new float[MSG_LEN];

            // node publisher setting
            publisher[i] = new Publisher(MSG_TOPIC_NAME[i], &msg[i]);
            nh.advertise(*publisher[i]);
        }
    }

    void publish()
    {
        Thread getTemperatureThread (getTemperature_thread);

        for(int i = 0; i < MSG_NUM; i++)
        {
            Thread msgThread (msg_thread(i));
        }
    }


    void getTemperature_thread(void const *args)
    {
        while(1)
        {
            mutex.lock();

            _mlx90640.getTemperatureFromSensor();
            nh.spinOnce();
            
            mutex.unlock();
        }
    }

    void msg_thread(u_int16_t part)
    {
        while(1)
        {
            for(int i = 0; i < MSG_LEN; i++)
            {
                // get 192 pixel data from mlx90640 sensor
                msg[part].data[i] = _mlx90640.mlx90640To[i + (MSG_LEN * part)];
            }
            publisher[part]->publish(&msg[part]);
        }
    }

    ~mlx90640_rosserial()
    {
        for(int i = 0; i < MSG_NUM; i++)
        {
            delete   publisher[i];
            delete[] msg[i].data;
        }
    }
}