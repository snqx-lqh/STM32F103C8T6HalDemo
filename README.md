# STM32F103C8T6HalDemo
这个库将用来保存一些使用STM32F103C8T6核心板驱动的外设，包括MPU6050等，将持续更新。

## 文件项目说明

00_Project：基础模板，后面的内容会根据他衍生

01_MPU6050：使用卡尔曼滤波、互补滤波、MahonyAHRS、MadgwickAHRS算法、DMP解算陀螺仪姿态。

​							移植讲解的博客：https://blog.csdn.net/wan1234512/article/details/146561546

02_MPU9250：使用卡尔曼滤波、互补滤波、MahonyAHRS、MadgwickAHRS算法、DMP解算陀螺仪姿态。

​							移植讲解的博客：https://blog.csdn.net/wan1234512/article/details/146562002?spm=1011.2124.3001.6209

03_NRF24L01：基于正点原子开源代码，添加部分接口。

04_MAX31850：移植arduino的OneWire库，实现单总线协议的读取PT100测量芯片的值。

05_PCF8591：使用AD/DA芯片，该芯片使用的是IIC控制芯片读取ADC和DA输出。

06_DS18B20：移植arduino的OneWire库，实现单总线协议的读取DS18B20测量芯片的值。
