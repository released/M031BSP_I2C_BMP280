# M031BSP_I2C_BMP280
 M031BSP_I2C_BMP280

update @ 2021/11/24

1. use I2C0 (PC1 : SCL , PC0 : SDA) , to drive pressure sensor BMP280  

2. BMP280 , 

	PIN#2 (CSB) : HIGH , CSB to VDDIO : enable I2C
	
	PIN#5 (SDO) : GND , SDO to GND : address 1110110 (0x76)

3. below is terminal read sensor log 

![image](https://github.com/released/M031BSP_I2C_BMP280/blob/main/log.jpg)

