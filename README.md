# Furtail
An animatronic tail for cosplay use

Servo pins are on D5 and D11 on Huaduino or normal arduino UNO/NANO.

For IMU version, it uses MPU6050 which use D2 as interrupt pin and A4 A5 as I2C port.

MPU6050 lib is needed. https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

You might need to get the IMU offsets first and change them in the IMU code,then download it to the chip. So that it will work properly.

You can download the IMU code into the chip first, open the Serial monitor on PC (Arduino IDE), wait for the offsets data show up.


For bluetooth (BT) version, it uses HC-05 BT module to communicate to the chip through hardware Serial on UNO/NANO (tx0 rx0).

After connected the BT module(you can use bt-serial helper on smartphone), send numbers to make different moves, check the code for more detailed informations.

