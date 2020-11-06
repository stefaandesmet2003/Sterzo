a proof of concept STERZO imitation based on a nrf51822 smart watch <br>

  to get going:
  - modify variant.h to suit the pin layout of your smartwatch,
    in particular I2C for the accelerometer, and Serial TX for the Serial.prints to work
  - install Arduino NRF51 board package & BLEPeripheral library
  - select the device settings corresponding with the smartwatch : ram/flash (xxaa), oscillator(RC)
  - hookup a ST-LINK for programming
  - flash a Nordic SoftDevice (S110 works for me)
  - program arduino sketch

  Have a look at my repo https://github.com/stefaandesmet2003/nrf51 for more details