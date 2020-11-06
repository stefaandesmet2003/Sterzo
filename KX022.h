#ifndef KX022_H
#define KX022_H

#include <Arduino.h>
#include <Wire.h>

#define KX022_ADDR_L 0x1E
#define KX022_ADDR_H 0x1F
#define KX022_ADDR_RESET 0x1D  // ? I cant find the reference to this

#define KX022_RANGE_2G 0b00000000  // range +/-2g
#define KX022_RANGE_4G 0b00001000  // range +/-4g
#define KX022_RANGE_8G 0b00011000  // range +/-8g


// KX022 registers
#define KX022_XHPL            0x00
#define KX022_XHPH            0x01
#define KX022_YHPL            0x02
#define KX022_YHPH            0x03
#define KX022_ZHPL            0x04
#define KX022_ZHPH            0x05
#define KX022_XOUTL           0x06
#define KX022_XOUTH           0x07
#define KX022_YOUTL           0x08
#define KX022_YOUTH           0x09
#define KX022_ZOUTL           0x0A
#define KX022_ZOUTH           0x0B
#define KX022_COTR            0x0C
#define KX022_WHOAMI          0x0F
#define KX022_TSCP            0x10 // current tilt position
#define KX022_TSPP            0x11 // previous tilt position
#define KX022_INS1            0x12 // interrupt source
#define KX022_INS2            0x13
#define KX022_INS3            0x14
#define KX022_STAT            0x15 // status of interrupt
#define KX022_INT_REL         0x17 // read to clear interrupt
#define KX022_CNTL1           0x18 // control register 1
#define KX022_CNTL2           0x19 // control register 2
#define KX022_CNTL3           0x1A // control register 3
#define KX022_ODCNTL          0x1B // output data rate & filter settings
#define KX022_INC1            0x1C // interrupt control
#define KX022_INC2            0x1D
#define KX022_INC3            0x1E
#define KX022_INC4            0x1F
#define KX022_INC5            0x20
#define KX022_INC6            0x21
#define KX022_TILT_TIMER      0x22 // initial count for tilt position state timer (qualify a change in orientation)
#define KX022_WUFC            0x23 // initial count for motion detection timer
#define KX022_TDTRC           0x24 // tap/double tap int enable
#define KX022_TDTC            0x25 // minimum separation between 2 taps of a double tap
#define KX022_TTH             0x26 // tap threshold high
#define KX022_TTL             0x27 // tap threshold low
#define KX022_FTD             0x28 // timing for a tap event
#define KX022_STD             0x29 // timing for a double tap event
#define KX022_TLT             0x2A // tap latency
#define KX022_TWS             0x2B // tap/double tap window counter
#define KX022_ATH             0x30 // wake-up threshold acceleration
#define KX022_TILT_ANGLE_LL   0x32 // low level (change in hor/vert only possible when tilt angle above this threshold)
#define KX022_TILT_ANGLE_HL   0x33 // high level
#define KX022_HYST_SET        0x34 // hysteresis between tilt states
#define KX022_LP_CNTL         0x35 // number of samples averaged in low power mode
#define KX022_BUF_CNTL1       0x3A // sample buffer control - threshold
#define KX022_BUF_CNTL2       0x3B // sample buffer control
#define KX022_BUF_STATUS_1    0x3C // sample buffer status
#define KX022_BUF_STATUS_2    0x3D // sample buffer status - trigger function
#define KX022_BUF_CLEAR       0x3E // clear sample buffer
#define KX022_BUF_READ        0x3F // buffer output register
#define KX022_SELF_TEST       0x60

#define DATA_OUT_BASE         0x06 // read 6 bytes from here to get the 3 acc axes data

// defaults
#define KX022_WHOAMI_RESPONSE 0x14
#define KX022_CNTL1_VALUE_STANDBY 0x41
#define KX022_ODCNTL_VALUE 0x02
#define KX022_CNTL3_VALUE 0xD8
#define KX022_TILT_TIMER_VALUE 0x01
#define KX022_CNTL1_VALUE_OPERATE 0xC1

// register KX022_CNTL1 */
#define CNTL1_PC1_STANDBY     (0x00)
#define CNTL1_PC1_OPERATING   (0x80)
#define CNTL1_PC1_MASK        (0x80)
#define CNTL1_RES_HIGHCURRENT (0x40)
#define CNTL1_DRDYE           (0x20)
#define CNTL1_GSEL_RANGE_2G   (0x00)
#define CNTL1_GSEL_RANGE_4G   (0x08)
#define CNTL1_GSEL_RANGE_8G   (0x18)
#define CNTL1_GSEL_MASK       (0x18)
#define CNTL1_TDTE            (0x04)
#define CNTL1_WUFE            (0x02)
#define CNTL1_TPE             (0x01)

// CNTL2-3
#define CNTL2_DEFAULT         (0x3F)
#define CNTL3_DEFAULT         (0x98)

// ODCNTL
#define ODCNTL_OSA_MASK       (0xF)
#define ODCNTL_DEFAULT        (0x02)

// interrupts
#define INC1_IEN_ENABLED      (0x20)
#define INC1_IEA_ACTIVEHIGH   (0x10)
#define INC1_IEL_PINLATCHING  (0x00)
#define INC2_DEFAULT          (0x3F)
#define INC3_DEFAULT          (0x3F)
#define INC4_BFI1_ENABLE      (0x40)
#define INC4_WMI1_ENABLE      (0x20)
#define INC4_DRDYI1_ENABLE    (0x10)
#define INC4_TDTI1_ENABLE     (0x04)
#define INC4_WUFI1_ENABLE     (0x02)
#define INC4_TPI1_ENABLE      (0x01)
// not using PIN2 for the moment

#define INS2_BFI              (0x40)
#define INS2_WMI              (0x20)
#define INS2_DRDY             (0x10)
#define INS2_TDTS             (0x0C)
#define INS2_WUFS             (0x02)
#define INS2_TPS              (0x01)
#define INS2_NOTAP            (0x00)
#define INS2_SINGLETAP        (0x01 << 2)
#define INS2_DOUBLETAP        (0x02 << 2)


// masks
#define KX022_ACC_OUTPUT_RATE_MASK 0b00001111 // was 0x00001111
//#define KX022_RANGE_MASK 0b00011000

// tilt position
#define TILTPOSITION_LEFT        0x20
#define TILTPOSITION_RIGHT       0x10
#define TILTPOSITION_DOWN        0x8
#define TILTPOSITION_UP          0x4
#define TILTPOSITION_FRONTDOWN   0x2
#define TILTPOSITION_FRONTUP     0x1

// Note : 400Hz+ are not available in low power mode
enum KX022_ACC_OUTPUT_RATE
{
   KX022_ACC_OUTPUT_RATE_12_5_HZ,
   KX022_ACC_OUTPUT_RATE_25_HZ,
   KX022_ACC_OUTPUT_RATE_50_HZ,
   KX022_ACC_OUTPUT_RATE_100_HZ,
   KX022_ACC_OUTPUT_RATE_200_HZ,
   KX022_ACC_OUTPUT_RATE_400_HZ,
   KX022_ACC_OUTPUT_RATE_800_HZ,
   KX022_ACC_OUTPUT_RATE_1600_HZ,
   KX022_ACC_OUTPUT_RATE_0_781_HZ,
   KX022_ACC_OUTPUT_RATE_1_563_HZ,
   KX022_ACC_OUTPUT_RATE_3_125_HZ,
   KX022_ACC_OUTPUT_RATE_6_25_HZ
};

// device functions
#define KX022_FUNCTION_NONE            (0x0)
#define KX022_FUNCTION_TILT            (0x1)
#define KX022_FUNCTION_WAKEUP          (0x2)
#define KX022_FUNCTION_TAPDOUBLETAP    (0x4)


const float KX022_ODR[] = {12.5,  25.0,   50.0,  100.0, 200.0, 400.0,
                           800.0, 1600.0, 0.781, 1.563, 3.125, 6.25};

const float KX022_ACC_SENSITIVITY[] = {16384.0, 8192.0, 4096.0};

class KX022
{
  public:
   KX022( uint8_t i2c_address = KX022_ADDR_H) : _i2c_address(i2c_address), _range(KX022_RANGE_4G) { }

   // initialize the KX022 with basic settings
   void init(uint8_t range = KX022_RANGE_4G, uint8_t rate  = KX022_ACC_OUTPUT_RATE_50_HZ);

   // the device can signal an external interrupt
   // but handling the interrupts requires i2creading outside the EXTI irqHandler
   void poll(void);

   float getAccel(uint8_t channelNum);
   void getAccelXYZ(float (&xyz)[3]);
   uint8_t readRegister (uint8_t regAddress);
   void writeRegister (uint8_t regAddress, uint8_t regValue);

  protected:
   uint8_t _range;
   uint8_t _i2c_address;
   uint8_t _functions = 0;
   void getRawXYZ(int16_t (&xyz)[3], uint8_t base_reg_location = DATA_OUT_BASE);
};


#endif