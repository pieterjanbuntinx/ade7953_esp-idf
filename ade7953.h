#ifndef __ADE7953__
#define __ADE7953__

#include <stdint.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#ifdef CONFIG_ADE7953_COMMS_PROT_SPI
#include <driver/spi_master.h>
#endif

#define ADE7953_PREF              1540  // 4194304 / (1540 / 1000) = 2723574 (= WGAIN, VAGAIN and VARGAIN)
#define ADE7953_UREF              26000 // 4194304 / (26000 / 10000) = 1613194 (= VGAIN)
#define ADE7953_IREF              10000 // 4194304 / (10000 / 10000) = 4194303 (= IGAIN, needs to be different than 4194304 in order to use calib.dat)
#define ADE7953_NO_LOAD_THRESHOLD 29196 // According to ADE7953 datasheet the default threshold for no load detection is 58,393 use half this value to measure lower (5w) powers.
#define ADE7953_NO_LOAD_ENABLE    0     // Set DISNOLOAD register to 0 to enable No-load detection
#define ADE7953_NO_LOAD_DISABLE   7     // Set DISNOLOAD register to 7 to disable No-load detection

// Default calibration parameters can be overridden by a rule as documented above.
#define ADE7953_GAIN_DEFAULT     0x400000 // = 0x400000 range 2097152 (min) to 6291456 (max)
#define ADE7953_PHCAL_DEFAULT    0        // = range -383 to 383 - Default phase calibration for Shunts
#define ADE7953_PHCAL_DEFAULT_CT 200      // = range -383 to 383 - Default phase calibration for Current Transformers (Shelly EM)

#define ADE7953_MAX_CHANNEL 2

#define ADE7953_CALIBREGS 6
#define ADE7953_REGISTERS 6

enum Ade7953_8BitRegisters
{
    // Register Name                    Addres  R/W  Bt  Ty  Default     Description
    // ----------------------------     ------  ---  --  --  ----------  --------------------------------------------------------------------
    ADE7953_SAGCYC = 0x000,      //      0x000   R/W  8   U   0x00        Sag line cycles
    ADE7953_DISNOLOAD,           //      0x001   R/W  8   U   0x00        No-load detection disable (see Table 16)
    ADE7953_RESERVED_0X002,      //      0x002
    ADE7953_RESERVED_0X003,      //      0x003
    ADE7953_LCYCMODE,            //      0x004   R/W  8   U   0x40        Line cycle accumulation mode configuration (see Table 17)
    ADE7953_RESERVED_0X005,      //      0x005
    ADE7953_RESERVED_0X006,      //      0x006
    ADE7953_PGA_V,               //      0x007   R/W  8   U   0x00        Voltage channel gain configuration (Bits[2:0])
    ADE7953_PGA_IA,              //      0x008   R/W  8   U   0x00        Current Channel A gain configuration (Bits[2:0])
    ADE7953_PGA_IB,              //      0x009   R/W  8   U   0x00        Current Channel B gain configuration (Bits[2:0])
    ADE7953_WRITE_PROT  = 0x040, //      0x040   R/W  8   U   0x00        Current Channel B gain configuration (Bits[2:0])
    ADE7953_LAST_OP     = 0x0FD, //      0x0FD   R/W  8   U   0x00        Current Channel B gain configuration (Bits[2:0])
    ADE7953_LAST_RWDATA = 0x0FF, //      0x0FF   R/W  8   U   0x00        Current Channel B gain configuration (Bits[2:0])
    ADE7953_VERSION     = 0x702, //      0x702   R/W  8   U   0x00        Current Channel B gain configuration (Bits[2:0])
    ADE7953_EX_REF      = 0x800, //      0x800   R/W  8   U   0x00        Current Channel B gain configuration (Bits[2:0])

};

enum Ade7953_16BitRegisters
{
    // Register Name                    Addres  R/W  Bt  Ty  Default     Description
    // ----------------------------     ------  ---  --  --  ----------  --------------------------------------------------------------------
    ADE7953_ZXTOUT = 0x100, //          0x100   R/W  16  U   0xFFFF      Zero-crossing timeout
    ADE7953_LINECYC,        //          0x101   R/W  16  U   0x0000      Number of half line cycles for line cycle energy accumulation mode
    ADE7953_CONFIG,         //          0x102   R/W  16  U   0x8004      Configuration register (see Table 18)
    ADE7953_CF1DEN,         //          0x103   R/W  16  U   0x003F      CF1 frequency divider denominator. When modifying this register, two sequential write operations must be performed to ensure that the write is successful.
    ADE7953_CF2DEN,         //          0x104   R/W  16  U   0x003F      CF2 frequency divider denominator. When modifying this register, two sequential write operations must be performed to ensure that the write is successful.
    ADE7953_RESERVED_0X105, //          0x105
    ADE7953_RESERVED_0X106, //          0x106
    ADE7953_CFMODE,         //          0x107   R/W  16  U   0x0300      CF output selection (see Table 19)
    ADE7943_PHCALA,         //          0x108   R/W  16  S   0x0000      Phase calibration register (Current Channel A). This register is in sign magnitude format.
    ADE7943_PHCALB,         //          0x109   R/W  16  S   0x0000      Phase calibration register (Current Channel B). This register is in sign magnitude format.
    ADE7943_PFA,            //          0x10A   R    16  S   0x0000      Power factor (Current Channel A)
    ADE7943_PFB,            //          0x10B   R    16  S   0x0000      Power factor (Current Channel B)
    ADE7943_ANGLE_A,        //          0x10C   R    16  S   0x0000      Angle between the voltage input and the Current Channel A input
    ADE7943_ANGLE_B,        //          0x10D   R    16  S   0x0000      Angle between the voltage input and the Current Channel B input
    ADE7943_Period,         //          0x10E   R    16  U   0x0000      Period register

    ADE7953_RESERVED_0X120 =
        0x120 // 0x120                            This register should be set to 30h to meet the performance specified in Table 1. To modify this register, it must be unlocked by setting Register Address 0xFE to 0xAD immediately prior.
};

enum Ade7953_32BitRegisters
{
    // Register Name                    Addres  R/W  Bt  Ty  Default     Description
    // ----------------------------     ------  ---  --  --  ----------  --------------------------------------------------------------------
    ADE7953_ACCMODE = 0x301, //         0x301   R/W  24  U   0x000000    Accumulation mode (see Table 21)

    ADE7953_AP_NOLOAD = 0x303, //       0x303   R/W  24  U   0x00E419    No load threshold for actual power
    ADE7953_VAR_NOLOAD,        //       0x304   R/W  24  U   0x00E419    No load threshold for reactive power
    ADE7953_VA_NOLOAD,         //       0x305   R/W  24  U   0x000000    No load threshold for appearant power

    ADE7953_AVA = 0x310,    //          0x310   R    24  S   0x000000    Instantaneous apparent power (Current Channel A)
    ADE7953_BVA,            //          0x311   R    24  S   0x000000    Instantaneous apparent power (Current Channel B)
    ADE7953_AWATT,          //          0x312   R    24  S   0x000000    Instantaneous active power (Current Channel A)
    ADE7953_BWATT,          //          0x313   R    24  S   0x000000    Instantaneous active power (Current Channel B)
    ADE7953_AVAR,           //          0x314   R    24  S   0x000000    Instantaneous reactive power (Current Channel A)
    ADE7953_BVAR,           //          0x315   R    24  S   0x000000    Instantaneous reactive power (Current Channel B)
    ADE7953_IA,             //          0x316   R    24  S   0x000000    Instantaneous current (Current Channel A)
    ADE7953_IB,             //          0x317   R    24  S   0x000000    Instantaneous current (Current Channel B)
    ADE7953_V,              //          0x318   R    24  S   0x000000    Instantaneous voltage (voltage channel)
    ADE7953_RESERVED_0X319, //          0x319
    ADE7953_IRMSA,          //          0x31A   R    24  U   0x000000    IRMS register (Current Channel A)
    ADE7953_IRMSB,          //          0x31B   R    24  U   0x000000    IRMS register (Current Channel B)
    ADE7953_VRMS,           //          0x31C   R    24  U   0x000000    VRMS register
    ADE7953_RESERVED_0X31D, //          0x31D
    ADE7953_AENERGYA,       //          0x31E   R    24  S   0x000000    Active energy (Current Channel A)
    ADE7953_AENERGYB,       //          0x31F   R    24  S   0x000000    Active energy (Current Channel B)
    ADE7953_RENERGYA,       //          0x320   R    24  S   0x000000    Reactive energy (Current Channel A)
    ADE7953_RENERGYB,       //          0x321   R    24  S   0x000000    Reactive energy (Current Channel B)
    ADE7953_APENERGYA,      //          0x322   R    24  S   0x000000    Apparent energy (Current Channel A)
    ADE7953_APENERGYB,      //          0x323   R    24  S   0x000000    Apparent energy (Current Channel B)
    ADE7953_OVLVL,          //          0x324   R/W  24  U   0xFFFFFF    Overvoltage level
    ADE7953_OILVL,          //          0x325   R/W  24  U   0xFFFFFF    Overcurrent level
    ADE7953_VPEAK,          //          0x326   R    24  U   0x000000    Voltage channel peak
    ADE7953_RSTVPEAK,       //          0x327   R    24  U   0x000000    Read voltage peak with reset
    ADE7953_IAPEAK,         //          0x328   R    24  U   0x000000    Current Channel A peak
    ADE7953_RSTIAPEAK,      //          0x329   R    24  U   0x000000    Read Current Channel A peak with reset
    ADE7953_IBPEAK,         //          0x32A   R    24  U   0x000000    Current Channel B peak
    ADE7953_RSTIBPEAK,      //          0x32B   R    24  U   0x000000    Read Current Channel B peak with reset
    ADE7953_IRQENA,         //          0x32C   R/W  24  U   0x100000    Interrupt enable (Current Channel A, see Table 22)
    ADE7953_IRQSTATA,       //          0x32D   R    24  U   0x000000    Interrupt status (Current Channel A, see Table 23)
    ADE7953_RSTIRQSTATA,    //          0x32E   R    24  U   0x000000    Reset interrupt status (Current Channel A)
    ADE7953_IRQENB,         //          0x32F   R/W  24  U   0x000000    Interrupt enable (Current Channel B, see Table 24)
    ADE7953_IRQSTATB,       //          0x330   R    24  U   0x000000    Interrupt status (Current Channel B, see Table 25)
    ADE7953_RSTIRQSTATB,    //          0x331   R    24  U   0x000000    Reset interrupt status (Current Channel B)

    ADE7953_CRC = 0x37F,    //          0x37F   R    32  U   0xFFFFFFFF  Checksum
    ADE7953_AIGAIN,         //          0x380   R/W  24  U   0x400000    Current channel gain (Current Channel A)
    ADE7953_AVGAIN,         //          0x381   R/W  24  U   0x400000    Voltage channel gain
    ADE7953_AWGAIN,         //          0x382   R/W  24  U   0x400000    Active power gain (Current Channel A)
    ADE7953_AVARGAIN,       //          0x383   R/W  24  U   0x400000    Reactive power gain (Current Channel A)
    ADE7953_AVAGAIN,        //          0x384   R/W  24  U   0x400000    Apparent power gain (Current Channel A)
    ADE7953_RESERVED_0X385, //          0x385
    ADE7953_AIRMSOS,        //          0x386   R/W  24  S   0x000000    IRMS offset (Current Channel A)
    ADE7953_RESERVED_0X387, //          0x387
    ADE7953_VRMSOS,         //          0x388   R/W  24  S   0x000000    VRMS offset
    ADE7953_AWATTOS,        //          0x389   R/W  24  S   0x000000    Active power offset correction (Current Channel A)
    ADE7953_AVAROS,         //          0x38A   R/W  24  S   0x000000    Reactive power offset correction (Current Channel A)
    ADE7953_AVAOS,          //          0x38B   R/W  24  S   0x000000    Apparent power offset correction (Current Channel A)
    ADE7953_BIGAIN,         //          0x38C   R/W  24  U   0x400000    Current channel gain (Current Channel B)
    ADE7953_BVGAIN,         //          0x38D   R/W  24  U   0x400000    This register should not be modified.
    ADE7953_BWGAIN,         //          0x38E   R/W  24  U   0x400000    Active power gain (Current Channel B)
    ADE7953_BVARGAIN,       //          0x38F   R/W  24  U   0x400000    Reactive power gain (Current Channel B)
    ADE7953_BVAGAIN,        //          0x390   R/W  24  U   0x400000    Apparent power gain (Current Channel B)
    ADE7953_RESERVED_0X391, //          0x391
    ADE7953_BIRMSOS,        //          0x392   R/W  24  S   0x000000    IRMS offset (Current Channel B)
    ADE7953_RESERVED_0X393, //          0x393
    ADE7953_RESERVED_0X394, //          0x394
    ADE7953_BWATTOS,        //          0x395   R/W  24  S   0x000000    Active power offset correction (Current Channel B)
    ADE7953_BVAROS,         //          0x396   R/W  24  S   0x000000    Reactive power offset correction (Current Channel B)
    ADE7953_BVAOS           //          0x397   R/W  24  S   0x000000    Apparent power offset correction (Current Channel B)
};

enum Ade7953CalibrationRegisters
{
    ADE7953_CAL_VGAIN,
    ADE7953_CAL_IGAIN,
    ADE7953_CAL_WGAIN,
    ADE7953_CAL_VAGAIN,
    ADE7953_CAL_VARGAIN,
    ADE7943_CAL_PHCAL
};

typedef enum
{
    ADE7953_SHELLY_25,
    ADE7953_SHELLY_EM,
    ADE7953_SHELLY_PLUS_2PM,
    ADE7953_SHELLY_PRO_1PM,
    ADE7953_SHELLY_PRO_2PM,
    ADE7953_SHELLY_PRO_4PM
} ShellyModels_t;

typedef struct {
    float  voltage;
    float  current;
    float  activePower;
    double activeEnergy;
} ade7953_data_t;

typedef struct {
    bool enabled;
#ifdef CONFIG_ADE7953_COMMS_PROT_SPI
    spi_device_handle_t spi_handle;
#else
    int i2c_port_number;
#endif
    uint8_t        addr;
    int32_t        calib_data[ADE7953_MAX_CHANNEL][ADE7953_CALIBREGS];
    char           chip_num;
    ade7953_data_t data[ADE7953_MAX_CHANNEL];
    bool           saveEnergiesToNvs;
} __attribute__((packed)) ade7953_t;

void    ade7953_init(ade7953_t* ade7953, ShellyModels_t model);
int32_t ade7953_readReg(ade7953_t* ade7953, uint16_t reg, TickType_t ticks_to_wait);
void    ade7953_writeReg(ade7953_t* ade7953, uint16_t reg, int32_t data, TickType_t ticks_to_wait);
void    ade7953_setDefaultsForShellyDevice(ade7953_t* ade7953, ShellyModels_t model);
void    ade7953_setCalibrationForShellyDevice(ade7953_t* ade7953, ShellyModels_t model);
void    ade7953_retrieveCalibrationFromNvs(ade7953_t* ade7953);
void    ade7953_setCalibrationForShellyDevice(ade7953_t* ade7953, ShellyModels_t model);

#endif /* __ADE7953__ */
