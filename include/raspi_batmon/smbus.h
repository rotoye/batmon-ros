#pragma once

#define I2CADDRESS1 0x0E // x = (not filled) y = (not filled) (Default for SMbus smart batteries)
#define I2CADDRESS2 0x0E // x = (not filled) y = (filled)
#define I2CADDRESS3 0x0E // x = (filled) y = (not filled)
#define I2CADDRESS4 0x0E // x = (filled) y = (filled)

//SMBUS Register enumeration
#define SMBUS_VOLTAGE 0x09                    // <Total volt						>	<uint16> <format mV >	<WordRead>
#define SMBUS_CURRENT 0x0a                    // <Current							>	<int16_t> <format mA>	<WordRead>
#define SMBUS_AVG_CURRENT 0x0b                    // Not implemented
#define SMBUS_TEMP 0x08                        // <Battery Temperature				>	<uint16> <format deciKelvin >	<WordRead>
#define SMBUS_MAN_NAME 0x20                    // <Manufacturer Name "Rotoye"		>	<char*> <format	>	<BlockRead>
#define SMBUS_MAN_DATE 0x1b
#define SMBUS_SERIAL_NUM 0x1c
#define SMBUS_RUN_TIME_TO_EMPTY 0x11                // Not implemented
#define SMBUS_AVG_TIME_TO_EMPTY 0x12                // Not implemented
#define SMBUS_RELATIVE_SOC 0x0d                    // <Remaining capacity				>	<uint16> <format %  >	<WordRead>
#define SMBUS_REMAIN_CAP 0x0f                    // <Remaining capacity  			>	<uint16> <format mAh>	<WordRead>
#define SMBUS_FULL_CAP 0x10                    // <Full capacity					>	<uint16> <format mAh>	<WordRead>
#define SMBUS_CYCLE_COUNT 0x17                    // <Number of cycles on the battery	>	<uint16> <format num>	<WordRead>
#define SMBUS_VCELL1 0x3f                    // <Cell Volt						>	<uint16> <format mV >	<WordRead>
#define SMBUS_VCELL2 0x3e                    // Same as above
#define SMBUS_VCELL3 0x3d                    // Same as above
#define SMBUS_VCELL4 0x3c                    // Same as above
#define SMBUS_VCELL5 0x3b                    // Same as above
#define SMBUS_VCELL6 0x3a                    // Same as above
#define SMBUS_VCELL7 0x39                    // Same as above
#define SMBUS_VCELL8 0x38                    // Same as above
#define SMBUS_VCELL9 0x37                    // Same as above
#define SMBUS_VCELL10 0x36                    // Same as above
#define SMBUS_CELL_COUNT 0x40                    // <Cell Volt						>	<uint16> <format num>	<WordRead>
#define SMBUS_SAFETY_STATUS 0x51                // <SafetyStatus structure below	>	<ByteArray> <format SafetyStatus>	<BlockRead>
#define SMBUS_ALERT_STATUS 0x50                    // Not implemented

//EEPROM parameter addresses
#define EEPROM_SHUNT_VAL_SET 0
#define EEPROM_CAPACITY_SET 2

//Defines for BATMON specific I2C functionality : Not using this now.
#define BATMON_MAIN_RESET_ADDRESS 0x90
#define BATMON_BQ_RESET_ADDRESS 0x91
#define BATMON_SHUNT_VAL_SET_ADDRESS 0x92
#define BATMON_CAPACITY_SET_ADDRESS 0x93

// STATUS messages 
#define BATMON_NOT_BOOTED 0x43
#define ADC_CHIP_CONNECTION_ERROR 0x44
#define ADC_CANT_WRITE_CHIP 0x45
#define ADC_CHIP_NOT_FOUND 0x46
#define ADC_I2C_ERROR 0x47
#define BATMON_READY 0x48
#define DEF_ERROR 0x49
#define BATMON_SLEEPING 0x40


//ROS battery constants

//Power supply status constants
const int POWER_SUPPLY_STATUS_UNKNOWN = 0u;
const int POWER_SUPPLY_STATUS_CHARGING = 1u;
const int POWER_SUPPLY_STATUS_DISCHARGING = 2u;
const int POWER_SUPPLY_STATUS_NOT_CHARGING = 3u;
const int POWER_SUPPLY_STATUS_FULL = 4u;

//Power supply health constants
const int POWER_SUPPLY_HEALTH_UNKNOWN = 0u;
const int POWER_SUPPLY_HEALTH_GOOD = 1u;
const int POWER_SUPPLY_HEALTH_OVERHEAT = 2u;
const int POWER_SUPPLY_HEALTH_DEAD = 3u;
const int POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4u;
const int POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5u;
const int POWER_SUPPLY_HEALTH_COLD = 6u;
const int POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7u;
const int POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8u;

//Power supply technology (chemistry) constants
const int POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0u;
const int POWER_SUPPLY_TECHNOLOGY_NIMH = 1u;
const int POWER_SUPPLY_TECHNOLOGY_LION = 2u;
const int POWER_SUPPLY_TECHNOLOGY_LIPO = 3u;
const int POWER_SUPPLY_TECHNOLOGY_LIFE = 4u;
const int POWER_SUPPLY_TECHNOLOGY_NICD = 5u;
const int POWER_SUPPLY_TECHNOLOGY_LIMN = 6u;
