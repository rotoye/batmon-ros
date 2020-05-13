#include <raspi_batmon/batmon_driver.h>

/**
 * batmon_driver class that handles the i2c communication
 * @author Nico Bartholomai
 * @param adapter_nr the adapter number, ex 3 from /dev/i2c-3
 * @param addr the address of the i2c port from 'i2cdetect -y 3'
 */
batmon_driver::batmon_driver(int adapter_nr, int addr) {
    file_num = adapter_nr;
    this->addr = addr;
}

/**
 * Reads a 16bit word using smbus
 * @param reg the register to read the word from
 * @return contents of the register as a 32bit int
 */
__s32 batmon_driver::read_16bit_word(int reg) {
    int file;
    char filename[20];
    //------ Open I2C BUS ------

    snprintf(filename, 19, "/dev/i2c-%d", file_num);

    file = open(filename, O_RDWR);

    if (file < 0) {
        ROS_ERROR("Failed to open the i2c bus.\n");
        close(file);
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        ROS_ERROR("Failed to acquire buc access.\n");
        close(file);
        return -1;
    }

    //------ READ BYTES with SMBus ----
    __s32 result_volt = i2c_smbus_read_word_data(file, reg);
    if (result_volt < 0) {
        ROS_ERROR("Failed to read from register: 0x%02X", reg);
        close(file);
        return -1;
    }
    close(file);
    return result_volt;
}

/**
 * gets the voltage
 * @return voltage in volts
 */
float batmon_driver::get_voltage() {
    // divide by 1000 to convert mV to V
    return read_16bit_word(SMBUS_VOLTAGE) / ((float) 1000);
}

/**
 * gets the current
 * @return current in Ah
 */
float batmon_driver::get_current() {
    __s16 curr = read_16bit_word(SMBUS_CURRENT);
    //divide by 1000 to convert mAh to Ah
    return curr / ((float) 1000);
}

/**
 * Gets the charge
 * @return charge in ah
 */
float batmon_driver::get_charge() {
    //divide by 1000 to convert mAh to Ah
    return read_16bit_word(SMBUS_REMAIN_CAP) / ((float) 1000);
}

/**
 * Gets the current max capacity
 * @return capacity in Ah
 */
float batmon_driver::get_capacity() {
    //divide by 1000 to convert mAh to Ah
    return read_16bit_word(SMBUS_FULL_CAP) / ((float) 1000);
}

/**
 * Gets the battery's percentage
 * @return the percentage from 0 to 1
 */
float batmon_driver::get_percentage() {
    int charge_rem = read_16bit_word(SMBUS_REMAIN_CAP);
    int charge_tot = read_16bit_word(SMBUS_FULL_CAP);
    //calculates the battery percentage
    if (charge_rem == -1 || charge_tot == -1)
        return -1;
    else
        return charge_rem > charge_tot ? (float) 1 : (float) charge_rem / ((float) charge_tot);
}

/**
 * Gets the battery's status
 * @return battery's status
 */
unsigned int batmon_driver::get_status() {
    //todo: wait until feature becomes available
    return POWER_SUPPLY_STATUS_UNKNOWN;
}

/**
 * Gets the battery's health
 * @return battery's health
 */
unsigned int batmon_driver::get_health() {
    //todo: wait until feature becomes available
    return POWER_SUPPLY_HEALTH_UNKNOWN;
}

/**
 * Gets the type of battery tech being used
 * @return battery's technology
 */
unsigned int batmon_driver::get_technology() {
    //todo: wait unil feat becomes available
    return POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

/**
 * Says if the battery is present
 * @return true if present false otherwise
 */
bool batmon_driver::is_present() {
    return get_voltage() != -1;
}

/**
 * gets the voltages of each cell in the battery
 * @return an array (vector) of all the voltages in the cells
 */
std::vector<float> batmon_driver::get_cell_voltage() {
    int num_cells = read_16bit_word(SMBUS_CELL_COUNT);
    std::vector<float> cell_volts(num_cells);
    int addr = SMBUS_VCELL1;
    for (int i = 0; i < num_cells; i++) {
        float value = read_16bit_word(addr--);
        //divide by 1000 to convert mV to V
        cell_volts[i] = value / ((float) 1000);
    }
    return cell_volts;
}

/**
 * Gets the serial number of the battery
 * @return the serial number as a string
 */
std::string batmon_driver::get_serialnumber() {
    int inStr = read_16bit_word(SMBUS_SERIAL_NUM);
    std::stringstream ss;
    ss << inStr;
    return ss.str();
}
