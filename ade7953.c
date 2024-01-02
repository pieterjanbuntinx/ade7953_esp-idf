#include "ade7953.h"
#include <esp_err.h>
#include <string.h>
#include <nvs_flash.h>
#include <esp_timer.h>

#ifndef CONFIG_ADE7953_COMMS_PROT_SPI
#include <driver/i2c.h>
#endif

// #define DEBUG

const uint16_t Ade7953CalibRegs[2][ADE7953_CALIBREGS] = {{ADE7953_AVGAIN, ADE7953_AIGAIN, ADE7953_AWGAIN, ADE7953_AVAGAIN, ADE7953_AVARGAIN, ADE7943_PHCALA},
                                                         {ADE7953_BVGAIN, ADE7953_BIGAIN, ADE7953_BWGAIN, ADE7953_BVAGAIN, ADE7953_BVARGAIN, ADE7943_PHCALB}};

const float ADE7953_LSB_PER_WATTSECOND = 2.5;
const float ADE7953_POWER_CORRECTION   = 23.41494; // See https://github.com/arendst/Tasmota/pull/16941

static void ade7953_task(void* arg);

static size_t ade7953_getRegSize(uint16_t reg) {
    size_t size = 0;
    switch ((reg >> 8) & 0x0F) {
    case 0x03: // 32-bit
        size++;
    case 0x02: // 24-bit
        size++;
    case 0x01: // 16-bit
        size++;
    case 0x00: // 8-bit
    case 0x07:
    case 0x08:
        size++;
    }
    return size;
}

#ifdef CONFIG_ADE7953_COMMS_PROT_SPI
static esp_err_t readData(ade7953_t* ade7953, uint16_t reg, size_t len, void* data, TickType_t ticks_to_wait) {
#ifdef DEBUG
    printf("%s\n", __FUNCTION__);
#endif

    int      bufsize   = len + 3;
    uint8_t* buffer_tx = malloc(bufsize);
    memset(buffer_tx, 0x00, bufsize);
    uint16_t tempreg = __bswap16(reg);
    memcpy(buffer_tx, &tempreg, sizeof(tempreg));
    buffer_tx[2] = 0x80; // Read

    uint8_t* buffer_rx = malloc(bufsize);
    memset(buffer_rx, 0, bufsize);

    esp_err_t ret = spi_device_acquire_bus(ade7953->spi_handle, portMAX_DELAY);
    if (ret != ESP_OK) goto err;

#ifdef DEBUG
    printf("tx_buf: ");
    for (int i = 0; i < bufsize; i++) {
        printf("0x%02x ", buffer_tx[i]);
    }
    printf("\n");
#endif

    spi_transaction_t t = {
        .tx_buffer = buffer_tx,
        .rx_buffer = buffer_rx,
        .length    = bufsize * 8,
        .rxlength  = bufsize * 8,
    };
    ret = spi_device_polling_transmit(ade7953->spi_handle, &t);

#ifdef DEBUG
    printf("rx_buf: ");
    for (int i = 0; i < bufsize; i++) {
        printf("0x%02x ", buffer_rx[i]);
    }
    printf("\n");
#endif

    if (ret == ESP_OK) {
        memcpy(data, buffer_rx + 3, len);
    }

    spi_device_release_bus(ade7953->spi_handle);
err:
    if (buffer_rx != NULL) {
        free(buffer_rx);
    }
    if (buffer_tx != NULL) {
        free(buffer_tx);
    }
    return ret;
}
#else
static esp_err_t readData(ade7953_t* ade7953, uint16_t reg, size_t len, void* data, TickType_t ticks_to_wait) {
    uint16_t  reg_temp = ((reg >> 8) | (reg << 8)) & 0xFFFF;
    esp_err_t err      = i2c_master_write_read_device(ade7953->i2c_port_number, ade7953->addr, (uint8_t*)&reg_temp, 2, data, len, ticks_to_wait);
    vTaskDelay(pdMS_TO_TICKS(1));
    return err;
}
#endif

int32_t ade7953_readReg(ade7953_t* ade7953, uint16_t reg, TickType_t ticks_to_wait) {
    int32_t   data = 0;
    size_t    size = ade7953_getRegSize(reg);
    esp_err_t err  = readData(ade7953, reg, size, &data, ticks_to_wait);

#ifdef DEBUG
    printf("err: %s\n", esp_err_to_name(err));
#endif

    int32_t ret = 0;
    for (int i = 0; i < size; i++) {
        ret = ret << 8 | ((uint8_t*)&data)[i];
    }
    return ret;
}

#ifdef CONFIG_ADE7953_COMMS_PROT_SPI
esp_err_t writeData(ade7953_t* ade7953, uint16_t reg, size_t len, void* data, TickType_t ticks_to_wait) {

#ifdef DEBUG
    printf("%s\n", __FUNCTION__);
#endif

    int bufsize = len + 3;

    uint8_t* buffer_tx = malloc(bufsize);
    uint16_t tempreg   = __bswap16(reg);
    memcpy(buffer_tx, &tempreg, sizeof(tempreg));
    buffer_tx[2] = 0x00; // write
    for (int i = 0; i < len; i++) {
        *(buffer_tx + 3 + i) = ((uint8_t*)data)[len - 1 - i];
    }

    uint8_t* buffer_rx = malloc(bufsize);
    memset(buffer_rx, 0, bufsize);

    esp_err_t ret = spi_device_acquire_bus(ade7953->spi_handle, portMAX_DELAY);
    if (ret != ESP_OK) goto err;

#ifdef DEBUG
    printf("tx_buf: ");
    for (int i = 0; i < bufsize; i++) {
        printf("0x%02x ", buffer_tx[i]);
    }
    printf("\n");
#endif

    spi_transaction_t t = {
        .tx_buffer = buffer_tx,
        .rx_buffer = buffer_rx,
        .length    = bufsize * 8,
        .rxlength  = bufsize * 8,
    };
    ret = spi_device_polling_transmit(ade7953->spi_handle, &t);

#ifdef DEBUG
    printf("rx_buf: ");
    for (int i = 0; i < bufsize; i++) {
        printf("0x%02x ", buffer_rx[i]);
    }
    printf("\n");
#endif

    spi_device_release_bus(ade7953->spi_handle);
err:
    if (buffer_rx != NULL) {
        free(buffer_rx);
    }
    if (buffer_tx != NULL) {
        free(buffer_tx);
    }
    return ret;
}
#else
esp_err_t writeData(ade7953_t* ade7953, uint16_t reg, size_t len, void* data, TickType_t ticks_to_wait) {
    uint16_t reg_temp       = ((reg >> 8) | (reg << 8)) & 0xFFFF;
    uint8_t* data_with_addr = malloc(len + 2);
    memcpy(data_with_addr, &reg_temp, 2);

    // MSB first
    for (int i = 0; i < len; i++) {
        uint8_t* ptr          = data + (len - 1) - i;
        data_with_addr[2 + i] = *ptr;
    }

#ifdef DEBUG
    printf("reg: 0x%04x, len: %i, data: ", reg, len);
    for (int i = 0; i < len; i++) {
        printf("%02x", ((uint8_t*)data)[i]);
    }
    printf(" data_with_addr: ");
    for (int i = 0; i < len + 2; i++) {
        printf("%02x", ((uint8_t*)data_with_addr)[i]);
    }
    printf("\n");
#endif

    esp_err_t ret = i2c_master_write_to_device(ade7953->i2c_port_number, ade7953->addr, data_with_addr, len + 2, ticks_to_wait);
    vTaskDelay(pdMS_TO_TICKS(1));
    free(data_with_addr);
    return ret;
}
#endif

void ade7953_writeReg(ade7953_t* ade7953, uint16_t reg, int32_t data, TickType_t ticks_to_wait) {
    size_t size = ade7953_getRegSize(reg);
    writeData(ade7953, reg, size, &data, ticks_to_wait);

#ifdef DEBUG
    int32_t d = ade7953_readReg(ade7953, reg, pdMS_TO_TICKS(100));
    printf("read: %04x, data: ", reg);
    for (int i = 0; i < size; i++) {
        printf("%02x", ((uint8_t*)&d)[i]);
    }
    printf("\n");
#endif
}

void ade7953_setCalibration(ade7953_t* ade7953, uint32_t regset, uint32_t calibset) {
    for (uint32_t i = 0; i < ADE7953_CALIBREGS; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        int32_t value = ade7953->calib_data[calibset][i];
        if (ADE7943_CAL_PHCAL == i) {
            //      if (ADE7953_PHCAL_DEFAULT == value) { continue; }  // ADE7953
            //      reset does NOT always reset all registers
            if (value < 0) {
                value = abs(value) + 0x200; // Add sign magnitude
            }
        }
        //    if (ADE7953_GAIN_DEFAULT == value) { continue; }  // ADE7953 reset
        //    does NOT always reset all registers
        printf("calib %i %i %i val: %i\n", regset, calibset, i, value);
        ade7953_writeReg(ade7953, Ade7953CalibRegs[regset][i], value, pdMS_TO_TICKS(100));
    }
}

void ade7953_setDefaultsForShellyDevice(ade7953_t* ade7953, ShellyModels_t model) {
    for (uint32_t channel = 0; channel < ADE7953_MAX_CHANNEL; channel++) {
        for (uint32_t i = 0; i < ADE7953_CALIBREGS; i++) {
            if (ADE7943_CAL_PHCAL == i) {
                ade7953->calib_data[channel][i] = (ADE7953_SHELLY_EM == model) ? ADE7953_PHCAL_DEFAULT_CT : ADE7953_PHCAL_DEFAULT;
            } else if (ADE7953_CAL_VGAIN == i) {
                ade7953->calib_data[channel][i] = (int32_t)((float)ADE7953_GAIN_DEFAULT * 1.0);
            } else if (ADE7953_CAL_IGAIN == i) {
                ade7953->calib_data[channel][i] = (int32_t)((float)ADE7953_GAIN_DEFAULT * 1.0);
            } else if (ADE7953_CAL_WGAIN == i || ADE7953_CAL_VAGAIN == i || ADE7953_CAL_VARGAIN == i) {
                ade7953->calib_data[channel][i] = (int32_t)((float)ADE7953_GAIN_DEFAULT * 1.0);
            } else {
                ade7953->calib_data[channel][i] = ADE7953_GAIN_DEFAULT;
            }
        }
    }
}

void ade7953_retrieveCalibrationFromNvs(ade7953_t* ade7953) {
    nvs_handle_t nvs_handle;
    esp_err_t    err = nvs_open("nvs", NVS_READWRITE, &nvs_handle);

    if (err == ESP_OK) {
        size_t required_size = 0;

        char nvs_key[32] = {0};
        sprintf(nvs_key, "ade7953_cal_%c", ade7953->chip_num);

        err = nvs_get_blob(nvs_handle, nvs_key, NULL, &required_size);
        if ((err == ESP_OK) && (required_size == ADE7953_MAX_CHANNEL * ADE7953_CALIBREGS)) {
            int32_t calib_data[ADE7953_MAX_CHANNEL][ADE7953_CALIBREGS];
            nvs_get_blob(nvs_handle, nvs_key, &calib_data, &required_size);
            memcpy(&ade7953->calib_data, &calib_data, required_size);
        }
    }
}

void ade7953_saveCalibrationFromNvs(ade7953_t* ade7953) {
    nvs_handle_t nvs_handle;
    esp_err_t    err = nvs_open("nvs", NVS_READWRITE, &nvs_handle);

    if (err == ESP_OK) {
        char nvs_key[32] = {0};
        sprintf(nvs_key, "ade7953_cal_%c", ade7953->chip_num);
        nvs_set_blob(nvs_handle, nvs_key, &ade7953->calib_data, ADE7953_MAX_CHANNEL * ADE7953_CALIBREGS);
    }
}

void ade7953_init(ade7953_t* ade7953, ShellyModels_t model) {
    if (ade7953->enabled != true) {
        ade7953->enabled = true;
    }

    ade7953_writeReg(ade7953, ADE7953_CONFIG, 0x0004, pdMS_TO_TICKS(100));                        // Locking the communication interface
                                                                                                  // (Clear bit COMM_LOCK), Enable HPF
    ade7953_writeReg(ade7953, 0x0FE, 0x00AD, pdMS_TO_TICKS(100));                                 // Unlock register 0x120
    ade7953_writeReg(ade7953, ADE7953_RESERVED_0X120, 0x0030, pdMS_TO_TICKS(100));                // Configure optimum setting
    ade7953_writeReg(ade7953, ADE7953_DISNOLOAD, 0x07, pdMS_TO_TICKS(100));                       // Disable no load detection, required before setting thresholds
    ade7953_writeReg(ade7953, ADE7953_AP_NOLOAD, ADE7953_NO_LOAD_THRESHOLD, pdMS_TO_TICKS(100));  // Set no load treshold for active power
    ade7953_writeReg(ade7953, ADE7953_VAR_NOLOAD, ADE7953_NO_LOAD_THRESHOLD, pdMS_TO_TICKS(100)); // Set no load treshold for reactive power
    ade7953_writeReg(ade7953, ADE7953_DISNOLOAD, 0x00, pdMS_TO_TICKS(100));                       // Enable no load detection

    // ade7953_retrieveCalibrationFromNvs(ade7953);
    ade7953_setCalibrationForShellyDevice(ade7953, model);

    char taskName[32];
    sprintf(taskName, "ade7953_task%c", ade7953->chip_num);
    xTaskCreate(ade7953_task, taskName, 2048, ade7953, configMAX_PRIORITIES - 3, NULL);
}

void ade7953_setCalibrationForShellyDevice(ade7953_t* ade7953, ShellyModels_t model) {
    ade7953_setCalibration(ade7953, 0, 0); // First ADE7953 A registers set with calibration set 0
    switch (model) {
    case ADE7953_SHELLY_25:
    case ADE7953_SHELLY_EM:
    case ADE7953_SHELLY_PLUS_2PM:
    // case ADE7953_SHELLY_PRO_1PM:          // Uses defaults for B registers
    case ADE7953_SHELLY_PRO_4PM:
        ade7953_setCalibration(ade7953, 1, 1); // First ADE7953 B registers set with calibration set 1
    default:
        break;
    }

    int32_t regs[ADE7953_CALIBREGS];
    for (uint32_t channel = 0; channel < 2; channel++) {
        for (uint32_t i = 0; i < ADE7953_CALIBREGS; i++) {
            regs[i] = ade7953_readReg(ade7953, Ade7953CalibRegs[channel][i], pdMS_TO_TICKS(100));
            if (ADE7943_CAL_PHCAL == i) {
                if (regs[i] >= 0x0200) {
                    regs[i] &= 0x01FF; // Clear sign magnitude
                    regs[i] *= -1;     // Make negative
                }
            }
        }
        printf("ADE: CalibRegs%c V %d, I %d, W %d, VA %d, VAr %d, Ph %d\n", 'A' + channel, regs[0], regs[1], regs[2], regs[3], regs[4], regs[5]);
    }
}

static void ade7953_task(void* arg) {
    ade7953_t* ade7953 = (ade7953_t*)arg;

    nvs_handle_t nvs_handle;
    esp_err_t    err = nvs_open("nvs", NVS_READWRITE, &nvs_handle);

    if (ade7953->saveEnergiesToNvs) {
        if (err == ESP_OK) {

            char nvs_key[32] = {0};
            sprintf(nvs_key, "ade7953_ena_%c", ade7953->chip_num);

            int64_t value = 0;
            if (nvs_get_i64(nvs_handle, nvs_key, &value) == ESP_OK) {
                ade7953->data[0].activeEnergy = (double)value / 1000.0;
            }
            value = 0;
            sprintf(nvs_key, "ade7953_enb_%c", ade7953->chip_num);
            if (nvs_get_i64(nvs_handle, nvs_key, &value) == ESP_OK) {
                ade7953->data[1].activeEnergy = (double)value / 1000.0;
            }
        }
    }

    int64_t  lastRetrievalTime = 0;
    int64_t  currentTime       = 0;
    uint32_t iRmsA, iRmsB, vRms;
    int32_t  accEnergyA, accEnergyB;

    int64_t time = 0;

    TickType_t lastEnergySavedTime = 0;
    while (1) {
        vRms  = ade7953_readReg(ade7953, ADE7953_VRMS, pdMS_TO_TICKS(100));
        iRmsA = ade7953_readReg(ade7953, ADE7953_IRMSA, pdMS_TO_TICKS(100));
        iRmsB = ade7953_readReg(ade7953, ADE7953_IRMSB, pdMS_TO_TICKS(100));

        currentTime = esp_timer_get_time();
        accEnergyA  = ade7953_readReg(ade7953, ADE7953_AENERGYA, pdMS_TO_TICKS(100));
        accEnergyB  = ade7953_readReg(ade7953, ADE7953_AENERGYB, pdMS_TO_TICKS(100));

        time              = currentTime - lastRetrievalTime;
        lastRetrievalTime = currentTime;

        ade7953->data[0].voltage     = (float)vRms / 10000.0;
        ade7953->data[0].current     = (float)iRmsA / 100000.0;
        ade7953->data[0].activePower = (float)abs(accEnergyA) / (float)ADE7953_LSB_PER_WATTSECOND / ((float)time / 1000.0f / 1000.0f);
        ade7953->data[0].activeEnergy += (float)abs(accEnergyA) / (float)ADE7953_LSB_PER_WATTSECOND / 3600.0f / 1000.0f;

        ade7953->data[1].voltage     = (float)vRms / 10000.0;
        ade7953->data[1].current     = (float)iRmsB / 100000.0;
        ade7953->data[1].activePower = (float)abs(accEnergyB) / (float)ADE7953_LSB_PER_WATTSECOND / 1 / ((float)time / 1000.0f / 1000.0f);
        ade7953->data[1].activeEnergy += (float)abs(accEnergyB) / (float)ADE7953_LSB_PER_WATTSECOND / 3600.0f / 1000.0f;

        if (ade7953->saveEnergiesToNvs) {
            if (pdTICKS_TO_MS(xTaskGetTickCount() - lastEnergySavedTime) > 60 * 1000) {
                nvs_handle_t nvs_handle;
                esp_err_t    err = nvs_open("nvs", NVS_READWRITE, &nvs_handle);

                if (err == ESP_OK) {
                    char nvs_key[32] = {0};
                    sprintf(nvs_key, "ade7953_ena_%c", ade7953->chip_num);
                    nvs_set_i64(nvs_handle, nvs_key, (int64_t)(ade7953->data[0].activeEnergy * 1000));
                    sprintf(nvs_key, "ade7953_enb_%c", ade7953->chip_num);
                    nvs_set_i64(nvs_handle, nvs_key, (int64_t)(ade7953->data[1].activeEnergy * 1000));
                    printf("saved energies to nvs!\n");
                }

                lastEnergySavedTime = xTaskGetTickCount();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}