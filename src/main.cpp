#include <cstdint>
#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef RFID_CS_PIN
#define RFID_CS_PIN 4
#endif
#ifndef RFID_MOSI_PIN
#define RFID_MOSI_PIN 5
#endif
#ifndef RFID_MISO_PIN
#define RFID_MISO_PIN 6
#endif
#ifndef RFID_SCK_PIN
#define RFID_SCK_PIN 7
#endif
#ifndef RFID_RST_PIN
#define RFID_RST_PIN 3
#endif
#ifndef RFID_IRQ_PIN
#define RFID_IRQ_PIN 8
#endif
#ifndef BUZZER_PIN
#define BUZZER_PIN 2
#endif

static const char *TAG = "rfid-buzzer";
static spi_device_handle_t rfid_spi = nullptr;

static constexpr uint8_t CommandReg = 0x01;
static constexpr uint8_t ComIEnReg = 0x02;
static constexpr uint8_t ComIrqReg = 0x04;
static constexpr uint8_t ErrorReg = 0x06;
static constexpr uint8_t Status2Reg = 0x08;
static constexpr uint8_t FIFODataReg = 0x09;
static constexpr uint8_t FIFOLevelReg = 0x0A;
static constexpr uint8_t ControlReg = 0x0C;
static constexpr uint8_t BitFramingReg = 0x0D;
static constexpr uint8_t ModeReg = 0x11;
static constexpr uint8_t TxModeReg = 0x12;
static constexpr uint8_t RxModeReg = 0x13;
static constexpr uint8_t TxControlReg = 0x14;
static constexpr uint8_t TxASKReg = 0x15;
static constexpr uint8_t TModeReg = 0x2A;
static constexpr uint8_t TPrescalerReg = 0x2B;
static constexpr uint8_t TReloadRegH = 0x2C;
static constexpr uint8_t TReloadRegL = 0x2D;
static constexpr uint8_t VersionReg = 0x37;

static constexpr uint8_t PCD_Idle = 0x00;
static constexpr uint8_t PCD_Transceive = 0x0C;
static constexpr uint8_t PCD_SoftReset = 0x0F;

static constexpr uint8_t PICC_CMD_REQA = 0x26;
static constexpr uint8_t PICC_CMD_SEL_CL1 = 0x93;

static esp_err_t rfid_transfer(uint8_t *tx_buffer, uint8_t *rx_buffer, size_t length) {
    spi_transaction_t transaction = {};
    transaction.length = length * 8;
    transaction.tx_buffer = tx_buffer;
    transaction.rx_buffer = rx_buffer;
    return spi_device_transmit(rfid_spi, &transaction);
}

static esp_err_t write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {static_cast<uint8_t>((reg << 1U) & 0x7EU), value};
    uint8_t rx[2] = {};
    return rfid_transfer(tx, rx, sizeof(tx));
}

static esp_err_t read_reg(uint8_t reg, uint8_t &value) {
    uint8_t tx[2] = {static_cast<uint8_t>(((reg << 1U) & 0x7EU) | 0x80U), 0x00};
    uint8_t rx[2] = {};
    ESP_RETURN_ON_ERROR(rfid_transfer(tx, rx, sizeof(tx)), TAG, "spi read failed");
    value = rx[1];
    return ESP_OK;
}

static esp_err_t set_bit_mask(uint8_t reg, uint8_t mask) {
    uint8_t value = 0;
    ESP_RETURN_ON_ERROR(read_reg(reg, value), TAG, "read reg failed");
    return write_reg(reg, static_cast<uint8_t>(value | mask));
}

static esp_err_t clear_bit_mask(uint8_t reg, uint8_t mask) {
    uint8_t value = 0;
    ESP_RETURN_ON_ERROR(read_reg(reg, value), TAG, "read reg failed");
    return write_reg(reg, static_cast<uint8_t>(value & ~mask));
}

static void buzzer_chirp(TickType_t duration_ticks) {
    gpio_set_level(static_cast<gpio_num_t>(BUZZER_PIN), 1);
    vTaskDelay(duration_ticks);
    gpio_set_level(static_cast<gpio_num_t>(BUZZER_PIN), 0);
}

static esp_err_t antenna_on() {
    uint8_t value = 0;
    ESP_RETURN_ON_ERROR(read_reg(TxControlReg, value), TAG, "read tx control failed");
    if ((value & 0x03U) != 0x03U) {
        ESP_RETURN_ON_ERROR(set_bit_mask(TxControlReg, 0x03U), TAG, "enable antenna failed");
    }
    return ESP_OK;
}

static esp_err_t rfid_reset() {
    gpio_set_level(static_cast<gpio_num_t>(RFID_RST_PIN), 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(static_cast<gpio_num_t>(RFID_RST_PIN), 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(write_reg(CommandReg, PCD_SoftReset), TAG, "soft reset failed");
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t rfid_init() {
    ESP_RETURN_ON_ERROR(rfid_reset(), TAG, "reset failed");
    ESP_RETURN_ON_ERROR(write_reg(TModeReg, 0x8D), TAG, "write tmode failed");
    ESP_RETURN_ON_ERROR(write_reg(TPrescalerReg, 0x3E), TAG, "write prescaler failed");
    ESP_RETURN_ON_ERROR(write_reg(TReloadRegL, 30), TAG, "write treload l failed");
    ESP_RETURN_ON_ERROR(write_reg(TReloadRegH, 0), TAG, "write treload h failed");
    ESP_RETURN_ON_ERROR(write_reg(TxASKReg, 0x40), TAG, "write txask failed");
    ESP_RETURN_ON_ERROR(write_reg(ModeReg, 0x3D), TAG, "write mode failed");
    ESP_RETURN_ON_ERROR(write_reg(TxModeReg, 0x00), TAG, "write tx mode failed");
    ESP_RETURN_ON_ERROR(write_reg(RxModeReg, 0x00), TAG, "write rx mode failed");
    return antenna_on();
}

static esp_err_t transceive(const uint8_t *send_data,
                            uint8_t send_len,
                            uint8_t *back_data,
                            uint8_t *back_len,
                            uint8_t *valid_bits = nullptr) {
    ESP_RETURN_ON_ERROR(write_reg(CommandReg, PCD_Idle), TAG, "idle failed");
    ESP_RETURN_ON_ERROR(write_reg(ComIEnReg, 0xF7), TAG, "irq enable failed");
    ESP_RETURN_ON_ERROR(write_reg(ComIrqReg, 0x7F), TAG, "clear irq failed");
    ESP_RETURN_ON_ERROR(write_reg(FIFOLevelReg, 0x80), TAG, "flush fifo failed");

    for (uint8_t i = 0; i < send_len; ++i) {
        ESP_RETURN_ON_ERROR(write_reg(FIFODataReg, send_data[i]), TAG, "fifo write failed");
    }

    uint8_t framing = valid_bits == nullptr ? 0x00 : *valid_bits;
    ESP_RETURN_ON_ERROR(write_reg(BitFramingReg, framing), TAG, "write framing failed");
    ESP_RETURN_ON_ERROR(write_reg(CommandReg, PCD_Transceive), TAG, "transceive failed");
    ESP_RETURN_ON_ERROR(set_bit_mask(BitFramingReg, 0x80), TAG, "start send failed");

    bool done = false;
    for (int attempt = 0; attempt < 25; ++attempt) {
        uint8_t irq_value = 0;
        ESP_RETURN_ON_ERROR(read_reg(ComIrqReg, irq_value), TAG, "read irq failed");
        if ((irq_value & 0x30U) != 0 || (irq_value & 0x01U) != 0) {
            done = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_RETURN_ON_ERROR(clear_bit_mask(BitFramingReg, 0x80), TAG, "clear start send failed");

    if (!done) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t error = 0;
    ESP_RETURN_ON_ERROR(read_reg(ErrorReg, error), TAG, "read error reg failed");
    if ((error & 0x13U) != 0) {
        return ESP_FAIL;
    }

    if (back_data != nullptr && back_len != nullptr) {
        uint8_t fifo_level = 0;
        uint8_t control = 0;
        ESP_RETURN_ON_ERROR(read_reg(FIFOLevelReg, fifo_level), TAG, "read fifo level failed");
        ESP_RETURN_ON_ERROR(read_reg(ControlReg, control), TAG, "read control failed");
        uint8_t last_bits = control & 0x07U;
        if (valid_bits != nullptr) {
            *valid_bits = last_bits;
        }

        if (fifo_level > *back_len) {
            return ESP_ERR_INVALID_SIZE;
        }

        for (uint8_t i = 0; i < fifo_level; ++i) {
            ESP_RETURN_ON_ERROR(read_reg(FIFODataReg, back_data[i]), TAG, "read fifo byte failed");
        }
        *back_len = fifo_level;
    }

    return ESP_OK;
}

static esp_err_t request_a() {
    uint8_t request = PICC_CMD_REQA;
    uint8_t back_data[8] = {};
    uint8_t back_len = sizeof(back_data);
    uint8_t valid_bits = 0x07;
    ESP_RETURN_ON_ERROR(write_reg(BitFramingReg, 0x07), TAG, "set reqa framing failed");
    esp_err_t result = transceive(&request, 1, back_data, &back_len, &valid_bits);
    ESP_RETURN_ON_ERROR(write_reg(BitFramingReg, 0x00), TAG, "restore framing failed");
    if (result != ESP_OK) {
        return result;
    }
    if (back_len != 2 || valid_bits != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t anticollision(uint8_t uid[4]) {
    uint8_t send_data[2] = {PICC_CMD_SEL_CL1, 0x20};
    uint8_t back_data[5] = {};
    uint8_t back_len = sizeof(back_data);
    ESP_RETURN_ON_ERROR(write_reg(BitFramingReg, 0x00), TAG, "clear framing failed");
    ESP_RETURN_ON_ERROR(transceive(send_data, sizeof(send_data), back_data, &back_len), TAG, "anticollision failed");
    if (back_len != 5) {
        return ESP_FAIL;
    }

    uint8_t checksum = 0;
    for (int i = 0; i < 4; ++i) {
        uid[i] = back_data[i];
        checksum ^= back_data[i];
    }
    if (checksum != back_data[4]) {
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting RC522 RFID reader on CS/SDA GPIO%d, buzzer GPIO%d", RFID_CS_PIN, BUZZER_PIN);

    gpio_config_t output_config = {};
    output_config.pin_bit_mask = (1ULL << BUZZER_PIN) | (1ULL << RFID_RST_PIN);
    output_config.mode = GPIO_MODE_OUTPUT;
    output_config.pull_up_en = GPIO_PULLUP_DISABLE;
    output_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    output_config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&output_config));
    gpio_set_level(static_cast<gpio_num_t>(BUZZER_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(RFID_RST_PIN), 1);

    gpio_config_t irq_config = {};
    irq_config.pin_bit_mask = 1ULL << RFID_IRQ_PIN;
    irq_config.mode = GPIO_MODE_INPUT;
    irq_config.pull_up_en = GPIO_PULLUP_ENABLE;
    irq_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    irq_config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&irq_config));

    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = RFID_MOSI_PIN;
    bus_config.miso_io_num = RFID_MISO_PIN;
    bus_config.sclk_io_num = RFID_SCK_PIN;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 32;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t device_config = {};
    device_config.clock_speed_hz = 1 * 1000 * 1000;
    device_config.mode = 0;
    device_config.spics_io_num = RFID_CS_PIN;
    device_config.queue_size = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device_config, &rfid_spi));

    ESP_ERROR_CHECK(rfid_init());

    uint8_t version = 0;
    ESP_ERROR_CHECK(read_reg(VersionReg, version));
    ESP_LOGI(TAG, "MFRC522 version register: 0x%02X", version);

    uint8_t last_uid[4] = {};
    bool have_last_uid = false;

    while (true) {
        if (request_a() == ESP_OK) {
            uint8_t uid[4] = {};
            if (anticollision(uid) == ESP_OK) {
                bool is_new_card = !have_last_uid || std::memcmp(uid, last_uid, sizeof(uid)) != 0;
                if (is_new_card) {
                    std::memcpy(last_uid, uid, sizeof(uid));
                    have_last_uid = true;
                    ESP_LOGI(TAG,
                             "Card UID: %02X:%02X:%02X:%02X",
                             uid[0],
                             uid[1],
                             uid[2],
                             uid[3]);
                    buzzer_chirp(pdMS_TO_TICKS(80));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}
