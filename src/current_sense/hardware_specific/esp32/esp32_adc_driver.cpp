
#include "esp32_mcu.h"
#include "esp32_adc_driver.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)
#define SIMPLEFOC_ADC_ATTEN ADC_ATTEN_DB_12 // ADC_11db
#define SIMPLEFOC_ADC_RES   ADC_BITWIDTH_12 // 12

#if 1
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_log.h"

static const char *TAG = "ADC_DRIVER";

typedef struct {
    gpio_num_t gpio;
    adc_unit_t unit;
    adc_channel_t channel;
} adc_gpio_map_t;

static const adc_gpio_map_t adc_gpio_map[] = {
    // ADC1通道
    { GPIO_NUM_1,  ADC_UNIT_1, ADC_CHANNEL_0 },
    { GPIO_NUM_2,  ADC_UNIT_1, ADC_CHANNEL_1 },
    { GPIO_NUM_3,  ADC_UNIT_1, ADC_CHANNEL_2 },
    { GPIO_NUM_4,  ADC_UNIT_1, ADC_CHANNEL_3 },
    { GPIO_NUM_5,  ADC_UNIT_1, ADC_CHANNEL_4 },
    { GPIO_NUM_6,  ADC_UNIT_1, ADC_CHANNEL_5 },
    { GPIO_NUM_7,  ADC_UNIT_1, ADC_CHANNEL_6 },
    { GPIO_NUM_8,  ADC_UNIT_1, ADC_CHANNEL_7 },
    { GPIO_NUM_9,  ADC_UNIT_1, ADC_CHANNEL_8 },
    { GPIO_NUM_10, ADC_UNIT_1, ADC_CHANNEL_9 },
    // ADC2通道
    { GPIO_NUM_11, ADC_UNIT_2, ADC_CHANNEL_0 },
    { GPIO_NUM_12, ADC_UNIT_2, ADC_CHANNEL_1 },
    { GPIO_NUM_13, ADC_UNIT_2, ADC_CHANNEL_2 },
    { GPIO_NUM_14, ADC_UNIT_2, ADC_CHANNEL_3 },
    { GPIO_NUM_15, ADC_UNIT_2, ADC_CHANNEL_4 },
    { GPIO_NUM_16, ADC_UNIT_2, ADC_CHANNEL_5 },
    { GPIO_NUM_17, ADC_UNIT_2, ADC_CHANNEL_6 },
    { GPIO_NUM_18, ADC_UNIT_2, ADC_CHANNEL_7 },
    { GPIO_NUM_19, ADC_UNIT_2, ADC_CHANNEL_8 },
    { GPIO_NUM_20, ADC_UNIT_2, ADC_CHANNEL_9 }
};

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_oneshot_unit_handle_t adc2_handle = NULL;

static bool get_adc_unit_channel_by_gpio(gpio_num_t gpio, adc_unit_t *unit, adc_channel_t *channel) {
    for (int i = 0; i < sizeof(adc_gpio_map)/sizeof(adc_gpio_map[0]); ++i) {
        if (adc_gpio_map[i].gpio == gpio) {
            *unit = adc_gpio_map[i].unit;
            *channel = adc_gpio_map[i].channel;
            return true;
        }
    }
    return false;
}
#endif

#if CONFIG_IDF_TARGET_ESP32 // if esp32 variant

#include "soc/sens_reg.h"

// configure the ADCs in RTC mode
// saves about 3us per call 
// going from 12us to 9us
void __configFastADCs(){
    
    // configure both ADCs in RTC mode
    SET_PERI_REG_MASK(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DATA_INV);
    SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DATA_INV);

    SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_FORCE_M); //SAR ADC1 controller (in RTC) is started by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD_FORCE_M); //SAR ADC1 pad enable bitmap is controlled by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_FORCE_M); //SAR ADC2 controller (in RTC) is started by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD_FORCE_M); //SAR ADC2 pad enable bitmap is controlled by SW

    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_M); //force XPD_SAR=0, use XPD_FSM
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S); //force XPD_AMP=0

    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_CTRL_REG, 0xfff << SENS_AMP_RST_FB_FSM_S);  //clear FSM
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
    while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_MEAS_STATUS_S) != 0); //wait det_fsm==
}


uint16_t IRAM_ATTR adcRead(uint8_t pin)
{
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Not ADC pin: "+String(pin));
        return false;//not adc pin
    }

    // start teh ADC conversion
    if(channel > 9){
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD, (1 << (channel - 10)), SENS_SAR2_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
    } else {
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
    }

    uint16_t value = 0;

    if(channel > 7){
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0); 
        // read the value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    } else {
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0); 
        // read the value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    // return value
    return value;
}

#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 // if esp32 s2 or s3 variants

#include "soc/sens_reg.h"


// configure the ADCs in RTC mode 
// no real gain - see if we do something with it later
// void __configFastADCs(){
    
//     SET_PERI_REG_MASK(SENS_SAR_READER1_CTRL_REG, SENS_SAR1_DATA_INV);
//     SET_PERI_REG_MASK(SENS_SAR_READER2_CTRL_REG, SENS_SAR2_DATA_INV);

//     SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_FORCE_M); //SAR ADC1 controller (in RTC) is started by SW
//     SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_SAR1_EN_PAD_FORCE_M); //SAR ADC1 pad enable bitmap is controlled by SW
//     SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_FORCE_M); //SAR ADC2 controller (in RTC) is started by SW
//     SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_SAR2_EN_PAD_FORCE_M); //SAR ADC2 pad enable bitmap is controlled by SW

//     CLEAR_PERI_REG_MASK(SENS_SAR_POWER_XPD_SAR_REG, SENS_FORCE_XPD_SAR_M); //force XPD_SAR=0, use XPD_FSM
//     SET_PERI_REG_BITS(SENS_SAR_POWER_XPD_SAR_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S); //force XPD_AMP=0

//     CLEAR_PERI_REG_MASK(SENS_SAR_AMP_CTRL3_REG, 0xfff << SENS_AMP_RST_FB_FSM_S);  //clear FSM
//     SET_PERI_REG_BITS(SENS_SAR_AMP_CTRL1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
//     SET_PERI_REG_BITS(SENS_SAR_AMP_CTRL1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
//     SET_PERI_REG_BITS(SENS_SAR_POWER_XPD_SAR_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
//     while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_SARADC_MEAS_STATUS_S) != 0); //wait det_fsm==
// }

uint16_t IRAM_ATTR adcRead(uint8_t pin)
{
#if 1
    adc_unit_t unit;
    adc_channel_t channel;
    if (!get_adc_unit_channel_by_gpio((gpio_num_t)pin, &unit, &channel)) {
        return 0xFFFF; // 无效通道返回异常值
    }

    int raw = 0;
    esp_err_t err;
    if (unit == ADC_UNIT_1) {
        if (!adc1_handle) return 0xFFFF;
        err = adc_oneshot_read(adc1_handle, channel, &raw);
    } else {
        if (!adc2_handle) return 0xFFFF;
        err = adc_oneshot_read(adc2_handle, channel, &raw);
    }
    if (err != ESP_OK) {
        return 0xFFFF;
    }
    return (uint16_t)raw;
#else
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Not ADC pin: "+String(pin));
        return false;//not adc pin
    }

    // start the ADC conversion
    if(channel > 9){
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS2_CTRL2_REG, SENS_SAR2_EN_PAD, (1 << (channel - 10)), SENS_SAR2_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
    } else {
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS1_CTRL2_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
    }

    uint16_t value = 0;

    if(channel > 9){
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DONE_SAR) == 0); 
        // read the value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    } else {
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DONE_SAR) == 0); 
        // read teh value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    return value;
#endif
}

#else // if others just use analogRead

#pragma message("SimpleFOC: Using analogRead for ADC reading, no fast ADC configuration available!")

uint16_t IRAM_ATTR adcRead(uint8_t pin){
    return analogRead(pin);
}

#endif

// configure the ADC for the pin
bool IRAM_ATTR adcInit(uint8_t pin){
#if 1
    // IDF 5.0+ ADC driver
    adc_unit_t unit;
    adc_channel_t channel;
    esp_err_t err;
    if (!get_adc_unit_channel_by_gpio((gpio_num_t)pin, &unit, &channel)) {
        ESP_LOGE(TAG, "GPIO %d is not ADC capable", pin);
        return false;
    }
    // init adc module.
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = unit,
    };
    if (unit == ADC_UNIT_1) {
        if (adc1_handle == NULL) {
            err = adc_oneshot_new_unit(&init_cfg, &adc1_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "ADC1 init failed: %s", esp_err_to_name(err));
                return false;
            }
        }
    } else { // ADC_UNIT_2
        if (adc2_handle == NULL) {
            err = adc_oneshot_new_unit(&init_cfg, &adc2_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "ADC2 init failed: %s", esp_err_to_name(err));
                return false;
            }
        }
    }

    // init adc channel
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = SIMPLEFOC_ADC_ATTEN,
        .bitwidth = SIMPLEFOC_ADC_RES,
    };
    if (unit == ADC_UNIT_1) {
        err = adc_oneshot_config_channel(adc1_handle, channel, &chan_cfg);
    } else {
        err = adc_oneshot_config_channel(adc2_handle, channel, &chan_cfg);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Config channel failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
#else
    static bool initialized = false;
    
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        SIMPLEFOC_ESP32_CS_DEBUG("ERROR: Not ADC pin: "+String(pin));
        return false;//not adc pin
    }

    if(! initialized){
        analogSetAttenuation(SIMPLEFOC_ADC_ATTEN);
        analogReadResolution(SIMPLEFOC_ADC_RES);
    }   
    pinMode(pin, ANALOG);
    analogSetPinAttenuation(pin, SIMPLEFOC_ADC_ATTEN);
    analogRead(pin);

#if CONFIG_IDF_TARGET_ESP32 // if esp32 variant
    __configFastADCs();
#endif

    initialized = true;
    return true;
#endif
}

#endif
