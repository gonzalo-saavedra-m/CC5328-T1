#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "sdkconfig.h"

#define CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BUF_SIZE (128)       // buffer size
#define TXD_PIN 1            // UART TX pin
#define RXD_PIN 3            // UART RX pin
#define UART_NUM UART_NUM_0  // UART port number
#define BAUD_RATE 115200     // Baud rate
#define M_PI 3.14159265358979323846

#define I2C_MASTER_SCL_IO GPIO_NUM_22  // GPIO pin
#define I2C_MASTER_SDA_IO GPIO_NUM_21  // GPIO pin
#define I2C_MASTER_FREQ_HZ 10000
#define BME_ESP_SLAVE_ADDR 0x76
#define WRITE_BIT 0x0
#define READ_BIT 0x1
#define ACK_CHECK_EN 0x0
#define EXAMPLE_I2C_ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

esp_err_t ret = ESP_OK;
esp_err_t ret2 = ESP_OK;

uint16_t val0[6];

// consts
uint8_t res_heat_0 = 0x5A;
uint8_t gas_wait_0 = 0x64;
uint8_t gas_wait_shared = 0x6E;
uint8_t ctrl_gas_1 = 0x71;
uint8_t ctrl_hum = 0x72;
uint8_t ctrl_meas = 0x74;

float task_delay_ms = 1000;

static void uart_setup() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

int serial_read(char *buffer, int size){
    int len = uart_read_bytes(UART_NUM, (uint8_t*)buffer, size, pdMS_TO_TICKS(1000));
    return len;
}

esp_err_t sensor_init(void) {
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;  // 0
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}


esp_err_t bme_i2c_read(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bme_i2c_write(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_wr, size_t size) {
    uint8_t size1 = 1;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size1, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ------------ BME 688 ------------- //
/*
 * @brief BME68X sensor settings structure which comprises of
 * over-sampling.
 */
struct bme68x_conf
{
    /*! Humidity oversampling. Refer @ref osx*/
    uint8_t os_hum;

    /*! Temperature oversampling. Refer @ref osx */
    uint8_t os_temp;

    /*! Pressure oversampling. Refer @ref osx */
    uint8_t os_pres;
};

/*
 * @brief BME68X gas heater configuration PARALLEL
 */
struct bme68x_heatr_conf
{
    /*! Store the heater temperature profile in degree Celsius */
    uint16_t *heatr_temp_prof;

    /*! Store the heating duration profile in milliseconds */
    uint16_t *heatr_dur_prof;

    /*! Variable to store the length of the heating profile */
    uint8_t profile_len;

    /*!
     * Variable to store heating duration for parallel mode
     * in milliseconds
     */
    uint16_t shared_heatr_dur;
};

/* Heater temperature in degree Celsius */
uint16_t temp_prof[3] = { 320, 100, 100 };

/* Multiplier to the shared heater duration */
uint16_t mul_prof[3] = { 5, 2, 10 };

uint8_t calc_gas_wait(uint16_t dur) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1176
    uint8_t factor = 0;
    uint8_t durval;

    if (dur >= 0xfc0) {
        durval = 0xff; /* Max duration*/
    } else {
        while (dur > 0x3F) {
            dur = dur >> 2;
            factor += 1;
        }

        durval = (uint8_t)(dur + (factor * 64));
    }

    return durval;
}

uint8_t calc_res_heat(uint16_t temp) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1145
    uint8_t heatr_res;
    uint8_t amb_temp = 25;

    uint8_t reg_par_g1 = 0xED;
    uint8_t par_g1;
    bme_i2c_read(I2C_NUM_0, &reg_par_g1, &par_g1, 1);

    uint8_t reg_par_g2_lsb = 0xEB;
    uint8_t par_g2_lsb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_lsb, &par_g2_lsb, 1);
    uint8_t reg_par_g2_msb = 0xEC;
    uint8_t par_g2_msb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_msb, &par_g2_msb, 1);
    uint16_t par_g2 = (int16_t)(CONCAT_BYTES(par_g2_msb, par_g2_lsb));

    uint8_t reg_par_g3 = 0xEE;
    uint8_t par_g3;
    bme_i2c_read(I2C_NUM_0, &reg_par_g3, &par_g3, 1);

    uint8_t reg_res_heat_range = 0x02;
    uint8_t res_heat_range;
    uint8_t mask_res_heat_range = (0x3 << 4);
    uint8_t tmp_res_heat_range;

    uint8_t reg_res_heat_val = 0x00;
    uint8_t res_heat_val;

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;

    if (temp > 400) {
        temp = 400;
    }

    bme_i2c_read(I2C_NUM_0, &reg_res_heat_range, &tmp_res_heat_range, 1);
    bme_i2c_read(I2C_NUM_0, &reg_res_heat_val, &res_heat_val, 1);
    res_heat_range = (mask_res_heat_range & tmp_res_heat_range) >> 4;

    var1 = (((int32_t)amb_temp * par_g3) / 1000) * 256;
    var2 = (par_g1 + 784) * (((((par_g2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (res_heat_range + 4));
    var5 = (131 * res_heat_val) + 65536;
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);

    return heatr_res;
}

int bme_get_chipid(void) {
    uint8_t reg_id = 0xd0;
    uint8_t tmp;

    bme_i2c_read(I2C_NUM_0, &reg_id, &tmp, 1);
    printf("Valor de CHIPID: %2X \n\n", tmp);

    if (tmp == 0x61) {
        printf("Chip BME688 reconocido.\n\n");
        return 0;
    } else {
        printf("Chip BME688 no reconocido. \nCHIP ID: %2x\n\n", tmp);  // %2X
    }

    return 1;
}

int bme_softreset(void) {
    uint8_t reg_softreset = 0xE0, val_softreset = 0xB6;

    ret = bme_i2c_write(I2C_NUM_0, &reg_softreset, &val_softreset, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("\nError en softreset: %s \n", esp_err_to_name(ret));
        return 1;
    } else {
        printf("\nSoftreset: OK\n\n");
    }
    return 0;
}

void bme_forced_mode(void) {
    /*
    Fuente: Datasheet[19]
    https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=19

    Para configurar el BME688 en forced mode los pasos son:

    1. Set humidity oversampling to 1x     |-| 0b001 to osrs_h<2:0>
    2. Set temperature oversampling to 2x  |-| 0b010 to osrs_t<2:0>
    3. Set pressure oversampling to 16x    |-| 0b101 to osrs_p<2:0>

    4. Set gas duration to 100 ms          |-| 0x59 to gas_wait_0
    5. Set heater step size to 0           |-| 0x00 to res_heat_0
    6. Set number of conversion to 0       |-| 0b0000 to nb_conv<3:0> and enable gas measurements
    7. Set run_gas to 1                    |-| 0b1 to run_gas<5>

    8. Set operation mode                  |-| 0b01  to mode<1:0>

    */

    // Datasheet[33]
    uint8_t mask;
    uint8_t prev;
    // Configuramos el oversampling (Datasheet[36])

    // 1. osrs_h esta en ctrl_hum (LSB) -> seteamos 001 en bits 2:0
    uint8_t osrs_h = 0b001;
    mask = 0b00000111;
    bme_i2c_read(I2C_NUM_0, &ctrl_hum, &prev, 1);
    osrs_h = (prev & ~mask) | osrs_h;

    // 2. osrs_t esta en ctrl_meas MSB -> seteamos 010 en bits 7:5
    uint8_t osrs_t = 0b01000000;
    // 3. osrs_p esta en ctrl_meas LSB -> seteamos 101 en bits 4:2 [Datasheet:37]
    uint8_t osrs_p = 0b00010100;
    uint8_t osrs_t_p = osrs_t | osrs_p;
    // Se recomienda escribir hum, temp y pres en un solo write

    // Configuramos el sensor de gas

    // 4. Seteamos gas_wait_0 a 100ms
    uint8_t gas_duration = calc_gas_wait(100);

    // 5. Seteamos res_heat_0
    uint8_t heater_step = calc_res_heat(300);

    // 6. nb_conv esta en ctrl_gas_1 -> seteamos bits 3:0
    uint8_t nb_conv = 0b00000000;
    // 7. run_gas esta en ctrl_gas_1 -> seteamos bit 5
    uint8_t run_gas = 0b00100000;
    uint8_t gas_conf = nb_conv | run_gas;

    bme_i2c_write(I2C_NUM_0, &gas_wait_0, &gas_duration, 1);
    bme_i2c_write(I2C_NUM_0, &res_heat_0, &heater_step, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_hum, &osrs_h, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_meas, &osrs_t_p, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_gas_1, &gas_conf, 1);

    // Seteamos el modo
    // 8. Seteamos el modo a 01, pasando primero por sleep
    uint8_t mode = 0b00000001;
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;

    do {
        ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);

        if (ret == ESP_OK) { //si es que está seteado algun power mode
            // Se pone en sleep
            pow_mode = (tmp_pow_mode & 0x03); //mask 0b00000011 (mantiene los dos bits menos significativos)
            if (pow_mode != 0) {
                tmp_pow_mode &= ~0x03; //setea tmp_pow_mode en 0b00000000
                ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1); //se escribe el sleep mode
            }
        }
    } while ((pow_mode != 0x0) && (ret == ESP_OK));

    tmp_pow_mode = (tmp_pow_mode & ~0x03) | mode;
    ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

int bme_check_forced_mode(void) {
    uint8_t tmp, tmp2, tmp3, tmp4, tmp5;
    ret = bme_i2c_read(I2C_NUM_0, &ctrl_hum, &tmp, 1);
    ret = bme_i2c_read(I2C_NUM_0, &gas_wait_0, &tmp2, 1);
    ret = bme_i2c_read(I2C_NUM_0, &res_heat_0, &tmp3, 1);
    ret = bme_i2c_read(I2C_NUM_0, &ctrl_gas_1, &tmp4, 1);
    ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp5, 1);
    vTaskDelay(task_delay_ms / portTICK_PERIOD_MS);
    return (tmp == 0b001 && tmp2 == 0x59 && tmp3 == 0x00 && tmp4 == 0b100000 && tmp5 == 0b01010101);
}

uint32_t bme68x_get_meas_dur(const uint8_t op_mode, struct bme68x_conf *conf /*osrs_t, osrs_p, osrs_h*/)
{
    uint32_t meas_dur = 0; /* Calculate in us */
    uint32_t meas_cycles;
    uint8_t os_to_meas_cycles[6] = { 0, 1, 2, 4, 8, 16 };

    if (conf != NULL) //Si hay valores para osrs_t, osrs_p y osrs_h -> if(osrs_t != NULL && osrs_p != NULL && osrs_h != NULL)
    {
            meas_cycles = os_to_meas_cycles[conf->os_temp]; //osrs_t
            meas_cycles += os_to_meas_cycles[conf->os_pres]; //osrs_p
            meas_cycles += os_to_meas_cycles[conf->os_hum]; //osrs_h

            /* TPH measurement duration */
            meas_dur = meas_cycles * UINT32_C(1963);
            meas_dur += UINT32_C(477 * 4); /* TPH switching duration */
            meas_dur += UINT32_C(477 * 5); /* Gas measurement duration */
    }

    return meas_dur;
}

/* This internal API is used to calculate the register value for
 * shared heater duration */
static uint8_t calc_heatr_dur_shared(uint16_t dur)
{
    uint8_t factor = 0;
    uint8_t heatdurval;

    if (dur >= 0x783)
    {
        heatdurval = 0xff; /* Max duration */
    }
    else
    {
        /* Step size of 0.477ms */
        dur = (uint16_t)(((uint32_t)dur * 1000) / 477);
        while (dur > 0x3F)
        {
            dur = dur >> 2;
            factor += 1;
        }

        heatdurval = (uint8_t)(dur + (factor * 64));
    }

    return heatdurval;
}

/* This internal API is used to set heater configurations */
void set_conf(const struct bme68x_heatr_conf *conf, uint8_t op_mode)
{
    uint8_t i;
    uint8_t shared_dur;
    uint8_t gass_wait_shared = 0x6E;

    //deberian ser 3 elementos para cada lista ya que profile_len = 3
    uint8_t res_heat[3] = { 0, 0, 0 }; //res heat <9:0>
    uint8_t rh_reg_data[3] = { 0, 0, 0 };
    uint8_t gas_wait[3] = { 0, 0, 0 }; //gas wait <9:0>
    uint8_t gw_reg_data[3] = { 0, 0, 0 };

    for (i = 0; i < conf->profile_len; i++) //profile_len = 3
    {//de los listados se usaran solo los primeros 3, al igual que las direcciones
        res_heat[i] = 0x5A + i; // desde 0x5A hasta 0x63, 0x5A + i. 0x5A +1 = 0x5B; 0x5b + 1 = 0x5C; 0x5C + 1 = 0x5D
        rh_reg_data[i] = calc_res_heat(conf->heatr_temp_prof[i] /*listado de 10 (3) temperaturas*/); //funcion definida en la plantilla
        gas_wait[i] = 0x64 + i; // desde 0x64 hasta 0x6D, 0x64 + i
        gw_reg_data[i] = (uint8_t) conf->heatr_dur_prof[i]; //listado de 10 (3) multiplicadores
    }

    shared_dur = calc_heatr_dur_shared(conf->shared_heatr_dur); //SE DEFINE MAS ABAJO
    // gas wait shared
    bme_i2c_write(I2C_NUM_0, &gas_wait_shared, &shared_dur, 1);

    // res heat
    bme_i2c_write(I2C_NUM_0, &res_heat[0], &rh_reg_data[0], 1);
    bme_i2c_write(I2C_NUM_0, &res_heat[1], &rh_reg_data[1], 1);
    bme_i2c_write(I2C_NUM_0, &res_heat[2], &rh_reg_data[2], 1);

    // gas wait
    bme_i2c_write(I2C_NUM_0, &gas_wait[0], &gw_reg_data[0], 1);
    bme_i2c_write(I2C_NUM_0, &gas_wait[1], &gw_reg_data[1], 1);
    bme_i2c_write(I2C_NUM_0, &gas_wait[2], &gw_reg_data[2], 1);
}

/*
 * @brief This API is used to set the gas configuration of the sensor.
 */
void bme68x_set_heatr_conf(
    uint8_t op_mode,
    const struct bme68x_heatr_conf *conf) {
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;
    if (conf != NULL) /*heatr_temp_prof, heatr_dur_prof, profile_len != NULL*/
    {
        //se pone en modo sleep y luego configura el modo paralelo
        do {
            ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);

            if (ret == ESP_OK) { //si es que está seteado algun power mode
                // Se pone en sleep
                pow_mode = (tmp_pow_mode & 0x03); //mask 0b00000011 (mantiene los dos bits menos significativos)
                if (pow_mode != 0) {
                    tmp_pow_mode &= ~0x03; //setea tmp_pow_mode en 0b00000000
                    ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1); //se escribe el sleep mode
                }
            }
        } while ((pow_mode != 0x0) && (ret == ESP_OK));

        //se configuran los valores de res_heat, gas_wait y gas_wait_shared
        set_conf(conf, op_mode); // LA DEFINIMOS MAS ABAJO
    }
}

void bme_sleep(void) {
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;
    do {
        ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);

        if (ret == ESP_OK) { //si es que está seteado algun power mode
            // Se pone en sleep
            pow_mode = (tmp_pow_mode & 0x03); //mask 0b00000011 (mantiene los dos bits menos significativos)
            if (pow_mode != 0) {
                tmp_pow_mode &= ~0x03; //setea tmp_pow_mode en 0b00000000
                ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1); //se escribe el sleep mode
            }
        }
    } while ((pow_mode != 0x0) && (ret == ESP_OK));
}

void bme_parallel_mode(void){
     // Datasheet[33]
    uint8_t gas_wait_shared = 0x6E;

    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_conf conf;
    uint8_t mask;
    uint8_t prev;
    // Configuramos el oversampling (Datasheet[36])

    // 1. osrs_h esta en ctrl_hum (LSB) -> seteamos 001 en bits 2:0
    uint8_t osrs_h = 0b001;
    mask = 0b00000111;
    bme_i2c_read(I2C_NUM_0, &ctrl_hum, &prev, 1);
    osrs_h = (prev & ~mask) | osrs_h;

    // 2. osrs_t esta en ctrl_meas MSB -> seteamos 010 en bits 7:5
    uint8_t osrs_t = 0b01000000;
    // 3. osrs_p esta en ctrl_meas LSB -> seteamos 101 en bits 4:2 [Datasheet:37]
    uint8_t osrs_p = 0b00010100;
    uint8_t osrs_t_p = osrs_t | osrs_p;
    // Se recomienda escribir hum, temp y pres en un solo write

    // 6. nb_conv esta en ctrl_gas_1 -> seteamos bits 3:0
    uint8_t nb_conv = 0b00000011;   //step 0-2 (3)
    // 7. run_gas esta en ctrl_gas_1 -> seteamos bit 5
    uint8_t run_gas = 0b00100000;
    uint8_t gas_conf = nb_conv | run_gas;

    // se definen los oversampling en la estructura conf
    conf.os_hum = osrs_h;
    conf.os_temp = osrs_t;
    conf.os_pres = osrs_p;

    bme_i2c_write(I2C_NUM_0, &ctrl_hum, &osrs_h, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_meas, &osrs_t_p, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_gas_1, &gas_conf, 1);

    // Seteamos el modo
    // 8. Seteamos el modo a 10, pasando primero por sleep
    uint8_t mode = 0b00000010; //parallel mode
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;

    //se pone en sleep
    do {
        ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);

        if (ret == ESP_OK) { //si es que está seteado algun power mode
            // Se pone en sleep
            pow_mode = (tmp_pow_mode & 0x03); //mask 0b00000011 (mantiene los dos bits menos significativos)
            if (pow_mode != 0) {
                tmp_pow_mode &= ~0x03; //setea tmp_pow_mode en 0b00000000
                ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1); //se escribe el sleep mode
            }
        }
    } while ((pow_mode != 0x0) && (ret == ESP_OK));

    //gas_wait_shared, gas_wait_x, res_wait_x
    /* Shared heating duration in milliseconds */
    heatr_conf.shared_heatr_dur = (uint16_t)(140 - (bme68x_get_meas_dur(mode, &conf) / 1000));
    heatr_conf.profile_len = 3;
    heatr_conf.heatr_dur_prof = mul_prof;
    heatr_conf.heatr_temp_prof = temp_prof;

    //set the gas configuration to the sensor
    bme68x_set_heatr_conf(mode /*0b00000010*/, &heatr_conf /*heatr_temp_prof, heatr_dur_prof, profile_len, shared_heatr_dur*/);

    tmp_pow_mode = (tmp_pow_mode & ~0x03) | mode;
    ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

int bme_temp_celsius(uint32_t temp_adc, int *t_fine) {
    // Datasheet[23]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    // Se obtienen los parametros de calibracion
    uint8_t addr_par_t1_lsb = 0xE9, addr_par_t1_msb = 0xEA;
    uint8_t addr_par_t2_lsb = 0x8A, addr_par_t2_msb = 0x8B;
    uint8_t addr_par_t3_lsb = 0x8C;
    uint16_t par_t1;
    uint16_t par_t2;
    uint16_t par_t3;

    uint8_t par[5];
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t3_lsb, par + 4, 1);

    par_t1 = (par[1] << 8) | par[0];
    par_t2 = (par[3] << 8) | par[2];
    par_t3 = par[4];

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int calc_temp;

    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2 = (var1 * (int32_t)par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = ((var3) * ((int32_t)par_t3 << 4)) >> 14;
    *t_fine = (int32_t)(var2 + var3);
    calc_temp = (((*t_fine * 5) + 128) >> 8);
    return calc_temp;
}

int bme_pressure_pascal(uint32_t pres_adc, int *t_fine) {
    uint8_t addr_par_p1_lsb = 0x8E, addr_par_p1_msb = 0x8F;
    uint8_t addr_par_p2_lsb = 0x90, addr_par_p2_msb = 0x91;
    uint8_t addr_par_p3_lsb = 0x92;
    uint8_t addr_par_p4_lsb = 0x94, addr_par_p4_msb = 0x95;
    uint8_t addr_par_p5_lsb = 0x96, addr_par_p5_msb = 0x97;
    uint8_t addr_par_p6_lsb = 0x99;
    uint8_t addr_par_p7_lsb = 0x98;
    uint8_t addr_par_p8_lsb = 0x9C, addr_par_p8_msb = 0x9D;
    uint8_t addr_par_p9_lsb = 0x9E, addr_par_p9_msb = 0x9F;
    uint8_t addr_par_p10_lsb = 0xA0;
    uint16_t par_p1;
    uint16_t par_p2;
    uint16_t par_p3;
    uint16_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    uint16_t par_p7;
    uint16_t par_p8;
    uint16_t par_p9;
    uint16_t par_p10;

    uint8_t par[16];
    bme_i2c_read(I2C_NUM_0, &addr_par_p1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p3_lsb, par + 4, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p4_lsb, par + 5, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p4_msb, par + 6, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p5_lsb, par + 7, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p5_msb, par + 8, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p6_lsb, par + 9, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p7_lsb, par + 10, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p8_lsb, par + 11, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p8_msb, par + 12, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p9_lsb, par + 13, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p9_msb, par + 14, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p10_lsb, par + 15, 1);

    par_p1 = (par[1] << 8) | par[0];
    par_p2 = (par[3] << 8) | par[2];
    par_p3 = par[4];
    par_p4 = (par[6] << 8) | par[5];
    par_p5 = (par[8] << 8) | par[7];
    par_p6 = par[9];
    par_p7 = par[10];
    par_p8 = (par[12] << 8) | par[11];
    par_p9 = (par[14] << 8) | par[13];
    par_p10 = par[15];

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int calc_press;

    var1 = ((int32_t)t_fine >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)par_p3 << 5)) >> 3) + (((int32_t)par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)par_p1) >> 15;
    calc_press = 1048576 - pres_adc;
    calc_press = (uint32_t)((calc_press - (var2 >> 12)) * ((uint32_t)3125));
    if(calc_press >= (1 << 30)){
        calc_press = ((calc_press / (uint32_t)var1) << 1);
    } else {
        calc_press = ((calc_press << 1) / (uint32_t)var1);
    }
    var1 = ((int32_t)par_p9 * (int32_t)(((calc_press >> 3) * (calc_press >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(calc_press >> 2) * (int32_t)par_p8) >> 13;
    var3 = ((int32_t)(calc_press >> 8) * (int32_t)(calc_press >> 8) * (int32_t)(calc_press >> 8) * (int32_t)par_p10) >> 17;
    calc_press = (int32_t)(calc_press) +  ((var1 + var2 + var3 + ((int32_t)par_p7 << 7)) >> 4);

    return calc_press;
}

int bme_hum_percent(uint32_t hum_adc, int *t_fine, uint32_t temp_comp) {
    uint8_t addr_par_h1_lsb = 0xE2 /*<3:0>*/, addr_par_h1_msb = 0xE3;
    uint8_t addr_par_h2_lsb = 0xE2 /*<7:4>*/, addr_par_h2_msb = 0xE1;
    uint8_t addr_par_h3_lsb = 0xE4;
    uint8_t addr_par_h4_lsb = 0xE5;
    uint8_t addr_par_h5_lsb = 0xE6;
    uint8_t addr_par_h6_lsb = 0xE7;
    uint8_t addr_par_h7_lsb = 0xE8;
    uint16_t par_h1;
    uint16_t par_h2;
    uint16_t par_h3;
    uint16_t par_h4;
    uint16_t par_h5;
    uint16_t par_h6;
    uint16_t par_h7;

    uint8_t par[9];
    //h1 y h2 se escriben en la misma direccion, pero en bits diferentes, h1 en <3:0> y h2 en <7:4>

    bme_i2c_read(I2C_NUM_0, &addr_par_h1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h3_lsb, par + 4, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h4_lsb, par + 5, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h5_lsb, par + 6, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h6_lsb, par + 7, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h7_lsb, par + 8, 1);

    par_h1 = (par[1] << 4) | (par[0] & 0x0F); //mask 0b00001111
    par_h2 = (par[3] << 4) | (par[2] & 0xF0); //mask 0b11110000
    par_h3 = par[4];
    par_h4 = par[5];
    par_h5 = par[6];
    par_h6 = par[7];
    par_h7 = par[8];

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    int64_t var5;
    int64_t var6;
    int64_t calc_hum;

    var1 = (int32_t)hum_adc - (int32_t)((int32_t)par_h1 << 4) - (((temp_comp * (int32_t)par_h3) / ((int32_t)100)) >> 1);
    var2 = ((int32_t)par_h2 * (((temp_comp * (int32_t)par_h4) / ((int32_t)100)) + (((temp_comp * ((temp_comp * (int32_t)par_h5) / ((int32_t)100))) >> 6) / ((int32_t)100)) + ((int32_t)(1 << 14)))) >> 10;
    var3 = var1 * var2;
    var4 = (((int32_t)par_h6 << 7) + ((temp_comp * (int32_t)par_h7) / ((int32_t)100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;
    calc_hum = (var3 + var6) >> 12;
    calc_hum = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;

    return calc_hum;
}

int bme_gas_resistance(uint32_t gas_adc, uint32_t gas_range) {
    uint32_t var1 = UINT32_C(262144) >> gas_range;
    int32_t var2 = (int32_t) gas_adc - INT32_C(512);
    var2 *= INT32_C(3);
    var2 = INT32_C(4096) + var2;
    int calc_gas_res = (UINT32_C(10000) * var1) / (uint32_t)var2;
    return calc_gas_res * 100;
}

uint8_t bme_get_mode(void) {
    uint8_t reg_mode = 0x74;
    uint8_t tmp;
    ret = bme_i2c_read(I2C_NUM_0, &reg_mode, &tmp, 1);
    tmp = tmp & 0x3;
    return tmp;
}

float compare_desc(const void* a, const void* b){
    return (*(float*)b - *(float*)a);
}

void calc_top5(float* data, float* top5, int length){
    qsort(data, length, sizeof(float), compare_desc);
    for(int i = 0; i < 5; i++){
        top5[i] = data[i];
    }
}


void bme_read_data(int length) {
    // Datasheet[23:41]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    uint8_t tmp;
    //data_temp, data_pres, data_hum, data_gas
    float* top5_temp = (float*) malloc(5 * sizeof(float));
    float* top5_pres = (float*) malloc(5 * sizeof(float));
    float* top5_hum = (float*) malloc(5 * sizeof(float));
    float* top5_gas = (float*) malloc(5 * sizeof(float));

    float* data_temp = (float*) malloc(length * sizeof(float));
    float* data_pres = (float*) malloc(length * sizeof(float));
    float* data_hum = (float*) malloc(length * sizeof(float));
    float* data_gas = (float*) malloc(length * sizeof(float));

    // para parallel se usan los fields de 0 a 2
    //field 0 -> {0x22, 0x23, 0x24}
    //field 1 -> {0x33, 0x34, 0x35}
    //field 2 -> {0x44, 0x45, 0x46}
    uint8_t parallel_temp_addr[3][3] = {{0x22, 0x23, 0x24}, {0x33, 0x34, 0x35}, {0x44, 0x45, 0x46}};
    uint8_t parallel_pres_addr[3][3] = {{0x1F, 0x20, 0x21}, {0x30, 0x31, 0x32}, {0x41, 0x42, 0x43}};
    uint8_t parallel_hum_addr[3][3] = {{0x25, 0x26}, {0x36, 0x37}, {0x47, 0x48}};
    uint8_t parallel_gas_addr[3][3] = {{0x2C, 0x2D}, {0x3D, 0x3E}, {0x4E, 0x4F}};
    //gas range is in 2D, 3E, 4F but in <3:0> bits, while gas adc in msb data is in <7:6> bits
    // para forced se usan solo los field 0

    uint8_t cur_mode = bme_get_mode();
    uint32_t f_temp_adc = 0;
    uint32_t f_pres_adc = 0;
    uint32_t f_hum_adc = 0;
    uint32_t f_gas_adc = 0;
    uint32_t f_gas_range = 0;

    uint32_t p_temp_adc[3] = {0, 0, 0};
    uint32_t p_pres_adc[3] = {0, 0, 0};
    uint32_t p_hum_adc[3] = {0, 0, 0};
    uint32_t p_gas_adc[3] = {0, 0, 0};
    uint32_t p_gas_range[3] = {0, 0, 0};

    uint32_t temp_parallel[3] = {0, 0, 0};
    uint32_t pres_parallel[3] = {0, 0, 0};
    uint32_t hum_parallel[3] = {0, 0, 0};
    uint32_t gas_parallel[3] = {0, 0, 0};

    printf("AFUERA DEL IF\n");
    printf("cur_mode para el if: %d\n", cur_mode);
    //if mode is forced
    if(cur_mode == 1){ //hacer 100 reads
        printf("DENTRO DEL IF\n");
        int t_fine; //podria haber errores en cómo se llama (&, *, etc.)
        // Datasheet[41]
        // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=41
        for(int i = 0; i < length; i++){ //100 reads
            bme_forced_mode();
            f_temp_adc = 0;
            bme_i2c_read(I2C_NUM_0, &parallel_temp_addr[0][0], &tmp, 1);
            f_temp_adc = f_temp_adc | tmp << 12;
            bme_i2c_read(I2C_NUM_0, &parallel_temp_addr[0][1], &tmp, 1);
            f_temp_adc = f_temp_adc | tmp << 4;
            bme_i2c_read(I2C_NUM_0, &parallel_temp_addr[0][2], &tmp, 1);
            f_temp_adc = f_temp_adc | (tmp & 0xf0) >> 4;

            uint32_t f_temp = bme_temp_celsius(f_temp_adc, &t_fine);
            //guarda el dato en data_temp[i]
            data_temp[i] = (float)f_temp / 100;
            float num = data_temp[i];

            f_pres_adc = 0;
            bme_i2c_read(I2C_NUM_0, &parallel_pres_addr[0][0], &tmp, 1);
            f_pres_adc = f_pres_adc | tmp << 12;
            bme_i2c_read(I2C_NUM_0, &parallel_pres_addr[0][1], &tmp, 1);
            f_pres_adc = f_pres_adc | tmp << 4;
            bme_i2c_read(I2C_NUM_0, &parallel_pres_addr[0][2], &tmp, 1);
            f_pres_adc = f_pres_adc | (tmp & 0xf0) >> 4;

            uint32_t f_pres = bme_pressure_pascal(f_pres_adc, &t_fine);
            //guarda el dato en data_pres[i]
            data_pres[i] = (float)f_pres;
            f_hum_adc = 0;
            bme_i2c_read(I2C_NUM_0, &parallel_hum_addr[0][0], &tmp, 1);
            f_hum_adc = f_hum_adc | tmp << 8;
            bme_i2c_read(I2C_NUM_0, &parallel_hum_addr[0][1], &tmp, 1);
            f_hum_adc = f_hum_adc | tmp;

            uint32_t f_hum = bme_hum_percent(f_hum_adc, &t_fine, f_temp);
            //guarda el dato en data_hum[i]
            data_hum[i] = (float)f_hum;
            f_gas_adc = 0;
            bme_i2c_read(I2C_NUM_0, &parallel_gas_addr[0][0], &tmp, 1);
            f_gas_adc = f_gas_adc | tmp << 2;
            bme_i2c_read(I2C_NUM_0, &parallel_gas_addr[0][1], &tmp, 1);
            f_gas_adc = f_gas_adc | (tmp & 0xC0) >> 6;

            //gas_range
            bme_i2c_read(I2C_NUM_0, &parallel_gas_addr[0][1], &tmp, 1);
            f_gas_range = f_gas_range | (tmp & 0x0F);

            uint32_t f_gas = bme_gas_resistance(f_gas_adc, f_gas_range);
            //guarda el dato en data_gas[i]
            data_gas[i] = (float)f_gas;
        }
    }
    //if mode is parallel
    else if (cur_mode == 2)
    {
        printf("DENTRO DEL ELSE IF\n");
        bme_parallel_mode();
        int readings = 0;
        while (readings < length) {
            for(int i = 0; i < 3; i++) {
                int t_fine;
                p_temp_adc[i] = 0;
                bme_i2c_read(I2C_NUM_0, &parallel_temp_addr[i][0], &tmp, 1); //msb
                p_temp_adc[i] = p_temp_adc[i] | tmp << 12;
                bme_i2c_read(I2C_NUM_0, &parallel_temp_addr[i][1], &tmp, 1); //lsb
                p_temp_adc[i] = p_temp_adc[i] | tmp << 4;
                bme_i2c_read(I2C_NUM_0, &parallel_temp_addr[i][2], &tmp, 1); //xlsb
                p_temp_adc[i] = p_temp_adc[i] | (tmp & 0xf0) >> 4;

                temp_parallel[i] = bme_temp_celsius(p_temp_adc[i], &t_fine);
                // uart_write_bytes(UART_NUM, (float)temp_parallel[i]/100, sizeof(float));
                data_temp[readings] = (float)temp_parallel[i]/100;

                p_pres_adc[i] = 0;
                bme_i2c_read(I2C_NUM_0, &parallel_pres_addr[i][0], &tmp, 1); //msb
                p_pres_adc[i] = p_pres_adc[i] | tmp << 12;
                bme_i2c_read(I2C_NUM_0, &parallel_pres_addr[i][1], &tmp, 1); //lsb
                p_pres_adc[i] = p_pres_adc[i] | tmp << 4;
                bme_i2c_read(I2C_NUM_0, &parallel_pres_addr[i][2], &tmp, 1); //xlsb
                p_pres_adc[i] = p_pres_adc[i] | (tmp & 0xf0) >> 4;

                pres_parallel[i] = bme_pressure_pascal(p_pres_adc[i], &t_fine);
                data_pres[readings] = (float)pres_parallel[i];

                p_hum_adc[i] = 0;
                bme_i2c_read(I2C_NUM_0, &parallel_hum_addr[i][0], &tmp, 1); //msb
                p_hum_adc[i] = p_hum_adc[i] | tmp << 8;
                bme_i2c_read(I2C_NUM_0, &parallel_hum_addr[i][1], &tmp, 1); //lsb
                p_hum_adc[i] = p_hum_adc[i] | tmp;

                hum_parallel[i] = bme_hum_percent(p_hum_adc[i], &t_fine, temp_parallel[i]);
                data_hum[readings] = (float)hum_parallel[i];

                p_gas_adc[i] = 0;
                /*Contains the MSB part gas resistance [9:2] of the raw gas resistance. lsb [1:0]*/
                bme_i2c_read(I2C_NUM_0, &parallel_gas_addr[i][0], &tmp, 1); //msb
                p_gas_adc[i] = p_gas_adc[i] | tmp << 2;
                bme_i2c_read(I2C_NUM_0, &parallel_gas_addr[i][1], &tmp, 1); //lsb
                p_gas_adc[i] = p_gas_adc[i] | (tmp & 0xC0) >> 6;

                //gas_range
                bme_i2c_read(I2C_NUM_0, &parallel_gas_addr[i][1], &tmp, 1); //lsb
                p_gas_range[i] = p_gas_range[i] | (tmp & 0x0F);

                gas_parallel[i] = bme_gas_resistance(p_gas_adc[i], p_gas_range[i]);
                data_gas[readings] = (float)gas_parallel[i];

                readings++;
            }
            //TODO: get 5 peaks from all lists
            // vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo entre lecturas
        }
    }
    else {
        printf("NO ENTRO\n");
    }

    // send data
    float num;
    uart_write_bytes(UART_NUM, "BEGIN_READINGS\n", 15);
    for (int i=0; i<length; i++) {
        num = data_temp[i];
        uart_write_bytes(UART_NUM, (char*)&num, sizeof(float));
        num = data_pres[i];
        uart_write_bytes(UART_NUM, (char*)&num, sizeof(float));
        num = data_hum[i];
        uart_write_bytes(UART_NUM, (char*)&num, sizeof(float));
        num = data_gas[i];
        uart_write_bytes(UART_NUM, (char*)&num, sizeof(float));
        uart_write_bytes(UART_NUM, "\n", 1);
    }
    uart_write_bytes(UART_NUM, "FINISH_READINGS\n", 16);
    //Top5 and send
    calc_top5(data_temp, top5_temp, length);
    calc_top5(data_pres, top5_pres, length);
    calc_top5(data_hum, top5_hum, length);
    calc_top5(data_gas, top5_gas, length);
    uart_write_bytes(UART_NUM, "BEGIN_TOP\n", 10);
    for (int i=0; i<5; i++) {
        num = top5_temp[i];
        uart_write_bytes(UART_NUM, (char*)&num, sizeof(float));
        num = top5_pres[i];
        uart_write_bytes(UART_NUM, (char*)&num, sizeof(float));
        num = top5_hum[i];
        uart_write_bytes(UART_NUM, (char*)&num, sizeof(float));
        num = top5_gas[i];
        uart_write_bytes(UART_NUM, (char*)&num, sizeof(float));
        uart_write_bytes(UART_NUM, "\n", 1);
    }
    uart_write_bytes(UART_NUM, "FINISH_TOP\n", 11);
    //free memory
    free(data_temp);
    free(data_pres);
    free(data_hum);
    free(data_gas);
    free(top5_temp);
    free(top5_pres);
    free(top5_hum);
    free(top5_gas);
}

void app_main(void) {
    ESP_ERROR_CHECK(sensor_init());
    bme_get_chipid();
    uart_setup();
    while(1) {
        // 1: Check if continue or not
        char continueResponse[1];
        uart_write_bytes(UART_NUM,"CONTINUE? (Y/N):\0",17);
        while(1) {
            int rLen = serial_read(continueResponse, 1);
            if (rLen > 0) break;
        }
        if (continueResponse[0] == 'N' || continueResponse[0] == 'n') break;
        if (continueResponse[0] != 'Y' && continueResponse[0] != 'y') continue;
        printf("Received response: %c\n", continueResponse[0]);
        // 2: Select power mode
        bme_softreset();
        bme_get_mode();
        char powermodeResponse[1];
        uart_write_bytes(UART_NUM,"SELECT_POWER_MODE\n",18);
        while(1) {
            int rLen = serial_read(powermodeResponse, 1);
            if (rLen > 0) break;
        }
        printf("Received power mode: %c\n", powermodeResponse[0]);
        if (powermodeResponse[0] == 'S') {
            bme_sleep();
            continue;
        }
        if (powermodeResponse[0] == 'F') {
            printf("Forced mode\n");
            bme_forced_mode();
        }
        if (powermodeResponse[0] == 'P') {
            printf("Parallel mode\n");
            bme_parallel_mode();
        }
        bme_read_data(50);
    }
}