#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#define LED_R_PIN 12  // LED vermelho
#define LED_G_PIN 13  // LED verde
#define LED_B_PIN 11  // LED azul

#define I2C_PORT i2c0
#define SDA_PIN 0      // GPIO0 para SDA (I2C0)
#define SCL_PIN 1      // GPIO1 para SCL (I2C0)

// Endereço do sensor BH1750
#define BH1750_ADDR 0x23
#define BH1750_CONTINUOUS_HIGH_RES_MODE 0x10

//posição do servo
#define PINO_SERVO 2      // Pino conectado ao fio de controle do servo
#define PERIODO_SERVO 20   // Período de 20ms = 50Hz

void bh1750_init() {
    uint8_t buf[1] = {BH1750_CONTINUOUS_HIGH_RES_MODE};
    i2c_write_blocking(I2C_PORT, BH1750_ADDR, buf, 1, false);
}

float bh1750_read_lux() {
    uint8_t data[2];
    int result = i2c_read_blocking(I2C_PORT, BH1750_ADDR, data, 2, false);
    if (result != 2) {
        printf("Erro ao ler o sensor BH1750\n");
        return -1;
    }
    uint16_t raw = (data[0] << 8) | data[1];
    return raw / 1.2; // Conversão para lux
}

void configurar_leds() {
    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);
    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_init(LED_G_PIN);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
}

void acender_led_por_lux(float lux) {
    if (lux <= 100) {
        gpio_put(LED_B_PIN, 1);
        gpio_put(LED_R_PIN, 0);
        gpio_put(LED_G_PIN, 0);
    } else if (lux <= 300) {
        gpio_put(LED_B_PIN, 1);
        gpio_put(LED_R_PIN, 1);
        gpio_put(LED_G_PIN, 0);
    } else if (lux <= 700) {
        gpio_put(LED_B_PIN, 1);
        gpio_put(LED_R_PIN, 1);
        gpio_put(LED_G_PIN, 1);
    } else {
        // Muita luz — pisca LED alto
        gpio_put(LED_B_PIN, 1);
        gpio_put(LED_R_PIN, 1);
        for (int i = 0; i < 3; i++) {
            gpio_put(LED_G_PIN, 1);
            sleep_ms(150);
            gpio_put(LED_G_PIN, 0);
            sleep_ms(150);
        }
    }
}

void enviar_pulso(uint duty_us) {
    gpio_put(PINO_SERVO, 1);
    sleep_us(duty_us);
    gpio_put(PINO_SERVO, 0);
    sleep_ms(PERIODO_SERVO - (duty_us / 1000));
}

void controlar_servo(float lux){
        // Controle do servo com base na luminosidade
    if (lux < 10) {
        enviar_pulso(500);   // Posição mínima (~0°)
    } else if (lux < 100) {
        enviar_pulso(1000);  // Posição baixa (~45°)
    } else if (lux < 300) {
        enviar_pulso(1500);  // Posição média (~90°)
    } else if (lux < 700) {
        enviar_pulso(1750);  // Posição alta (~135°)
    } else {
        enviar_pulso(2000);  // Posição máxima (~180°)
    }
}

int main() {
    stdio_init_all();

    // Inicializa o canal I2C
    i2c_init(I2C_PORT, 100 * 1000); // 100kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(100);
    bh1750_init();
    configurar_leds();

    // Inicializa pino do servo
    gpio_init(PINO_SERVO);
    gpio_set_dir(PINO_SERVO, GPIO_OUT);


    while (true) {
        float lux = bh1750_read_lux();
        if (lux >= 0) {
            printf("Luminosidade: %.2f lux\n", lux);
            acender_led_por_lux(lux);
            controlar_servo(lux);
        }
        sleep_ms(1000);
    }

    return 0;
}