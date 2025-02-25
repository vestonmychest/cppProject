#include <cstdio>
#include <cstring>
#include <cmath>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/timer.h"
#include "uart/PicoUart.h"
#include "mqtt/lwipopts.h"
#include "mqtt/Countdown.h"
#include "mqtt/IPStack.h"
#include "IPStack.h"
#include "Countdown.h"
#include "MQTTClient.h"

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#if 0
#define UART_NR 0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#else
#define UART_NR 1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#endif


#define USE_MQTT



class StepperMotor{
private:
    uint8_t pin1, pin2 ,pin3, pin4;
    public:
    StepperMotor(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4):pin1(pin1), pin2(pin2), pin3(pin3), pin4(pin4) {}

    void initialize() {
            gpio_init(pin1);
            gpio_set_dir(pin1, GPIO_OUT);
            gpio_init(pin2);
            gpio_set_dir(pin2, GPIO_OUT);
            gpio_init(pin3);
            gpio_set_dir(pin3, GPIO_OUT);
            gpio_init(pin4);
            gpio_set_dir(pin4, GPIO_OUT);


    }

};
class Button {
private:
    uint8_t pin;

    public :
        Button(uint pin, bool pullup = true, bool invert = false) : pin(pin) {}

    void initialize() {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin); // Käytetään sisäistä pull-up vastusta
    }

    bool ispressed() {
        return gpio_get(pin) == 0;
    }

};
int main() {

    stdio_init_all();
    Button button(8);
    button.initialize(); // Alustetaan painike

    while (true) {
        if (button.ispressed()) {
            printf("pressed\n");
        }

    }




    printf("\nBoot\n");
}