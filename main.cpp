#include <cstdio>
#include <cstring>
#include <cmath>
#include "pico/stdlib.h"
#include <functional>
#include "pico/time.h"
#include "hardware/timer.h"
#include "uart/PicoUart.h"
#include "mqtt/lwipopts.h"
#include "mqtt/Countdown.h"
#include "mqtt/IPStack.h"
#include "IPStack.h"
#include "Countdown.h"
#include "MQTTClient.h"
#include <iostream>
#include <sys/unistd.h>

#define USE_MQTT

// Stepper motor control pins (ULN2003 driver)
#define IN1 13
#define IN2 6
#define IN3 3
#define IN4 2

class StepperMotor {
private:
    uint8_t in1, in2, in3, in4;
    int position = 0;

    const uint8_t step_sequence[8][4] = {
        {1, 0, 0, 1}, {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0},
        {0, 1, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}
    };

public:
    StepperMotor(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4)
        : in1(in1), in2(in2), in3(in3), in4(in4) {initialize();}

    void initialize() {
        gpio_init(in1);
        gpio_set_dir(in1, GPIO_OUT);
        gpio_init(in2);
        gpio_set_dir(in2, GPIO_OUT);
        gpio_init(in3);
        gpio_set_dir(in3, GPIO_OUT);
        gpio_init(in4);
        gpio_set_dir(in4, GPIO_OUT);
    }

    void move(bool &direction, int delay_ms, std::function<bool()> stopCondition) {
        printf("Starting motor movement with direction control\n");

        while (true) {
            if (stopCondition()) {
                direction = !direction; // Reverse direction immediately
                printf("Limit switch activated! Reversing direction.\n");

                // Move away from the switch before checking again
                for (int i = 0; i < 100; i++) {
                    position = (direction) ? (position + 1) % 8 : (position - 1 + 8) % 8;
                    gpio_put(in1, step_sequence[position][0]);
                    gpio_put(in2, step_sequence[position][1]);
                    gpio_put(in3, step_sequence[position][2]);
                    gpio_put(in4, step_sequence[position][3]);
                    sleep_ms(delay_ms);
                }
            }

            position = (direction) ? (position + 1) % 8 : (position - 1 + 8) % 8;
            gpio_put(in1, step_sequence[position][0]);
            gpio_put(in2, step_sequence[position][1]);
            gpio_put(in3, step_sequence[position][2]);
            gpio_put(in4, step_sequence[position][3]);

            sleep_ms(delay_ms);
        }
    }
};

class LimitSwitch {
private:
    uint8_t pin;

public:
    LimitSwitch(uint8_t pin) : pin(pin) {initialize();}

    void initialize() {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }

    bool isTriggered() {
        return gpio_get(pin) == 0;
    }
};

class Button {
private:
    uint8_t pin;

public:
    Button(uint8_t pin) : pin(pin) {initialize();}

    void initialize() {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }

    bool isPressed() {
        return gpio_get(pin) == 0;
    }
};

int main() {
    stdio_init_all();
    StepperMotor motor(IN1, IN2, IN3, IN4);
    Button button(8);
    LimitSwitch limitSwitch1(4);
    LimitSwitch limitSwitch2(5);

    bool direction = false;
    printf("\nBoot\n");

    while (true) {
        if (button.isPressed()) {
            motor.move(direction, 1,  [&]()
                { return limitSwitch1.isTriggered() || limitSwitch2.isTriggered(); });
        }
    }

    return 0;
}
