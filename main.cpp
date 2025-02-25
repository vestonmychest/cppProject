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
#include <iostream>

#define USE_MQTT


#include <cstdio>
#include "pico/stdlib.h"

// Stepper motor control pins (ULN2003 driver)
#define IN1 2
#define IN2 3
#define IN3 6
#define IN4 13

class StepperMotor {
private:
    uint8_t in1, in2, in3, in4;
    int position = 0;

    // Correct 8-step half-step sequence
    const uint8_t step_sequence[8][4] = {
        {1, 0, 0, 1}, {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0},
        {0, 1, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}
    };

public:
    StepperMotor(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4)
        : in1(in1), in2(in2), in3(in3), in4(in4) {}

    void initialize() {
        gpio_init(in1); gpio_set_dir(in1, GPIO_OUT);
        gpio_init(in2); gpio_set_dir(in2, GPIO_OUT);
        gpio_init(in3); gpio_set_dir(in3, GPIO_OUT);
        gpio_init(in4); gpio_set_dir(in4, GPIO_OUT);
    }

    void step(int steps, bool direction, int delay_ms = 0) {
        printf("Starting motor movement: %d steps, direction: %s\n", steps, direction ? "FORWARD" : "REVERSE");

        for (int i = 0; i < steps; i++) {
            position = (direction) ? (position + 1) % 8 : (position - 1 + 8) % 8;

            gpio_put(in1, step_sequence[position][0]);
            gpio_put(in2, step_sequence[position][1]);
            gpio_put(in3, step_sequence[position][2]);
            gpio_put(in4, step_sequence[position][3]);

            printf("Step: %d, Position: %d\n", i, position);

            sleep_ms(delay_ms);

            // if (i % 1024 == 0 && i > 0) {
            //     printf("Cooling motor driver...\n");
            //     sleep_ms(2000); // Allow driver to cool down
            // }
        }

        stop(); // Turn off all coils
        printf("Motor movement complete!\n");
    }

    void stop() {
        gpio_put(in1, 0);
        gpio_put(in2, 0);
        gpio_put(in3, 0);
        gpio_put(in4, 0);
    }
};

class Button {
private:
    uint8_t pin;

public:
    Button(uint8_t pin) : pin(pin) {}

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
    motor.initialize();
    Button button(8);
    button.initialize();

    bool direction = false;
    printf("\nBoot\n");

    while (true) {
        if (button.isPressed()) {
            printf("Button pressed - Moving stepper motor\n");
            motor.step(4096, direction);  // Move 2 full shaft rotations
            sleep_ms(500);
        }
    }
}
