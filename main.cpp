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

    // Reversed 8-step half-step sequence (for smoother motion)
    const uint8_t step_sequence[8][4] = {
        {1, 0, 0, 1},
        {1, 0, 0, 0},
        {1, 1, 0, 0},
        {0, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 1},
        {0, 0, 0, 1}
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

    void step(int steps, bool direction, int delay_ms = 1) {
        for (int i = 0; i < steps; i++) {
            int step_index = direction ? i % 8 : (7 - (i % 8)); // Forward or Reverse
            gpio_put(in1, step_sequence[step_index][0]);
            gpio_put(in2, step_sequence[step_index][1]);
            gpio_put(in3, step_sequence[step_index][2]);
            gpio_put(in4, step_sequence[step_index][3]);
            sleep_ms(delay_ms);
        }
        sleep_ms(1000); // Small pause to allow coils to reset
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

    printf("\nBoot\n");

    while (true) {
        if (button.isPressed()) {
            printf("Button pressed - Moving stepper motor\n");
            motor.step(2048, true);  // One full revolution in half-steps
            sleep_ms(500);
        }
    }
}
