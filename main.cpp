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


class LimitSwitch
{
private:
    uint8_t pin;
    bool triggered = false;
    static LimitSwitch *instance1;
    static LimitSwitch *instance2;

    static void gpio_irq_handler(uint gpio, uint32_t events) {
        busy_wait_ms(100);

        if (instance1 && gpio == instance1->pin) { // varmistetaan onko nullptr ja gpio pin
            instance1->triggered = true;
            printf("Limit switch 1 (pin %d) triggered!\n", gpio);
        }
        if (instance2 && gpio == instance2->pin) {
            instance2->triggered = true;
            printf("Limit switch 2 (pin %d) triggered!\n", gpio);
        }
    }

public:

    LimitSwitch(uint8_t pin) : pin(pin) {initialize();}

    void initialize()
    {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);


        if (!instance1) {
            instance1 = this;
        } else if (!instance2)
        {
            instance2 = this;
        }

        gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    }


        bool isTriggered() {
            return triggered;
        }

        void resetTrigger() {
            triggered = false;
        }



};
LimitSwitch* LimitSwitch::instance1 = nullptr; // asetetaan nullptr
LimitSwitch* LimitSwitch::instance2 = nullptr;


class StepperMotor
{
private:
    uint8_t in1, in2, in3, in4;
    int position = 0;
    int total_steps = 0;
    bool calibrated = false;

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
    } // salee turha

    void move_to_other_end(bool direction)
    {
        for (int i = 0; i < total_steps; i++) {
            step(direction);
            sleep_ms(1);
        }
    }

    void step(bool direction) {
        position = (direction) ? (position + 1) % 8 : (position - 1 + 8) % 8;
        gpio_put(in1, step_sequence[position][0]);
        gpio_put(in2, step_sequence[position][1]);
        gpio_put(in3, step_sequence[position][2]);
        gpio_put(in4, step_sequence[position][3]);
    }

    void calibrate(int delay_ms, LimitSwitch& switch1, LimitSwitch& switch2) {
        bool direction = false;
        int step_count = 0;
        LimitSwitch* first_triggered = nullptr;


        // Nollataan kytkimet ennen kalibroinnin aloitusta
        switch1.resetTrigger();
        switch2.resetTrigger();

        // Ensimmäinen reun

        while (true) {
            step(direction);
            sleep_ms(delay_ms);

            if (switch1.isTriggered()) {
                first_triggered = &switch1; // Tallennetaan kumpi kytkin aktivoitui
                break;
            } else if (switch2.isTriggered()) {
                first_triggered = &switch2;
                break;
            }
        }



        // Varmistetaan, että limit-kytkin ei ole enää painettuna
        switch1.resetTrigger();
        switch2.resetTrigger();

        // Käännetään suuntaa ja siirrytään toiseen reunaan
        direction = !direction;



        while (true) { // seurataan vain toista kytkintä
            step(direction);
            step_count++;
            sleep_ms(delay_ms);

            if ((first_triggered == &switch1 && switch2.isTriggered()) || (first_triggered == &switch2 && switch1.isTriggered())) {
                break;
                }
        }

        // Tallennetaan askelmäärä
        total_steps = step_count- 150; // miinustetaan jotta ei osu seinään uudestaan
        std:: cout << "Total steps: " << total_steps << std::endl;


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

    int main()
    {
        stdio_init_all();
        StepperMotor motor(IN1, IN2, IN3, IN4);
        Button button(8);
        LimitSwitch limitSwitch1(4);
        LimitSwitch limitSwitch2(5);


        printf("\nBoot\n");


        while (true) {
            if (button.isPressed()) {
                motor.calibrate(1, limitSwitch1, limitSwitch2);
                motor.move_to_other_end(false);

            }
        }


        }



