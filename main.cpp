
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


class Button {
private:
    uint8_t pin;
    bool lastState = true; // Oletetaan, että nappi on ylös vedettynä alussa
    bool pressed = false; //track if button was pressed

public:
    Button(uint8_t pin) : pin(pin) { initialize(); }

    void initialize() {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }

    bool isPressed() {
        bool currentState = gpio_get(pin) == 0; //low = pressed

        if (!currentState && pressed) {
            // Painettiin juuri alas
            pressed = false;
            return true;
        }

        if (currentState) {
            pressed = true;
        }

        return false; //no new press detected
    }
};

class LimitSwitch {
private:
    uint8_t pin;
    bool triggered = false;
    static LimitSwitch *instance1;
    static LimitSwitch *instance2;

    static void gpio_irq_handler(uint gpio, uint32_t events) {
        busy_wait_ms(100);

        if (instance1 && gpio == instance1->pin) {
            // varmistetaan onko nullptr ja gpio pin
            instance1->triggered = true;
            printf("Limit switch 1 (pin %d) triggered!\n", gpio);
        }
        if (instance2 && gpio == instance2->pin) {
            instance2->triggered = true;
            printf("Limit switch 2 (pin %d) triggered!\n", gpio);
        }
    }

public:
    LimitSwitch(uint8_t pin) : pin(pin) { initialize(); }

    void initialize() {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);


        if (!instance1) {
            instance1 = this;
        } else if (!instance2) {
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

LimitSwitch *LimitSwitch::instance1 = nullptr; // asetetaan nullptr
LimitSwitch *LimitSwitch::instance2 = nullptr;


enum class DoorState { CLOSED, OPEN, STOPPED, OPENING, CLOSING };


class StepperMotor {
private:
    uint8_t in1, in2, in3, in4;
    int position = 0;
    DoorState state = DoorState::CLOSED; // oletus tila
    DoorState previous_state = DoorState::CLOSED;
    int total_steps = 0;
    int steps_moved = 0;
    bool calibrated = false;
    bool direction = true;

    const uint8_t step_sequence[8][4] = {
        {1, 0, 0, 1}, {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0},
        {0, 1, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}
    };

public:
    StepperMotor(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4)
        : in1(in1), in2(in2), in3(in3), in4(in4) { initialize(); }

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

    void startOpening() {
        if (state == DoorState::CLOSED || state == DoorState::STOPPED) {
            previous_state = state;
            state = DoorState::OPENING;
            direction = true;
            printf("Door starting to open...\n");
        }
    }

    void startClosing() {
        if (state == DoorState::OPEN || state == DoorState::STOPPED) {
            previous_state = state;
            state = DoorState::CLOSING;
            direction = false;
            printf("Door starting to close...\n");
        }
    }

    void stop() {
        if (state == DoorState::OPENING || state == DoorState::CLOSING) {
            previous_state = state;
            state = DoorState::STOPPED;
            printf("Door stopped!\n");
        }
    }

    void move(Button &stopButton) {
        for (int i = 0; i < total_steps; i++) {
            step(direction);
            sleep_ms(1);
            steps_moved++; // Lasketaan liikutut askeleet

            // Jos nappia painetaan, pysäytetään moottori
            if (stopButton.isPressed()) {
                stop();
                printf("Movement interrupted by button press!\n");
                return; // Poistutaan funktiosta
            }
        }

        // Jos liike on valmis, päivitetään tila
        if (state == DoorState::OPENING) {
            state = DoorState::OPEN;
            printf("Door open!\n");
        } else if (state == DoorState::CLOSING) {
            state = DoorState::CLOSED;
            printf("Door closed!\n");
        }
        steps_moved = 0;
    }

    void move_back() {
        for (int i = 0; i < steps_moved; i++) {
            step(!direction);
            sleep_ms(1);
        }
        steps_moved = 0;
        if (previous_state == DoorState::CLOSING) {
            state = DoorState::OPEN;
        } else if (previous_state == DoorState::OPENING) {
            state = DoorState::CLOSED;
        }
    }

    DoorState getState() const {
        return state;
    }


    void step(bool direction) {
        position = (direction) ? (position + 1) % 8 : (position - 1 + 8) % 8;
        gpio_put(in1, step_sequence[position][0]);
        gpio_put(in2, step_sequence[position][1]);
        gpio_put(in3, step_sequence[position][2]);
        gpio_put(in4, step_sequence[position][3]);
    }

    void calibrate(int delay_ms, LimitSwitch &switch1, LimitSwitch &switch2) {
        int step_count = 0;
        total_steps = 0;
        LimitSwitch *first_triggered = nullptr;


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

        while (true) {
            // seurataan vain toista kytkintä
            step(direction);
            step_count++;
            sleep_ms(delay_ms);

            if ((first_triggered == &switch1 && switch2.isTriggered()) || (
                    first_triggered == &switch2 && switch1.isTriggered())) {
                step_count -= 200;
                break;
            }
        }
        // Tallennetaan askelmäärä
        total_steps = step_count; // miinustetaan jotta ei osu seinään uudestaan
        calibrated = true;
        std::cout << "Total steps: " << total_steps << std::endl;
    }
};


int main() {
    stdio_init_all();
    StepperMotor motor(IN1, IN2, IN3, IN4);
    Button button1(8); //sw1
    Button button2(9); // sw0
    Button button3(7); // sw2
    LimitSwitch limitSwitch1(4);
    LimitSwitch limitSwitch2(5);


    printf("\nBoot\n");

    uint32_t button2PressTime = 0; //uint to save time that went by after pressing button2
    uint32_t button3PressTime = 0; //uint to save time that went by after pressing button3

    while (true) {

        if (button2.isPressed()) { //check if button2 is pressed
            button2PressTime = to_ms_since_boot(get_absolute_time()); //if button2 is pressed then save the time
        }                                                               // that has gone AFTER release of the button2

        if (button3.isPressed()) { //check if button3 is pressed
            button3PressTime = to_ms_since_boot(get_absolute_time()); //if button3 is pressed then save the time
        }                                                               // that has gone AFTER release of the button3


        // Check if both buttons were pressed within 500ms of each other
        if (button2PressTime > 0 && button3PressTime > 0 && abs((int)(button2PressTime - button3PressTime)) <= 500) {

            motor.calibrate(1, limitSwitch1, limitSwitch2); //calibrate motor
            std::cout << "Motor calibrated" << std::endl;

            //reset timestamps
            button2PressTime = 0;
            button3PressTime = 0;
        }

        if (button1.isPressed()) {
            DoorState currentState = motor.getState();
            if (currentState == DoorState::CLOSED) {
                motor.startOpening();
            } else if (currentState == DoorState::OPEN) {
                motor.startClosing();
            } else if (currentState == DoorState::OPENING || currentState == DoorState::CLOSING) {
                motor.stop(); // Pysäytä moottori jos se liikkuu
            } else if (currentState == DoorState::STOPPED) {
                motor.move_back();
            }


            // Jos moottori on liikkeessä, liikutetaan sitä askel kerrallaan
            if (motor.getState() == DoorState::OPENING || motor.getState() == DoorState::CLOSING) {
                motor.move(button1);
            }
        }
    }
}
