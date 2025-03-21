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
#include "hardware/i2c.h"


#define USE_MQTT
// Stepper motor control pins (ULN2003 driver)
#define IN1 13
#define IN2 6
#define IN3 3
#define IN4 2
//EEPROM addresses
#define EEPROM_ADDRESS 0x50
#define EEPROM_DIRECTION_ADDR  0x7FF0  // 1-byte;
#define EEPROM_STEPS_ADDR      0x7FF4  // 4-byte; total_steps is 4 byte int
#define EEPROM_CALIBRATION_ADDR 0x7FF8 // 1-byte; 1 = calibrated; 0 = not calibrated
#define EEPROM_DOOR_STATE_ADDR 0x7FF9 //1 byte; 6 door states
#define EEPROM_STEPS_MOVED_ADDR 0x7FFC //4-byte; steps_moved is 4 byte int
#define EEPROM_PREVIOUS_STATE_ADDR 0x8000 //1 byte; 6 door states
//I2C pins
#define I2C_PORT i2c0
#define SDA_PIN 16
#define SCL_PIN 17


void gpio_irq_handler(uint gpio, uint32_t event_mask);

class Led {
private:
    uint8_t pin;
    bool blinking = false;
    absolute_time_t last_toggle_time;
    uint32_t blink_interval = 1000; // Blink interval in milliseconds
    bool state = false; // Track LED state

public:
    Led(uint8_t pin) : pin(pin) { initialise(); }

    void initialise() {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }

    void on() {
        blinking = false;
        gpio_put(pin, 1);
    }

    void off() {
        blinking = false;
        gpio_put(pin, 0);
    }

    void startBlinking(uint32_t interval) {
        blink_interval = interval;
        blinking = true;
        last_toggle_time = get_absolute_time();
    }

    void stopBlinking() {
        blinking = false;
        gpio_put(pin, 0);
    }

    void update() {
        if (!blinking) return;

        if (absolute_time_diff_us(last_toggle_time, get_absolute_time()) >= blink_interval * 1000) {
            state = !state; // Toggle LED state
            gpio_put(pin, state);
            last_toggle_time = get_absolute_time();
        }
    }
};


class RotaryEncoder {
public:
    static RotaryEncoder *instance;
    uint8_t pinA, pinB;
    int position;
    bool lastA;
    int last_positon;
    int stable_steps;

    RotaryEncoder(uint8_t pinA, uint8_t pinB) : pinA(pinA), pinB(pinB), position(0), lastA(false) {
        instance = this;
        initialize();
    }

    void initialize() {
        gpio_init(pinA);
        gpio_init(pinB);
        gpio_set_dir(pinA, GPIO_IN);
        gpio_set_dir(pinB, GPIO_IN);


        gpio_set_irq_enabled_with_callback(pinA, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_handler);
    }

    void update() {
        bool bState = gpio_get(pinB);

        if (bState == 0) {
            position++; // Myötäpäivään
        } else {
            position--; // Vastapäivään
        }
    }

    bool check_status() {
        if (position == last_positon) {
            stable_steps++; // Jos ei muutosta, lisätään stable_steps
        } else {
            stable_steps = 0; // Jos liike havaittu, nollataan laskuri
            last_positon = position; // Päivitetään viimeisin asento
        }

        if (stable_steps >= 400) { // steppiä jumissa
            std::cout << "ERROR: Motor stuck" << std::endl;
            stable_steps = 0; // Nollataan virheen jälkeen
            return true;
        }
        return false;
    }

    void reset() {
        stable_steps = 0;
        position = 0;
        last_positon = 0;
    }
};

RotaryEncoder *RotaryEncoder::instance = nullptr;

class EEPROM {
public:
    EEPROM() { initialize(); }

    void initialize() {
        i2c_init(I2C_PORT, 100 * 1000); //Initialize the I2C port (i2c0) with a clock speed of 100 kHz.
        gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(SDA_PIN);
        gpio_pull_up(SCL_PIN);

        // Check if the EEPROM is responding
        uint8_t test_byte = 0;
        int ret = i2c_read_blocking(I2C_PORT, EEPROM_ADDRESS, &test_byte, 1, false);
        //read 1 byte from the EEPROM to verify that it is connected
        if (ret == PICO_ERROR_GENERIC) {
            printf("EEPROM NOT FOUND! Check wiring and address.\n"); //read fails. check wiring i guess?
        } else {
            printf("EEPROM detected successfully.\n"); //read is OK
        }
    }


    static bool write(uint16_t mem_address, uint8_t *data, size_t length) {
        //write data to EEPROM
        //mem_address =  The memory address in the EEPROM where the data will be written
        //data = a pointer to the data to be written
        //length = the number of bytes to write
        uint8_t buffer[2 + length]; //hold mem_add and data; 2 first are mem_add; rest data
        buffer[0] = (mem_address >> 8) & 0xFF; //stores the high byte of the memory address
        buffer[1] = mem_address & 0xFF; //Stores the low byte of the memory address
        memcpy(&buffer[2], data, length); // Copies the data into the buffer after the address bytes
        int ret = i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, buffer, 2 + length, false);
        //write the buffer into EEPROM; returns number of bytes written
        sleep_ms(5);
        return ret == (2 + length); // Returns true if the write was successful
    }

    static bool read(uint16_t mem_address, uint8_t *data, size_t length) {
        //read data from EEPROM
        //mem_address =  The memory address in the EEPROM to read from
        //data = A pointer to the buffer where the read data will be stored
        //length = the number of bytes to read
        uint8_t addr_buffer[2] = {static_cast<uint8_t>(mem_address >> 8), static_cast<uint8_t>(mem_address & 0xFF)};
        //A buffer to hold the memory address



        int write_status = i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, addr_buffer, 2, true);
        //Writes the memory address to the EEPROM to set the read pointer; true = not send a stop condition

        if (!write_status) {
            printf("EEPROM Read: Failed to write data to address 0x%X\n", mem_address);
        }
        if (write_status != 2) {
            //if write fails return false
            printf("EEPROM write (read request) failed at address: 0x%X, Status: %d\n", mem_address, write_status);
            return false;
        }



        int read_status = i2c_read_blocking(I2C_PORT, EEPROM_ADDRESS, data, length, false);
        //read lenght-amount bytes from EEPROM to data; false = stop after the read
        if (read_status != length) {
            //if read fails return false

            return false;
        }


        return true;
    }


    void clear() {
        uint8_t zero = 0;
        int32_t zero_steps, zero_steps_moved = 0;

        EEPROM::write(EEPROM_DIRECTION_ADDR, &zero, sizeof(zero)); // Poistetaan suunta
        EEPROM::write(EEPROM_STEPS_ADDR, (uint8_t *) &zero_steps, sizeof(zero_steps)); // Nollataan askeleet
        EEPROM::write(EEPROM_CALIBRATION_ADDR, &zero, sizeof(zero)); // Nollataan kalibrointitila
        EEPROM::write(EEPROM_DOOR_STATE_ADDR, &zero, sizeof(zero)); //clear door state
        EEPROM::write(EEPROM_STEPS_MOVED_ADDR, &zero, sizeof(zero_steps_moved)); //clear steps moved
        EEPROM::write(EEPROM_PREVIOUS_STATE_ADDR, &zero, sizeof(zero)); // Nollataan edellinen tila

        printf("EEPROM cleared!\n");
    }
};


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
public:
    static LimitSwitch *instance1, *instance2;
    uint8_t pin;
    bool triggered;

    LimitSwitch(uint8_t pin) : pin(pin), triggered(false) {
        if (!instance1) instance1 = this;
        else if (!instance2) instance2 = this;
        initialize();
    }

    void initialize() {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
        gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_FALL, true);
    }

    void resetTrigger() {
        triggered = false;
    }

    bool isTriggered() {
        return triggered;
    }
};


LimitSwitch *LimitSwitch::instance1 = nullptr;
LimitSwitch *LimitSwitch::instance2 = nullptr;


//irq handleri jossa käsitellään tapahtuman tyyppi ja toimitaan sen perusteella. tämä pakollista sillä mahdollista vain yksi callback
void gpio_irq_handler(uint gpio, uint32_t events) {
    //

    if (gpio == RotaryEncoder::instance->pinA || gpio == RotaryEncoder::instance->pinB) {
        RotaryEncoder::instance->update();
    }
    if (LimitSwitch::instance1 && gpio == LimitSwitch::instance1->pin) {
        LimitSwitch::instance1->triggered = true;
    }
    if (LimitSwitch::instance2 && gpio == LimitSwitch::instance2->pin) {
        LimitSwitch::instance2->triggered = true;
    }
}


enum class DoorState { CLOSED, OPEN, STOPPED, OPENING, CLOSING, STUCK };


class StepperMotor {
private:
    uint8_t in1, in2, in3, in4;
    int position = 0;

    int total_steps = 0;
    int steps_moved = 0;
    bool direction = true;

    const uint8_t step_sequence[8][4] = {
        {1, 0, 0, 1}, {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0},
        {0, 1, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}
    };

public:
    DoorState state = DoorState::CLOSED; // oletus tila
    DoorState previous_state = DoorState::CLOSED;


    StepperMotor(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4)
        : in1(in1), in2(in2), in3(in3), in4(in4) {
        initialize();
    }

    bool calibrated = false;

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

    void saveStepsMovedToEEPROM() {
        //save steps moved
        // Write steps moved to EEPROM
        bool write_success = EEPROM::write(EEPROM_STEPS_MOVED_ADDR, (uint8_t *) &steps_moved, sizeof(steps_moved));

        if (!write_success) {
            printf("Failed to save steps moved %d to EEPROM address 0x%04X\n", steps_moved, EEPROM_STEPS_MOVED_ADDR);
        }
    }

    void saveDirectionToEEPROM() {
        //save direction
        // Write direction to EEPROM
        bool write_success = EEPROM::write(EEPROM_DIRECTION_ADDR, (uint8_t *) &direction, sizeof(direction));
        if (!write_success) {
            printf("Failed to save direction %d to EEPROM address 0x%04X\n", direction, EEPROM_DIRECTION_ADDR);
        }
    }

    void saveDoorStateToEEPROM() {
        uint8_t state_value = static_cast<uint8_t>(state); // Convert enum to byte
        if (EEPROM::write(EEPROM_DOOR_STATE_ADDR, &state_value, sizeof(state_value))) {
        } else {
            printf("Failed to save DoorState to EEPROM!\n");
        }
    }

    void savePreviousDoorStateToEEPROM() {
        uint8_t previous_state_value = static_cast<uint8_t>(previous_state); // Convert enum to byte
        if (EEPROM::write(EEPROM_PREVIOUS_STATE_ADDR, &previous_state_value, sizeof(previous_state_value))) {
        } else {
            printf("Failed to save Previous DoorState: %d to EEPROM!\n", previous_state_value);
        }
    }

    void loadStepsMovedFromEEPROM() {
        int read_steps_moved;
        if (EEPROM::read(EEPROM_STEPS_MOVED_ADDR, (uint8_t *) &read_steps_moved, sizeof(read_steps_moved))) {
            steps_moved = read_steps_moved;
            return;
        }
        steps_moved = 0;
        printf("Invalid steps moved in EEPROM, defaulting to %d\n", steps_moved);
    }

    void loadDoorStateFromEEPROM() {
        uint8_t state_value;
        if (EEPROM::read(EEPROM_DOOR_STATE_ADDR, &state_value, sizeof(state_value))) {
            if (state_value <= static_cast<uint8_t>(DoorState::STUCK)) {
                // Validate range
                state = static_cast<DoorState>(state_value);
                printf("DoorState loaded: %d\n", state_value);
                return;
            }
        }
        printf("Invalid DoorState in EEPROM, set to default .\n");
        state = DoorState::CLOSED; // Default if EEPROM is corrupt
    }

    void loadPreviousStateFromEEPROM() {
        uint8_t read_prev_state;
        if (EEPROM::read(EEPROM_PREVIOUS_STATE_ADDR, &read_prev_state, sizeof(read_prev_state))) {
            printf("Previous DoorState loaded: %d\n", read_prev_state);
            previous_state = static_cast<DoorState>(read_prev_state);
            return;
        }
        previous_state = DoorState::CLOSED;
        printf("Invalid Previous DoorState in EEPROM, set to default.\n");
    }


    bool loadDirectionFromEEPROM() {
        //load only dir from EEPROM
        bool eeprom_direction; //boolean for dir
        //Reads the motor's direction from the EEPROM at address 0x7FF0
        //The data is stored in the read_dir variable
        //The success flag is updated to false if the read operation fails
        bool success = EEPROM::read(EEPROM_DIRECTION_ADDR, (uint8_t *) &eeprom_direction, sizeof(eeprom_direction));

        if (success) {
            // If EEPROM contains valid data
            direction = eeprom_direction;
        } else {
            // If EEPROM read fails, use the default direction
            printf("Failed to load direction from EEPROM. Using default direction: %d\n", direction);
            return direction;
        }
    }

    void startOpening() {
        if (state == DoorState::CLOSED || state == DoorState::STOPPED) {
            previous_state = state;
            state = DoorState::OPENING;
            direction = !direction; // kulku suunnan vaihto
            printf("Door starting to open...\n");
            saveDoorStateToEEPROM();
            savePreviousDoorStateToEEPROM();
            saveDirectionToEEPROM();
        }
    }

    void startClosing() {
        if (state == DoorState::OPEN || state == DoorState::STOPPED) {
            previous_state = state;
            state = DoorState::CLOSING;
            direction = !direction; // vaihdetaan suunta
            printf("Door starting to close...\n");
            saveDoorStateToEEPROM();
            savePreviousDoorStateToEEPROM();
            saveDirectionToEEPROM();
        }
    }

    void stop() {
        if (state == DoorState::OPENING || state == DoorState::CLOSING) {
            previous_state = state;
            state = DoorState::STOPPED;
            direction = !direction;
            printf("Door stopped!\n");
            saveDoorStateToEEPROM();
            savePreviousDoorStateToEEPROM();
            saveDirectionToEEPROM();
        }
    }

    void move(Button &stopButton, RotaryEncoder &encoder) {
        int steps_taken = 0;

        while (steps_taken < total_steps) {
            step(direction);
            sleep_ms(1); //
            steps_moved++;
            steps_taken++;

            // Jos nappia painetaan, pysäytetään moottori
            if (stopButton.isPressed()) {
                stop();
                printf("Movement interrupted by button press!\n");
                saveStepsMovedToEEPROM();
                return;
            }

            // Tarkistetaan, onko moottori juumissa
            if (encoder.check_status()) {
                state = DoorState::STUCK;
                std::cout << "Motor stuck! Please calibrate again." << std::endl;
                saveDoorStateToEEPROM();
                return;
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
        saveDirectionToEEPROM(); //save dir to eeprom
        saveDoorStateToEEPROM();
    }

    void move_back(RotaryEncoder &encoder) {
        int steps = 0;

        while (steps < steps_moved) {
            step(direction);
            sleep_ms(1); //
            steps++;

            if (encoder.check_status()) {
                state = DoorState::STUCK;
                std::cout << "Motor stuck please calibrate again" << std::endl;
                saveDoorStateToEEPROM();
                return;
            }
        }

        if (previous_state == DoorState::CLOSING) {
            state = DoorState::OPEN;
            std::cout << "Door open" << std::endl;
        } else if (previous_state == DoorState::OPENING) {
            state = DoorState::CLOSED;
            std::cout << "Door closed" << std::endl;
        }
        steps_moved = 0;
        saveDirectionToEEPROM(); //save dir to eeprom
        saveDoorStateToEEPROM();
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

    void calibrate(int delay_ms, LimitSwitch &switch1, LimitSwitch &switch2, RotaryEncoder &encoder, Led &led) {
        int step_count = 0;
        int tries = 0;
        total_steps = 0; // nollataan
        steps_moved = 0; // nollataan
        LimitSwitch *first_triggered = nullptr;
        encoder.reset(); // nollataan asento

        // Nollataan kytkimet ennen kalibroinnin aloitusta
        switch1.resetTrigger();
        switch2.resetTrigger();
        led.off(); // pois päältä vika tila ledi

        std::cout << "Starting calibration" << std::endl;

        if(state== DoorState::STUCK) {// vaihetaan kulkusuunta ja koitetaan kalibroida
            direction = !direction;
        }

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
            if (encoder.check_status()) {
                direction = !direction;
                tries ++;
                if (tries < 3) {
                    std::cout << "Changing direction" << std::endl;
                }
            }
            if(tries == 3) { // jos jää kolmesti jumiin keskeytetään
                std::cout << "Motor stuck. Could not calibrate"<< std::endl;
                return;
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
                break;
            }
            if (encoder.check_status()) {
                calibrated = false;
                std::cout << "Could not calibrate" << std::endl;
                led.startBlinking(500);
                return;
            }
        }
        // Tallennetaan askelmäärä
        total_steps = step_count; // miinustetaan jotta ei osu seinään uudestaan
        calibrated = true;
        tries = 0;
        state = DoorState::CLOSED; // oletus tila
        std::cout << "Motor calibrated " << std::endl;
        saveToEEPROM(); //save the data to EEPROM
    }

    void saveToEEPROM() {
        //save motor's dir, calib, total steps and steps moved to EEPROM
        // Write direction; calls write method
        if (!EEPROM::write(EEPROM_DIRECTION_ADDR, (uint8_t *) &direction, sizeof(direction))) {
            printf("Failed to write direction to EEPROM!\n");
        }
        sleep_ms(10); // Add delay between writes

        // Write total steps; calls write method
        if (!EEPROM::write(EEPROM_STEPS_ADDR, (uint8_t *) &total_steps, sizeof(total_steps))) {
            printf("Failed to write total steps to EEPROM!\n");
        }
        sleep_ms(10); // Add delay between writes

        // Write calibration status; calls write method
        if (!EEPROM::write(EEPROM_CALIBRATION_ADDR, (uint8_t *) &calibrated, sizeof(calibrated))) {
            printf("Failed to write calibration status to EEPROM!\n");
        }
        sleep_ms(10); // Add delay between writes


        // Read back and verify; calls read method
        bool dir;
        int steps;
        bool calib;
        // int steps_moved;
        EEPROM::read(EEPROM_DIRECTION_ADDR, (uint8_t *) &dir, sizeof(dir));
        EEPROM::read(EEPROM_STEPS_ADDR, (uint8_t *) &steps, sizeof(steps));
        EEPROM::read(EEPROM_CALIBRATION_ADDR, (uint8_t *) &calib, sizeof(calib));
        // EEPROM::read(EEPROM_STEPS_MOVED_ADDR, (uint8_t*)&steps_moved, sizeof(steps_moved));
    }

    void loadFromEEPROM() {
        //load dir, calib, total steps and steps moved from EEPROM
        bool success = true; // A boolean flag to track if read is OK
        int read_steps; //read_steps_moved //store total steps and steps moved read from EEPROM
        bool read_dir, read_calib; //booleans to store dir and calib read from EEPROM

        //Reads the motor's direction from the EEPROM at address 0x7FF0
        //The data is stored in the read_dir variable
        //The success flag is updated to false if the read operation fails
        success &= EEPROM::read(EEPROM_DIRECTION_ADDR, (uint8_t *) &read_dir, sizeof(read_dir));


        //Reads the total steps from the EEPROM at address 0x7FF4
        //The data is stored in the read_steps variable
        //The success flag is updated to false if the read operation fails
        success &= EEPROM::read(EEPROM_STEPS_ADDR, (uint8_t *) &read_steps, sizeof(read_steps));

        //Reads the calibration status from the EEPROM at address 0x7FF8
        //The data is stored in the read_calib variable
        //The success flag is updated to false if the read operation fails
        success &= EEPROM::read(EEPROM_CALIBRATION_ADDR, (uint8_t *) &read_calib, sizeof(read_calib));

        //Reads the steps moved from the EEPROM at address 0x7FFD
        //The data is stored in the read_calib variable
        //The success flag is updated to false if the read operation fails
        //success &= EEPROM::read(EEPROM_STEPS_MOVED_ADDR, (uint8_t*)&read_steps_moved, sizeof(read_steps_moved));
        //printf("Reading steps moved from EEPROM...\n");

        if (!success) {
            //check if any of the read operations failed; reset to default values if fail
            printf("EEPROM read failed! Resetting to defaults.\n");
            direction = true;
            total_steps = 0;
            calibrated = false;
            // steps_moved = 0;
            return;
        }

        // Validate amount of steps
        if (read_steps < 0 || read_steps > 100000) {
            //there was an issue with the steps amount from EEPROM
            //so added this for debug purposes
            printf("Invalid EEPROM total_steps! Resetting.\n");
            total_steps = 0; //total steps to 0 if fail validation
            calibrated = false; // not calibrated if fail validation
        } else {
            total_steps = read_steps; //If the data is valid, assign read_steps to total_steps
        }

        if (read_calib != 0 && read_calib != 1) {
            // Checks if the calibration status is neither 0 nor 1
            printf("Invalid EEPROM calibration flag! Resetting.\n");
            calibrated = false; // not calibrated if fail validation
        } else {
            calibrated = read_calib; //assign read_calib to calibrated if validation OK
        }

        if (read_dir != 0 && read_dir != 1) {
            //Checks if the direction is neither 0 nor 1
            printf("Invalid EEPROM direction flag! Resetting.\n");
            direction = true; //dir to default (true) if fail
        } else {
            direction = read_dir; //if valid, assign read_dir to direction
        }
        printf("Loaded from EEPROM -> Steps: %d, Direction: %d, Calibrated: %d\n",
               total_steps, direction, calibrated);

    }

    //getter function; returns the current calibration status of the stepper motor
    bool isCalibrated() { return calibrated; }
};


int main() {
    stdio_init_all();
    StepperMotor motor(IN1, IN2, IN3, IN4);
    RotaryEncoder encoder(27, 28);
    Button button1(8); //sw1
    Button button2(9); // sw0
    Button button3(7); // sw2
    LimitSwitch limitSwitch1(4);
    LimitSwitch limitSwitch2(5);
    Led led1(20);
    EEPROM eeprom;
    eeprom.initialize();


    printf("\nBoot\n");
    motor.loadFromEEPROM();
    motor.loadStepsMovedFromEEPROM();
    motor.loadDoorStateFromEEPROM();
    motor.loadPreviousStateFromEEPROM();



    if (motor.state == DoorState::CLOSING || motor.state == DoorState::OPENING) {
        motor.calibrated = false;
        std::cout << "Motor was stopped during movement. Please calibrate the motor again" << std::endl;
    }


    uint32_t button2PressTime = 0; //uint to save time that went by after pressing button2
    uint32_t button3PressTime = 0; //uint to save time that went by after pressing button3

    while (true) {
        led1.update();

        if (button2.isPressed()) {
            //check if button2 is pressed
            //if button2 is pressed then save the time
            // that has gone AFTER release of the button2
            button2PressTime = to_ms_since_boot(get_absolute_time());
        }

        if (button3.isPressed()) {
            //check if button3 is pressed
            //if button3 is pressed then save the time
            // that has gone AFTER release of the button3
            button3PressTime = to_ms_since_boot(get_absolute_time());
        }

        // Check if both buttons were pressed within 500ms of each other
        if (button2PressTime > 0 && button3PressTime > 0 && abs((int) (button2PressTime - button3PressTime)) <=
            500) {
            eeprom.clear();
            motor.calibrate(1, limitSwitch1, limitSwitch2, encoder, led1); //calibrate motor

            //reset timestamps
            button2PressTime = 0;
            button3PressTime = 0;
        }

        if (motor.calibrated) {
            switch (motor.getState()) {
                case DoorState::CLOSED:
                    if (button1.isPressed()) {
                        motor.startOpening();
                    }
                    break;
                case DoorState::OPEN:
                    if (button1.isPressed()) {
                        motor.startClosing();
                    }
                    break;
                case DoorState::OPENING:
                case DoorState::CLOSING:
                    // Jos nappia painetaan, pysäytetään moottori, muuten siirretään askel kerrallaan.
                    if (button1.isPressed()) {
                        motor.stop();
                    } else {
                        motor.move(button1, encoder);
                    }
                    break;
                case DoorState::STOPPED:
                    if (button1.isPressed()) {
                        motor.move_back(encoder);
                    }
                    break;
                case DoorState::STUCK:
                    motor.stop();
                    led1.startBlinking(500);
                    motor.calibrated = false;
                    motor.saveToEEPROM(); // tallenetaan ei calibroitu epromiin
                    break;
                default:
                    DoorState::STOPPED;
                    break;
            }
        }
    }
};
