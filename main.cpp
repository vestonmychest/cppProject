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
//I2C pins
#define I2C_PORT i2c0
#define SDA_PIN 16
#define SCL_PIN 17


class RotaryEncoder {
private:
    uint8_t pinA, pinB; // pins
    int position;  // seurataan encoderin asentoa
    bool lastA;  // viimeisin signaali

    static RotaryEncoder* instance;  // Yksi instanssi keskeytyksen käsittelyä varten

    static void gpio_irq_handler(uint gpio, uint32_t events) {
        if (instance) {
            instance->update();
        }
    }

public:
    RotaryEncoder(uint8_t pinA, uint8_t pinB)
        : pinA(pinA), pinB(pinB), position(0), lastA(false) {
        initialize();
    }

    void initialize() {
        gpio_init(pinA);
        gpio_init(pinB);
        gpio_set_dir(pinA, GPIO_IN);
        gpio_set_dir(pinB, GPIO_IN);
        gpio_pull_up(pinA);
        gpio_pull_up(pinB);

        lastA = gpio_get(pinA);  // Alustetaan viimeisin A-signaalin tila

        instance = this;
        gpio_set_irq_enabled_with_callback(pinA, GPIO_IRQ_EDGE_RISE , true, &gpio_irq_handler);
    }


    void update() {
        bool aState = gpio_get(pinA);
        bool bState = gpio_get(pinB);

        if (aState != lastA) {  // **Tunnistetaan nousureuna A:ssa**
            if (bState == aState) {
                position++;  //  **Myötäpäivään**
            } else {
                position--;  //  **Vastapäivään**
            }
            printf("[ENCODER] Position updated: %d\n", position);
        }
        lastA = aState;
    }

    int getPosition() const {
        return position;
    }

    void reset() {
        position = 0;
    }

    bool status_check(int &stable_steps) {
        static int last_position = position;

        if (position == last_position) {
            stable_steps++;  // Jos ei liikettä, lisätään laskuriin
        } else {
            stable_steps = 0; // Jos liike havaitaan, nollataan laskuri
        }

        last_position = position;  // Päivitetään viimeisin sijainti

        return stable_steps > 8000;  // Palauttaa **true**, jos moottori on jumissa
    }

};

// Alustetaan instanssi osoitin (tarvitaan keskeytyksen käsittelyyn)
RotaryEncoder* RotaryEncoder::instance = nullptr;

class EEPROM {
public:
    EEPROM(){ initialize(); }

    void initialize(){
        i2c_init(I2C_PORT, 100 * 1000); //Initialize the I2C port (i2c0) with a clock speed of 100 kHz.
        gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(SDA_PIN);
        gpio_pull_up(SCL_PIN);

        // Check if the EEPROM is responding
        uint8_t test_byte = 0;
        int ret = i2c_read_blocking(I2C_PORT, EEPROM_ADDRESS, &test_byte, 1, false); //read 1 byte from the EEPROM to verify that it is connected
        if (ret == PICO_ERROR_GENERIC) {
            printf("EEPROM NOT FOUND! Check wiring and address.\n"); //read fails. check wiring i guess?
        } else {
            printf("EEPROM detected successfully.\n"); //read is OK
        }
    }


    static bool write(uint16_t mem_address, uint8_t *data, size_t length) { //write data to EEPROM
        //mem_address =  The memory address in the EEPROM where the data will be written
        //data = a pointer to the data to be written
        //length = the number of bytes to write
        uint8_t buffer[2 + length]; //hold mem_add and data; 2 first are mem_add; rest data
        buffer[0] = (mem_address >> 8) & 0xFF; //stores the high byte of the memory address
        buffer[1] = mem_address & 0xFF; //Stores the low byte of the memory address
        memcpy(&buffer[2], data, length); // Copies the data into the buffer after the address bytes
        int ret = i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, buffer, 2 + length, false); //write the buffer into EEPROM; returns number of bytes written
        sleep_ms(5);
        return ret == (2 + length); // Returns true if the write was successful
    }

    static bool read(uint16_t mem_address, uint8_t *data, size_t length) { //read data from EEPROM
        //mem_address =  The memory address in the EEPROM to read from
        //data = A pointer to the buffer where the read data will be stored
        //length = the number of bytes to read
        uint8_t addr_buffer[2] = { static_cast<uint8_t>(mem_address >> 8), static_cast<uint8_t>(mem_address & 0xFF) }; //A buffer to hold the memory address

        printf("EEPROM Read: Requesting data from address 0x%X\n", mem_address);

        int write_status = i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, addr_buffer, 2, true); //Writes the memory address to the EEPROM to set the read pointer; true = not send a stop condition
        printf("EEPROM Read: Writing data to address 0x%X\n", mem_address);
        if (!write_status) {
            printf("EEPROM Read: Failed to write data to address 0x%X\n", mem_address);
        }
        if (write_status != 2) { //if write fails return false
            printf("EEPROM write (read request) failed at address: 0x%X, Status: %d\n", mem_address, write_status);
            return false;
        }

        printf("EEPROM Read: Request successful, reading data...\n");

        int read_status = i2c_read_blocking(I2C_PORT, EEPROM_ADDRESS, data, length, false); //read lenght-amount bytes from EEPROM to data; false = stop after the read
        if (read_status != length) { //if read fails return false
            printf("EEPROM read failed at address: 0x%X, Expected: %d, Got: %d\n",
                   mem_address, length, read_status);
            return false;
        }

        printf("EEPROM Read: Data successfully read from 0x%X\n", mem_address);
        return true;
    }


    void clear() {
        uint8_t zero = 0;
        int32_t zero_steps = 0;

        EEPROM::write(EEPROM_DIRECTION_ADDR, &zero, sizeof(zero));  // Poistetaan suunta
        EEPROM::write(EEPROM_STEPS_ADDR, (uint8_t*)&zero_steps, sizeof(zero_steps));  // Nollataan askeleet
        EEPROM::write(EEPROM_CALIBRATION_ADDR, &zero, sizeof(zero));  // Nollataan kalibrointitila

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
    bool direction = true;

    const uint8_t step_sequence[8][4] = {
        {1, 0, 0, 1}, {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0},
        {0, 1, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}
    };

public:
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

    void saveDirectionToEEPROM() { //save direction
        // Write direction to EEPROM
        bool write_success = EEPROM::write(EEPROM_DIRECTION_ADDR, (uint8_t*)&direction, sizeof(direction));

        if (write_success) {
            printf("Successfully saved direction %d to EEPROM address 0x%04X\n", direction, EEPROM_DIRECTION_ADDR);
        } else {
            printf("Failed to save direction %d to EEPROM address 0x%04X\n", direction, EEPROM_DIRECTION_ADDR);
        }
    }

    bool loadDirectionFromEEPROM() { //load only dir from EEPROM
        bool eeprom_direction; //boolean for dir
        //Reads the motor's direction from the EEPROM at address 0x7FF0
        //The data is stored in the read_dir variable
        //The success flag is updated to false if the read operation fails
        bool success = EEPROM::read(EEPROM_DIRECTION_ADDR, (uint8_t*)&eeprom_direction, sizeof(eeprom_direction));

        if (success) {
            // If EEPROM contains valid data, flip the direction
            direction = !eeprom_direction;
            printf("Loaded direction from EEPROM: %d. Flipped to: %d\n", eeprom_direction, direction);
            return direction;
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

    void move(Button &stopButton){
        for (int i = 0; i < total_steps; i++) {
            step(direction);
            sleep_ms(1);
            steps_moved++; // Lasketaan liikutut askeleet

            // Jos nappia painetaan, pysäytetään moottori
            if (stopButton.isPressed()) {
                stop();
                printf("Movement interrupted by button press!\n");
                saveDirectionToEEPROM();//save dir to eeprom
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
        saveDirectionToEEPROM();//save dir to eeprom
    }

    void move_back() {
        for (int i = 0; i < steps_moved; i++) {
            step(!direction);
            sleep_ms(1);
        }
        steps_moved = 0;
        if (previous_state == DoorState::CLOSING) {
            state = DoorState::OPEN;
            direction = true;
        } else if (previous_state == DoorState::OPENING) {
            state = DoorState::CLOSED;
            direction = false;
        }
        saveDirectionToEEPROM(); //save dir to eeprom
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

    void calibrate(int delay_ms, LimitSwitch &switch1, LimitSwitch &switch2, RotaryEncoder &encoder) {
        int step_count = 0;
        total_steps = 0;
        steps_moved = 0;
        encoder.reset(); // nollataan asento
        LimitSwitch *first_triggered = nullptr;

        int stable_steps = 0;



        // Nollataan kytkimet ennen kalibroinnin aloitusta
        switch1.resetTrigger();
        switch2.resetTrigger();


        // Ensimmäinen reun
        while (true) {
            step(direction);
            sleep_ms(delay_ms);
            printf("Encoder Position: %d\n", encoder.getPosition());

            if (encoder.status_check(stable_steps)) {  //  Moottori on jumissa (500 askelta ilman liikettä)
                printf("Warning: Motor hit the wall! Reversing direction...\n");
                direction = !direction;  // Vaihda suunta
                stable_steps = 0;
                continue;  // Yritetään uudelleen vastakkaiseen suuntaan
            }


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
        stable_steps = 0;

        while (true) {
            // seurataan vain toista kytkintä
            step(direction);
            step_count++;
            sleep_ms(delay_ms);

            if (encoder.status_check(stable_steps)) {
                printf("Warning: Motor hit the wall! Reversing direction again...\n");
                direction = !direction;
                stable_steps = 0;
                continue;
            }

            if ((first_triggered == &switch1 && switch2.isTriggered()) || (
                    first_triggered == &switch2 && switch1.isTriggered())) {
                break;
            }
        }
        // Tallennetaan askelmäärä
        total_steps = step_count; // miinustetaan jotta ei osu seinään uudestaan
        calibrated = true;
        std::cout << "Total steps: " << total_steps << std::endl;
        saveToEEPROM(); //save the data to EEPROM
    }

    void saveToEEPROM() { //save motor's dir, calib and total steps to EEPROM
        // Write direction; calls write method
        if (!EEPROM::write(EEPROM_DIRECTION_ADDR, (uint8_t*)&direction, sizeof(direction))) {
            printf("Failed to write direction to EEPROM!\n");
        }
        sleep_ms(10); // Add delay between writes

        // Write total steps; calls write method
        if (!EEPROM::write(EEPROM_STEPS_ADDR, (uint8_t*)&total_steps, sizeof(total_steps))) {
            printf("Failed to write total steps to EEPROM!\n");
        }
        sleep_ms(10); // Add delay between writes

        // Write calibration status; calls write method
        if (!EEPROM::write(EEPROM_CALIBRATION_ADDR, (uint8_t*)&calibrated, sizeof(calibrated))) {
            printf("Failed to write calibration status to EEPROM!\n");
        }
        sleep_ms(10); // Add delay between writes

        // Read back and verify; calls read method
        bool dir;
        int steps;
        bool calib;
        EEPROM::read(EEPROM_DIRECTION_ADDR, (uint8_t*)&dir, sizeof(dir));
        EEPROM::read(EEPROM_STEPS_ADDR, (uint8_t*)&steps, sizeof(steps));
        EEPROM::read(EEPROM_CALIBRATION_ADDR, (uint8_t*)&calib, sizeof(calib));

        printf("\nSaved to EEPROM - Steps: %d, Direction: %d, Calibrated: %d\n", steps, dir, calib);
    }

    void loadFromEEPROM() { //load dir, calib and total steps from EEPROM
        printf("Loading EEPROM from EEPROM...\n");
        bool success = true; // A boolean flag to track if read is OK
        int read_steps; //store total steps read from EEPROM
        bool read_dir, read_calib; //booleans to store dir and calib read from EEPROM

        //Reads the motor's direction from the EEPROM at address 0x7FF0
        //The data is stored in the read_dir variable
        //The success flag is updated to false if the read operation fails
        success &= EEPROM::read(EEPROM_DIRECTION_ADDR, (uint8_t*)&read_dir, sizeof(read_dir));
        printf("Reading direction from EEPROM...\n");

        //Reads the total steps from the EEPROM at address 0x7FF4
        //The data is stored in the read_steps variable
        //The success flag is updated to false if the read operation fails
        success &= EEPROM::read(EEPROM_STEPS_ADDR, (uint8_t*)&read_steps, sizeof(read_steps));
        printf("Reading steps from EEPROM...\n");

        //Reads the calibration status from the EEPROM at address 0x7FF8
        //The data is stored in the read_calib variable
        //The success flag is updated to false if the read operation fails
        success &= EEPROM::read(EEPROM_CALIBRATION_ADDR, (uint8_t*)&read_calib, sizeof(read_calib));
        printf("Reading calibration from EEPROM...\n");

        printf("\nRaw EEPROM Data -> Steps: %d, Direction: %d, Calibrated: %d\n",
               read_steps, read_dir, read_calib);

        if (!success) { //check if any of the read operations failed; reset to default values if fail
            printf("EEPROM read failed! Resetting to defaults.\n");
            direction = true;
            total_steps = 0;
            calibrated = false;
            return;
        }

        // Validate amount of steps
        if (read_steps < 0 || read_steps > 100000) { //there was an issue with the steps amount from EEPROM
                                                     //so added this for debug purposes
            printf("Invalid EEPROM total_steps! Resetting.\n");
            total_steps = 0; //total steps to 0 if fail validation
            calibrated = false; // not calibrated if fail validation
        } else {
            total_steps = read_steps; //If the data is valid, assign read_steps to total_steps
        }

        if (read_calib != 0 && read_calib != 1) { // Checks if the calibration status is neither 0 nor 1
            printf("Invalid EEPROM calibration flag! Resetting.\n");
            calibrated = false; // not calibrated if fail validation
        } else {
            calibrated = read_calib; //assign read_calib to calibrated if validation OK
        }

        if (read_dir != 0 && read_dir != 1) { //Checks if the direction is neither 0 nor 1
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
    EEPROM eeprom;
    eeprom.initialize();


    printf("\nBoot\n");

    printf("Encoder Position: %d\n", encoder.getPosition());
    // Load calibration data from EEPROM


    if (motor.isCalibrated()) {
        printf("Motor is calibrated. ");
    } else {
        printf("Motor is not calibrated. Please calibrate.\n");
    }

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

            motor.calibrate(1, limitSwitch1, limitSwitch2, encoder); //calibrate motor
            std::cout << "Motor calibrated\n" << std::endl;

            //reset timestamps
            button2PressTime = 0;
            button3PressTime = 0;
        }

        if (button1.isPressed() && motor.calibrated) { //move the door only if the motor
            DoorState currentState = motor.getState(); // has been calibrated
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
