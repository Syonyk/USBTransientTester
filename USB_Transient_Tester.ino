#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <LiteSerialLogger.h>

// Initialize the DF Robot display with the proper pins.
// This is using the stock LiquidCrystal library - no I2C support needed!
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Shift register clock, data, and latch pins.  All digital IO pins.
#define SR_CLOCK 2
#define SR_DATA 11
#define SR_LATCH 3

// Current, in 0.05A steps, per shift register output pin.
// 20 - 1A - 5Ω resistors.  Etc.
const uint8_t current_per_pin[8] = {20, 20, 10, 5, 5, 1, 1, 1};

Adafruit_ADS1115 ads;

// https://www.dfrobot.com/wiki/index.php/LCD_KeyPad_Shield_For_Arduino_SKU:_DFR0009
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// How long after a button press to delay for debounce and scroll.
#define BUTTON_DELAY_MS 200

/**
 * Store the digits of the current and time.  This is the "display 
 * representation" that is converted for actual use on test start.
 */
uint8_t amps[2][3];
uint8_t time[2][4];

/**
 * A bit-packed struct for the cursor position updates.  This is the core of 
 * the menu configuration.  It relies on the fact that the upper and lower rows
 * are identical except for some text.
 * 
 * left_col and right_col are the corresponding columns to move the cursor to on
 * left and right button presses.  This adjusts the cursor position to the new
 * column directly.
 * 
 * left_toggle_row and right_toggle_row indicate if the row should be toggled on
 * a left or right press - this handles row wrapping as you move off the end of
 * one row or another as there are only two rows.
 * 
 * is_amps/amps_pos indicate if the positions are for amps, and indicate the
 * decimal position in amps.  Same for ms.  These are used to update the proper
 * decimal position in the array.
 */
typedef struct {
    // Column to move to on left key.
    uint8_t left_col : 4;
    // Column to move to on right key.
    uint8_t right_col : 4;
    // Toggle row on left/right.
    uint8_t left_toggle_row : 1;
    uint8_t right_toggle_row : 1;
    uint8_t is_amps : 1;
    uint8_t amps_pos : 2;
    uint8_t is_ms : 1;
    uint8_t ms_pos : 2;
} cursor_update;


// Populate an array of the above structs with the needed information.
const cursor_update cursor_positions[16] = {
    //  L   R  LT RT IA AP IM MP
    { 0, 0, 0, 0, 0, 0, 0, 0}, // 0  "H"
    { 0, 0, 0, 0, 0, 0, 0, 0}, // 1  "i"
    { 0, 0, 0, 0, 0, 0, 0, 0}, // 2  ":"
    {13, 5, 1, 0, 1, 0, 0, 0}, // 3  1s of amps
    { 0, 0, 0, 0, 0, 0, 0, 0}, // 4  "."
    { 3, 6, 0, 0, 1, 1, 0, 0}, // 5  tenths of amps
    { 5, 10, 0, 0, 1, 2, 0, 0}, // 6  hundredths of amps
    { 0, 0, 0, 0, 0, 0, 0, 0}, // 7  "A"
    { 0, 0, 0, 0, 0, 0, 0, 0}, // 8  " "
    { 0, 0, 0, 0, 0, 0, 0, 0}, // 9  " "
    { 6, 11, 0, 0, 0, 0, 1, 0}, // 10 1000s of ms
    {10, 12, 0, 0, 0, 0, 1, 1}, // 11 100s of ms
    {11, 13, 0, 0, 0, 0, 1, 2}, // 12 10s of ms
    {12, 3, 0, 1, 0, 0, 1, 3}, // 13 1s of ms
    { 0, 0, 0, 0, 0, 0, 0, 0}, // 14 "m"
    { 0, 0, 0, 0, 0, 0, 0, 0}, // 15 "s"
};

/**
 * For updating amps, they can only be updated 0.05A at a time.  This structure
 * and associated array handles this - it sets the max value for each position
 * and the increment value for each position.
 * 
 * Note that with the bit packing, each element is only a single byte.
 */
typedef struct {
    // Max value for this position.
    uint8_t max : 4;
    // How much to increment by.
    uint8_t increment : 3;
} amps_digits;

const amps_digits amps_positions[3] = {
    {3, 1},
    {10, 1},
    {10, 5}
};

/**
 * One byte of cursor information.  Row is 0-1, col is 0-15, on is binary, and 
 * update indicates if the cursor needs to be updated on the next refresh.  Only
 * updating if needed avoids flicker of the cursor during normal operation.
 */
struct {
    uint8_t row : 1;
    uint8_t col : 4;
    uint8_t on : 1;
    uint8_t update : 1;
} cursor;

void setup() {
    // Ensure the resistors are turned off on initial poweron.
    set_current(0);

    // To avoid "double start" problems on software updates, delay on start.
    delay(2000);

    // Configure the digital pins driving the shift register as outputs.
    pinMode(SR_LATCH, OUTPUT);
    pinMode(SR_CLOCK, OUTPUT);
    pinMode(SR_DATA, OUTPUT);

    // Initialize the LiteSerial logging library.
    // https://github.com/Syonyk/LiteSerialLogger
    LiteSerial.begin(115200);

    // LCD is a 16x2 unit.
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);

    memset(amps, 0, sizeof (amps));
    memset(time, 0, sizeof (time));

    // Start the voltmeter!
    ads.begin();
}

/**
 * Set the current to some multiple of 0.05A nominal.
 * 
 * 0 - 0A.
 * 1 - 0.05A
 * 60 - 3A.
 * 
 * You get the idea.
 * 
 * This simply checks to see if the remaining amps requested are greater than
 * each pin's value, and if so, sets it.
 * 
 * It does not rotate current around resistors.  This might be nice to do at
 * some point.
 * 
 * Returns true if set properly, false if unable to set.
 */
bool set_current(uint8_t factor) {
    uint8_t output_value = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if (factor >= current_per_pin[i]) {
            output_value |= 1 << i;
            factor -= current_per_pin[i];
        }
    }

    // Prepare the latch to take new data.
    digitalWrite(SR_LATCH, LOW);
    // Shift out the value.
    shiftOut(SR_DATA, SR_CLOCK, MSBFIRST, output_value);
    // Latch the new value and update things.
    digitalWrite(SR_LATCH, HIGH);

    // If any data is left, something went wrong.
    if (factor) {
        return false;
    }

    return true;
}

/*
 * Configure the voltmeter to the proper gain, read the differential voltage,
 * and report back the desired values.
 * 
 * Constants are from the ADS documentation for various gains.  get_usb_volts
 * returns the voltage across the USB port, and get_usb_amps returns the amps
 * across the shunt resistors (divides by the 0.05Ω value before returning).
 * 
 * These are not particularly efficient functions as they use floating point.
 */
float get_usb_volts() {
    ads.setGain(GAIN_TWOTHIRDS);
    int16_t results = ads.readADC_Differential_2_3();
    return (0.0001875F * results);
}

float get_usb_amps() {
    ads.setGain(GAIN_SIXTEEN);
    int16_t results = ads.readADC_Differential_0_1();
    return (0.000007812F * results) / 0.05F;
}


// From DF Robot documentation.  Correct for my shield.

int read_LCD_buttons() {
    uint16_t adc_key_in = analogRead(0); // read the value from the sensor

    if (adc_key_in > 1000) return btnNONE;

    // For V1.1 us this threshold
    if (adc_key_in < 50) return btnRIGHT;
    if (adc_key_in < 150) return btnUP;
    if (adc_key_in < 300) return btnDOWN;
    if (adc_key_in < 450) return btnLEFT;
    if (adc_key_in < 700) return btnSELECT;

    return btnNONE; // when all others fail, return this.
}

/**
 * Update the cursor.  Move the location if it has changed (otherwise leave it
 * to avoid flicker), turn it on or off as requested to indicate the current
 * position on the display.
 */
void update_cursor() {
    if (cursor.update) {
        lcd.setCursor(cursor.col, cursor.row);
        if (cursor.on) {
            lcd.cursor();
        } else {
            lcd.noCursor();
        }
        cursor.update = 0;
    }
}

// Prints the inital menu layout.  Turn off the cursor before starting to avoid
// draw updates.

void print_menu_initial() {
    lcd.noCursor();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Hi: .  A      ms"));
    lcd.setCursor(0, 1);
    lcd.print(F("Lo: .  A      ms"));
}

// Print the amp array on the screen.

void print_amps(const bool is_lo, const uint8_t amps[3]) {
    lcd.noCursor();
    lcd.setCursor(3, is_lo);
    lcd.print(amps[0]);
    lcd.setCursor(5, is_lo);
    lcd.print(amps[1]);
    lcd.print(amps[2]);
    // Reposition the cursor.
    update_cursor();
}

// Print the amp array on the screen.

void print_ms(const bool is_lo, const uint8_t ms[4]) {
    lcd.noCursor();
    lcd.setCursor(10, is_lo);
    lcd.print(ms[0]);
    lcd.print(ms[1]);
    lcd.print(ms[2]);
    lcd.print(ms[3]);
    // Reposition the cursor.
    update_cursor();
}

/**
   Runs the menu.  Format:
   Hi:0.00A  0000ms
   Lo:0.00A  0000ms
 */
void run_menu() {
    // Display the static characters.
    print_menu_initial();

    // Print the initial amps values.
    print_amps(false, amps[0]);
    print_amps(true, amps[1]);

    print_ms(false, time[0]);
    print_ms(true, time[1]);

    // Set things initially to amps-hi.
    cursor.row = 0;
    cursor.col = 3;
    cursor.on = 0;
    cursor.update = 1;

    while (1) {
        // Check for keys and update things.
        switch (read_LCD_buttons()) {
            case btnRIGHT:
                // 1 bit value - just wraps around on overflow.
                cursor.row = cursor.row + 
                        cursor_positions[cursor.col].right_toggle_row;
                // Column is the absolute new value.
                cursor.col = cursor_positions[cursor.col].right_col;
                cursor.update = 1;
                delay(BUTTON_DELAY_MS);
                break;
            case btnLEFT:
                cursor.row = cursor.row + 
                        cursor_positions[cursor.col].left_toggle_row;
                cursor.col = cursor_positions[cursor.col].left_col;
                cursor.update = 1;
                delay(BUTTON_DELAY_MS);
                break;

            case btnUP:
                // Increment the proper value.
                if (cursor_positions[cursor.col].is_amps) {
                    uint8_t pos = cursor_positions[cursor.col].amps_pos;
                    amps[cursor.row][pos] += amps_positions[pos].increment;
                    // If the value has overflowed, reset it to 0.
                    if (amps[cursor.row][pos] >= amps_positions[pos].max) {
                        amps[cursor.row][pos] = 0;
                    }
                    // Update the displayed value.
                    print_amps(cursor.row, amps[cursor.row]);
                } else if (cursor_positions[cursor.col].is_ms) {
                    uint8_t pos = cursor_positions[cursor.col].ms_pos;
                    time[cursor.row][pos]++;
                    if (time[cursor.row][pos] >= 10) {
                        time[cursor.row][pos] = 0;
                    }
                    print_ms(cursor.row, time[cursor.row]);
                }
                delay(BUTTON_DELAY_MS);
                break;
            case btnDOWN:
                if (cursor_positions[cursor.col].is_amps) {
                    uint8_t pos = cursor_positions[cursor.col].amps_pos;
                    // Since these are unsigned, increase to max if 0 before
                    // subtracting the proper increment value.
                    if (amps[cursor.row][pos] == 0) {
                        amps[cursor.row][pos] = amps_positions[pos].max;
                    }
                    amps[cursor.row][pos] -= amps_positions[pos].increment;
                    print_amps(cursor.row, amps[cursor.row]);
                } else if (cursor_positions[cursor.col].is_ms) {
                    uint8_t pos = cursor_positions[cursor.col].ms_pos;
                    if (time[cursor.row][pos] == 0) {
                        time[cursor.row][pos] = 10;
                    }
                    time[cursor.row][pos]--;
                    print_ms(cursor.row, time[cursor.row]);
                }
                delay(BUTTON_DELAY_MS);
                break;
            case btnSELECT:
                // Time to run - go back to the loop and we'll run it.
                delay(BUTTON_DELAY_MS);
                return;
                break;
            case btnNONE:
            default:
                break;
        }

        // The compiler should optimize this - is there any reason to use a
        // power of 2?  No, but it feels right.
        if ((millis() % 2048) > 1024) {
            // Turn it on if it's not on.
            if (!cursor.on) {
                cursor.on = 1;
                cursor.update = 1;
            }
        } else {
            // Turn it off if not off.
            if (cursor.on) {
                cursor.on = 0;
                cursor.update = 1;
            }
        }

        update_cursor();
    }
}

/**
 * Convert from the amp array (3 places) to 0.05A steps which are used to set 
 * the resistors.
 */
uint8_t convert_amp_array_to_steps(const uint8_t amps[3]) {
    uint8_t steps = 0;

    // Ones place - 20 steps per amp.
    steps += 20 * amps[0];

    // Tenths place - 2 steps per tenth.
    steps += 2 * amps[1];

    // Hundredths place - 0 or 5, add 1 if 5.
    steps += (amps[2] / 5);

    return steps;
}

uint16_t convert_time_array(const uint8_t time[4]) {
    uint16_t ms = 0;

    ms += 1000 * time[0];
    ms += 100 * time[1];
    ms += 10 * time[2];
    ms += time[3];

    return ms;
}

// Output the static charracters for the run mode.
void print_run_initial() {
    lcd.noCursor();
    lcd.setCursor(0, 0);
    lcd.print(F("R-Hi: .  A  .  V"));
    lcd.setCursor(0, 1);
    lcd.print(F("R-Lo: .  A  .  V"));
}

void print_currents(const uint8_t row, const float amps, const float volts) {
    lcd.setCursor(5, row);
    lcd.print(amps, 2);
    lcd.setCursor(11, row);
    lcd.print(volts, 2);
}

void run_test() {
    uint8_t steps_hi, steps_lo;
    uint16_t time_hi, time_lo;

    // Storage for amps & volts.
    float amps_hi = 0, amps_lo = 0;
    float volts_hi = 0, volts_lo = 0;

    // Timeout, in millis.
    unsigned long timeout;

    bool exit = false;

    // Step 1: Convert from desired actual amp values to the offset steps.
    steps_hi = convert_amp_array_to_steps(amps[0]);
    steps_lo = convert_amp_array_to_steps(amps[1]);

    // Same for time.
    time_hi = convert_time_array(time[0]);
    time_lo = convert_time_array(time[1]);

    // Clear the screen and print the runtime header.
    lcd.clear();
    print_run_initial();

    /**
     * The voltages and currents are taken while running in the given state
     * (high or low), and are updated at the beginning of the following step.
     * 
     * This allows them to stabilize out so the actual report data is transient
     * free.  The scope will capture the transients.
     */
    while (1) {
        // First: Set high current and calculate the timeout.
        set_current(steps_hi);

        timeout = millis() + time_hi;
        print_currents(1, amps_lo, volts_lo);

        while (millis() < timeout) {
            amps_hi = get_usb_amps();
            volts_hi = get_usb_volts();
            // If an exit is requested, set the flag for that.
            if (read_LCD_buttons() == btnSELECT) {
                exit = true;
            }
        }

        // Great - set the low current.
        set_current(steps_lo);
        timeout = millis() + time_lo;
        print_currents(0, amps_hi, volts_hi);
        while (millis() < timeout) {
            amps_lo = get_usb_amps();
            volts_lo = get_usb_volts();
            if (read_LCD_buttons() == btnSELECT) {
                exit = true;
            }
        }

        if (exit) {
            // Set the current to 0 before exiting to avoid cooking resistors.
            set_current(0);
            return;
        }
    }
}

// The state just toggles between the menu and the test!
void loop() {
    run_menu();
    run_test();
}
