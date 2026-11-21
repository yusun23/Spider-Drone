#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "esp_adc_cal.h"

// Defines and establishes bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

// Minimum debounce time
const unsigned long debounce_time = 75;

int count = 0;
// Joystick Variables
volatile int x_pin = 0, y_pin = 0, z_pin = 0;
volatile bool button[3] = {false, false, false}, button_debounce[3] = {false, false, false}, analog_int[3] = {false, false, false};
int temp_x_pin = 0, temp_y_pin = 0, temp_z_pin;
hw_timer_t *Timer0 = NULL;
unsigned long latest_time[3] = {0};
volatile unsigned long currentTime[3] = {0};

// Bluetooth Variables
BluetoothSerial BTTX;
const String device_name = "Spider Remote";
const String slave_name = "Spider Drone";
uint8_t address[] = {0xB0, 0xCB, 0xD8, 0x8A, 0x3A, 0x42};
char send[11];
volatile bool update = true;
volatile bool bt_sent = true;
volatile bool bt_connected = false;
volatile bool waiting_for_ack = false;
String ack_buffer = "";

// Moving Average Variables
// const int MA_LEN = 12;
// int MA_index = 0;
// int MA_values[3][12];

/**
 * @brief Handles the Interrupts of Bluetooth to send messages or signal error
 *
 * @param event triggered event
 * @param param
 * @return None
 */
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {

  switch (event) {

    case ESP_SPP_INIT_EVT:
      bt_connected = false;
      break;

    case ESP_SPP_OPEN_EVT:
      bt_connected = true;
      waiting_for_ack = false;
      break;

    case ESP_SPP_CLOSE_EVT:
      bt_connected = false;
      waiting_for_ack = false;
      break;

    case ESP_SPP_WRITE_EVT:
      bt_sent = true;   // safe "send complete" signal
      break;

    default:
      break;
  }
}

/**
 * @brief Handles the changing of buttons, built-in ISR
 *
 * @return None
 */
void IRAM_ATTR Button_18_Interrupt(){
  currentTime[0] = micros();
  button_debounce[0] = true;
}

/**
 * @brief Handles the changing of buttons, built-in ISR
 *
 * @return None
 */
void IRAM_ATTR Button_19_Interrupt(){
  currentTime[1] = micros();
  button_debounce[1] = true;
}

/**
 * @brief Handles the changing of buttons, built-in ISR
 *
 * @return None
 */
void IRAM_ATTR Button_21_Interrupt(){
  currentTime[2] = micros();
  button_debounce[2] = true;
}

/**
 * @brief Timer triggered interrupts that read the joystick values, built-in ISR
 *
 * @return None
 */
void IRAM_ATTR Timer0_ISR(){
  analog_int[0] = true;
  analog_int[1] = true;
  analog_int[2] = true;
}

/**
 * @brief Clears the button debounce and makes sure this is the actual press
 *
 * @param button_number Button that was pressed
 * @return None
 */
void debounce(int button_number){

  currentTime[button_number] = millis();

  // Debounce check
  if (currentTime[button_number] - latest_time[button_number] > debounce_time) {

    noInterrupts();
    button[button_number] = !button[button_number];
    interrupts();

    latest_time[button_number] = currentTime[button_number];
    update = true;
  }
}

/**
 * @brief Processes the bluetooth data before sending
 *
 * @return None
 */
void transmit_processing(){
  // Processes information for when a button is pressed
  if (button[0] || button[1] || button[2]){

    int single_x = (int)(x_pin/5.0);
    int single_y = (int)(y_pin/5.0);
    int single_z = (int)(z_pin/5.0);

    if (button[0] && button[1] && !button[2])
      send[0] = '3';
    else if (button[0] && !button[1] && !button[2])
      send[0] = '0';
    else if (button[1] && !button[0] && !button[2])
      send[0] = '1';
    else if (button[2] && !button[0] && !button[1])
      send[0] = '2';

    send[1] = 'X';
    send[2] = '0' + (single_x/10);
    send[3] = '0' + (single_x % 10);
    send[4] = 'Y';
    send[5] = '0' + (single_y/10);
    send[6] = '0' + (single_y % 10);
    send[7] = 'Z';
    send[8] = '0' + (single_z/10);
    send[9] = '0' + (single_z % 10);
    send[10] = '\0';
  }
  // Processes information for when a button isn't pressed
  else{
    send[0] = '4';
    send[1] = 'X';
    send[2] = '0' + (x_pin/100);
    send[3] = '0' + ((x_pin % 100)/10);
    send[4] = '0' + (x_pin % 10);
    send[5] = 'Y';
    send[6] = '0' + (y_pin/100);
    send[7] = '0' + ((y_pin % 100)/10);
    send[8] = '0' + (y_pin % 10);
    send[9] = '\0';
  }
}

int adc_processor(int raw){
  return (int)(raw/40.96 + 0.5);
}

/**
 * @brief Instantiates variables and setups up functions for the application
 *
 * @return None
 */
void setup() {
  // Button Initialization
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);

  // Button Interrupt
  attachInterrupt(digitalPinToInterrupt(18), Button_18_Interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(19), Button_19_Interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(21), Button_21_Interrupt, FALLING);

  // ADC Initialization
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);

  // Timer Interrupt
  Timer0 = timerBegin(0, 80, true);                 // Timer hardware to be used, divider, count up (true) or down (false)
  timerAttachInterrupt(Timer0, &Timer0_ISR, true);  // Hardware Timer, Interupt function address, edge (true for rising, false for falling)
  timerAlarmWrite(Timer0, 50000, true);             // Hardware Timer, ticks before interupt, autoreload (true or false)
  timerAlarmEnable(Timer0);                         // Enable the timer

  Serial.begin(115200);

  // Bluetooth Initialization
  BTTX.begin(device_name, true);
  BTTX.register_callback(BT_EventHandler);

  delay(2000);

  Serial.println("Connecting...");

  while (!BTTX.connect(address)) {
    delay(200);
  }

  Serial.println("Connected!");
}

/**
 * @brief Continuous loop that signals for bluetooth sending
 *
 * @return None
 */
void loop() {
  while (BTTX.available()) {
    char c = BTTX.read();
    if (c == '\n') {
      ack_buffer.trim();
      if (ack_buffer == "ACK") {
        waiting_for_ack = false;
      }
      ack_buffer = "";
    }
    else {
      ack_buffer += c;
    }
  }
  if (analog_int[0]){
    temp_x_pin = analogRead(32);
    analog_int[0] = false;
  }
  if (analog_int[1]){
    temp_y_pin = analogRead(26);
    analog_int[1] = false;
  }
  if (analog_int[2]){
    temp_z_pin = analogRead(4);
    analog_int[2] = false;
  }

  if (temp_x_pin != x_pin){
    x_pin = adc_processor(temp_x_pin);
    update = true;
  }
  if (temp_y_pin != y_pin){
    y_pin = adc_processor(temp_y_pin);
    update = true;
  }
  if (temp_z_pin != z_pin){
    z_pin = adc_processor(temp_z_pin);
    update = true;
  }
  if (count++ >= 1000){
    count = 0;
    Serial.println("X: " + String(x_pin) + " Y: " + String(y_pin) + " Z: " + String(z_pin));
  }
  if (button_debounce[0]){
    debounce(0);
    button_debounce[0] = false;
    update = true;
  }
  if (button_debounce[1]){
    debounce(1);
    button_debounce[1] = false;
    update = true;
  }
  if (button_debounce[2]){
    debounce(2);
    button_debounce[2] = false;
    update = true;
  }
  if (update &&
    bt_connected &&
    bt_sent &&
    !waiting_for_ack){
      transmit_processing();
      update = false;

      bt_sent = false;
      waiting_for_ack = true;

      BTTX.print(send);
      BTTX.print('\n');
  }
}