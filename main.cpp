#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

// Bluetooth Settings
BluetoothSerial BT;
const String device_name = "Spider Drone";
char Rx_Buffer[32];
volatile bool update = false;
volatile bool bt_started = false;
volatile bool bt_connected = false;
volatile bool bt_received = false;
volatile bool bt_congestion = false;
bool first_connection = true;
volatile bool ack_pending = false;

// Servo Control
int pinServos[] = {22, 23, 25, 26, 27, 32, 33, 4, 13, 18, 19, 21};
Servo hip_servos[4];
Servo elbow_servos[4];
Servo foot_servos[4];
const int hip_home[4] = {45, 135, 45, 135};
const int elbow_home[4] = {90, 90, 90, 90};
const int foot_home[4] = {90, 90, 90, 90};
const int x_count = 60;
const int y_count = 60;
int phase = 0;
int gait_phase = 0;  // 0 or 1
int step_size = 18;

// Joystick Variables
int hip_temp[4], elbow_temp[4], foot_temp[4];
int leg_select;
int x_axis, y_axis, z_axis;
int set_move = 0;

void initialize();
void shutdown();
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void bluetooth_processing();
void move_leg(int leg, int hip_delta, int lift_delta);
void motion_processing();
void hip_control(int hip_axis);
void elbow_control(int elbow_axis);
void end_control(int foot_axis);
void hip_manual(int hip_axis);
void elbow_manual(int elbow_axis);
void end_manual(int foot_axis);

/**
 * @brief Initializes the robot to a default position
 * 
 * @return None
 */
void initialize(){

    elbow_servos[1].write(45);
    elbow_servos[3].write(45);
    delay(200);

    // Ready position for feet
    foot_servos[1].write(180);
    foot_servos[3].write(180);
    delay(200);

    // Ready position for hips
    hip_servos[1].write(135);
    hip_servos[3].write(135);
    hip_temp[1] = 135;
    hip_temp[3] = 135;
    delay(200);

    // Ready position for feet
    foot_servos[1].write(155);
    foot_servos[3].write(155);
    delay(200);

    foot_servos[1].write(90);
    foot_servos[3].write(90);
    foot_temp[1] = 90;
    foot_temp[3] = 90;
    delay(200);

    // Ready position for elbows
    elbow_servos[1].write(90);
    elbow_servos[3].write(90);
    elbow_temp[1] = 90;
    elbow_temp[3] = 90;
    delay(700);

    elbow_servos[2].write(135);
    elbow_servos[0].write(135);
    delay(200);

    // Ready position for feet
    foot_servos[0].write(0);
    foot_servos[2].write(0);
    delay(200);

    // Ready position for hips
    hip_servos[0].write(45);
    hip_servos[2].write(45);
    hip_temp[0] = 45;
    hip_temp[2] = 45;
    delay(200);

    // Ready position for feet
    foot_servos[0].write(25);
    foot_servos[2].write(25);
    delay(200);

    foot_servos[0].write(90);
    foot_servos[2].write(90);
    foot_temp[0] = 90;
    foot_temp[2] = 90;
    delay(200);

    // Ready position for elbows
    elbow_servos[0].write(90);
    elbow_servos[2].write(90);
    elbow_temp[0] = 90;
    elbow_temp[2] = 90;
    delay(700);
}

/**
 * @brief Shuts down the robot by moving it to a position that is easy to pick up
 * 
 * @return None
 */
void shutdown(){

  // Power off position for hips
    hip_servos[1].write(90);
    hip_servos[3].write(90);
    delay(200);

  // Power off position for elbows
    elbow_servos[1].write(0);
    elbow_servos[3].write(0);
    delay(200);
    
    foot_servos[1].write(0);
    foot_servos[3].write(0);
    delay(500);

    // Power off position for hips
    hip_servos[0].write(90);
    hip_servos[2].write(90);
    delay(200);

  // Power off position for elbows
    elbow_servos[0].write(180);
    elbow_servos[2].write(180);
    delay(200);
    
    foot_servos[0].write(180);
    foot_servos[2].write(180);
    delay(500);
}

/**
* @brief Handles bluetooth errors and received messages
*
* @param event Triggered event
* @param param 
* @return None
*/
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if (event == ESP_SPP_INIT_EVT)
    bt_started = true;
  else if (event == ESP_SPP_UNINIT_EVT)
    bt_started = false;
  else if (event == ESP_SPP_OPEN_EVT)
    bt_connected = true;
  else if (event == ESP_SPP_CLOSE_EVT)
    bt_connected = false;
  else if (event == ESP_SPP_DATA_IND_EVT ) {
    bt_received = true;
    update = true;
  }
  else if (event == ESP_SPP_CONG_EVT){
    bt_congestion = true;
  }
}

/**
* @brief Processes data from bluetooth and turns them into real ints
*
* @return None
*/
void bluetooth_processing() {
  x_axis = 0;
  y_axis = 0;
  z_axis = 0;

  char *xPtr = strchr(Rx_Buffer, 'X');
  char *yPtr = strchr(Rx_Buffer, 'Y');
  char *zPtr = strchr(Rx_Buffer, 'Z');

  if (!xPtr || !yPtr) return;
  
  leg_select = *(xPtr - 1) - '0';  
  if (leg_select >= 0 && leg_select <= 3 && zPtr){
    x_axis = atoi(xPtr + 1) * (-2) + 20;
    y_axis = atoi(yPtr + 1) * 2 - 20;
    z_axis = atoi(zPtr + 1) * 2 - 20;
    if (x_axis <= 4 && x_axis >= -4)
      x_axis = 0;
    if (y_axis <= 4 && y_axis >= -4 )
      y_axis = 0;
    if (z_axis <= 4 && z_axis >= -4)
      z_axis = 0;
  } 
  else if (leg_select == 4 && !zPtr){
    x_axis = atoi(xPtr + 1) * (-2) + 100;
    y_axis = atoi(yPtr + 1) * 2 - 100;
    if (x_axis <= 14 && x_axis >= -3)
      x_axis = 0;
    if (y_axis <= 3 && y_axis >= -18 )
      y_axis = 0;
    z_axis = 0;
  }
}

/**
 * @brief Moves the specified leg by the specified amounts for the hip and lift
 * 
 * @param leg Leg to move (0-3)
 * @param hip_delta Amount to move the hip joint (positive for forward, negative for backward
 * @param lift_delta Amount to move the lift joint (positive for lift, negative for lower)
 * @return None
 */
void move_leg(int leg, int hip_delta, int lift_delta)
{
  leg_select = leg;

  // lift / lower
  elbow_control(lift_delta);

  // move forward/back
  hip_control(hip_delta);

  // stabilize foot
  end_control(0);
}

/**
 * @brief Lifts the specified leg
 * 
 * @param leg Leg to lift (0-3)
 * @return None
 */
void lift_leg(int leg)
{
    leg_select = leg;
    elbow_control(35);   // higher lift (main fix)
    end_control(-12);    // slightly more clearance
}

/**
 * @brief Lowers the specified leg
 * 
 * @param leg Leg to lower (0-3)
 * @return None
 */
void lower_leg(int leg)
{
    leg_select = leg;
    // hard return to ground stance
    elbow_servos[leg].write(90);
    elbow_temp[leg] = 90;
    foot_servos[leg].write(90);
    foot_temp[leg] = 90;
}

/**
* @brief Processes the x, y and z axes to actual movement
*
* @return None
*/
void motion_processing()
{
  // MANUAL MODE
  if (leg_select >= 0 && leg_select <= 3){
    int x = x_axis / 5;
    int y = y_axis / 5;
    int z = z_axis / 5;

    hip_manual(x);
    elbow_manual(z);
    end_manual(y);
    return;
  }
  // WALK MODE
  int forward = y_axis / 20;
  int turn = x_axis / 30;

  if (forward == 0 && turn == 0)
    return;

  static unsigned long lastStep = 0;
  if (millis() - lastStep < 900)
    return;
  lastStep = millis();

  int s = step_size;
  int side[4] = {0, 1, 0, 1}; // 0 = left legs (0,2), 1 = right legs (1,3)
  int forward_cmd = forward * (s * 0.7);
  int turn_cmd    = turn * (s * 0.6);
  switch (gait_phase){
    // Lift front-left and rear-left
    case 0:
      lift_leg(0);
      lift_leg(2);
      gait_phase = 1;
      break;

    // Move left legs forward, right legs backward
    case 1:
      leg_select = 0;
      hip_control((forward_cmd - turn_cmd) * 0.6);
      leg_select = 2;
      hip_control((forward_cmd - turn_cmd) * 0.6);
      leg_select = 1;
      hip_control((-forward_cmd - turn_cmd) * 0.6);
      leg_select = 3;
      hip_control((-forward_cmd - turn_cmd) * 0.6);
      gait_phase = 2;
      break;
    case 2:
      lower_leg(0);
      lower_leg(2);
      gait_phase = 3;
      break;
    // Lift right pair
    case 3:
      lift_leg(1);
      lift_leg(3);
      gait_phase = 4;
      break;
    // Move right pair forward, left pair backward
    case 4:
      leg_select = 1;
      hip_control((forward_cmd + turn_cmd) * 0.6);
      leg_select = 3;
      hip_control((forward_cmd + turn_cmd) * 0.6);
      leg_select = 0;
      hip_control((-forward_cmd + turn_cmd) * 0.6);
      leg_select = 2;
      hip_control((-forward_cmd + turn_cmd) * 0.6);
      gait_phase = 5;
      break;
    case 5:
      lower_leg(1);
      lower_leg(3);
      gait_phase = 0;
      break;
  }
}

/**
 * @brief Controls the movement for the hip joint
 * 
 * @param hip_axis Amount to move the hip joint (positive for forward, negative for backward)
 * @return None
 */
void hip_manual(int hip_axis)
{
    if (leg_select % 2 == 1)
      hip_axis = -hip_axis;
    hip_temp[leg_select] += hip_axis;
    hip_temp[leg_select] = constrain(hip_temp[leg_select], 0, 180);
    hip_servos[leg_select].write(hip_temp[leg_select]);
}

/**
 * @brief Controls the movement for the elbow joint
 * 
 * @param elbow_axis Amount to move the elbow joint (positive for forward, negative for backward)
 * @return None
 */
void elbow_manual(int elbow_axis)
{
    if (leg_select % 2 == 1)
      elbow_axis = -elbow_axis;
    elbow_temp[leg_select] += elbow_axis;
    elbow_temp[leg_select] = constrain(elbow_temp[leg_select], 0, 180);
    elbow_servos[leg_select].write(elbow_temp[leg_select]);
}

/**
 * @brief Controls the movement for the end joint
 * 
 * @param foot_axis Amount to move the end joint (positive for forward, negative for backward)
 * @return None
 */
void end_manual(int foot_axis)
{
    if (leg_select % 2 == 1)
      foot_axis = -foot_axis;
    foot_temp[leg_select] += foot_axis;
    foot_temp[leg_select] = constrain(foot_temp[leg_select], 0, 180);
    foot_servos[leg_select].write(foot_temp[leg_select]);
}

/**
 * @brief Controls the movement for the hip joint
 * 
 * @param hip_axis Amount to move the hip joint (positive for forward, negative for backward)
 * @return None
 */
void hip_control(int hip_axis)
{
    int dir[4] = {-1, 1, -1, 1};

    hip_axis = constrain(hip_axis, -120, 120);

    int target =
    hip_home[leg_select] +
    (hip_axis * dir[leg_select]);

    hip_temp[leg_select] = target;
    hip_servos[leg_select].write(target);
}

/**
* @brief Controls the movement for the elbow joint
*
* @param elbow_axis Amount to move the elbow joint (positive for forward, negative for backward)
* @return None
*/
void elbow_control(int elbow_axis)
{
    int dir[4] = {1, -1, 1, -1};

    int target =
        elbow_home[leg_select] +
        (elbow_axis * dir[leg_select]);

    target = constrain(target, 20, 160);

    elbow_temp[leg_select] = target;
    elbow_servos[leg_select].write(target);
}

/**
* @brief Controls the movement for the end joint
*
* @param foot_axis Amount to move the end joint (positive for forward, negative for backward)
* @return None
*/
void end_control(int foot_axis)
{
    int dir[4] = {1, -1, 1, -1};

    // MUCH smaller influence range (critical fix)

    int target =
    foot_home[leg_select] +
    (foot_axis * dir[leg_select]);

    // tighter clamp prevents leg collapsing inward
    target = constrain(target, 80, 100);

    foot_temp[leg_select] = target;
    foot_servos[leg_select].write(target);
}

/**
* @brief Initializes and setups up important variable, pins and functions
*
* @return None
*/
void setup() {
  Serial.begin(115200);
  BT.begin(device_name);
  BT.register_callback(BT_EventHandler);
  for (int i = 0, j = 0; i < 4; i++, j+=3) {
    hip_servos[i].attach(pinServos[j]);
    elbow_servos[i].attach(pinServos[j + 1]);
    foot_servos[i].attach(pinServos[j + 2]);
  } 
}

void loop() {
  if(bt_congestion){
    Serial.println("Buffer is full, clearing Buffer.");
    while(BT.available())
      BT.read();
    bt_congestion = false;
  }
  if(bt_received){
    static String incoming = "";

    while (BT.available()) {
      char c = BT.read();

      if (c == '\n') {

        incoming.trim();  // removes hidden \r or spaces

        if (incoming.length() > 0) {
          incoming.toCharArray(Rx_Buffer, sizeof(Rx_Buffer));

          Rx_Buffer[sizeof(Rx_Buffer) - 1] = '\0'; // safety null terminate

          update = true;
        }

        incoming = "";  // reset buffer after full packet
      }
      else {
        incoming += c;
        // safety: prevent memory blow-up if no newline ever comes
        if (incoming.length() > 50) {
          incoming = "";
        }
      }
    }
  }
  if(BT.hasClient() && first_connection){
    Serial.println("Connected to controller");
    first_connection = false;
    delay(500);
    initialize();
  }
  else if (!BT.hasClient() && first_connection){
    Serial.println("Waiting for controller...");
    delay(500);
  }
  else if (!BT.hasClient() && !first_connection){
    Serial.println("Disconnected from controller");
    shutdown();
    first_connection = true;
    delay(500);
  }
  else if(update && !first_connection && BT.hasClient()){
    update = false;

    bluetooth_processing();
    motion_processing();

    BT.println("ACK");
    Serial.println("Leg Select: " + String(leg_select));
    Serial.print("Hip: ");
    Serial.println(hip_temp[leg_select]);
    Serial.print("Elbow: ");
    Serial.println(elbow_temp[leg_select]);
    Serial.print("Foot: ");
    Serial.println(foot_temp[leg_select]);
  }
}