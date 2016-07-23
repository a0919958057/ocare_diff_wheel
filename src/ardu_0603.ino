#include <Arduino.h>

// The modbus serial commiuncation library
#include <Modbus.h>
#include <ModbusSerial.h>

/********************** Modbus Information ************************/

#define MB_SENSOR_ADDRESS	100
#define MB_SLAVER_ID_		6
// Define the Modbus HOLD_REGISTER Mapping
enum StateHoldRegister {
	CHASSIS_MODE,
	LEFT_WHEEL_TORQUE,
	RIGHT_WHEEL_TORQUE,
	TRACKLINE_TORQUE_MODE,
	SENSOR_BW_MODE,
	CMD_CHASSIS_MODE,
	CMD_LEFT_WHEEL_TORQUE,
	CMD_RIGHT_WHEEL_TORQUE,
	CMD_TRACKLINE_TORQUE_MODE,
	CMD_SENSOR_BW_MODE
};

/******Defination of the Chassis Mode
* TRACK_LINE :     Tracking the path and follow the path
* CONTROLLABLE:    The Arduino would use LEFT_WHEEL_TORQUE and RIGHT_WHEEL_TORQUE to effort the wheel
* STOP:            All of the driver would stop until Mode become others.
* **************************/
enum ChassisMode {
	MODE_TRACK_LINE,
	MODE_CONTROLLABLE,
	MODE_STOP
};

/******Defination of the Chassis Mode command
* TRACK_LINE :     Tracking the path and follow the path
* CONTROLLABLE:    The Arduino would use LEFT_WHEEL_TORQUE and RIGHT_WHEEL_TORQUE to effort the wheel
* STOP:            All of the driver would stop until Mode become others.
* **************************/
enum ChassisModeCMD {
	MODE_TRACK_LINE_CMD,
	MODE_CONTROLLABLE_CMD,
	MODE_STOP_CMD
};

/******Defination of the Chassis Mode
* HIGH :           Useing the higher torque Fuzzy rule
* MED:             Useing the normal torque Fuzzy rule
* LOW:             Useing the low torque Fuzzy rule
* **************************/
enum TrackingTorqueMode {
	TORQUE_HIGH,
	TORQUE_MED,
	TORQUE_LOW
};

/******Defination of the Chassis Mode command
* HIGH :           command arduino use the higher torque Fuzzy rule
* MED:             command arduino use the normal torque Fuzzy rule
* LOW:             command arduino use the low torque Fuzzy rule
* **************************/
enum TrackingTorqueModeCMD {
	TORQUE_HIGH_CMD,
	TORQUE_MED_CMD,
	TORQUE_LOW_CMD
};

/******Defination of the Sensor BW Mode
* BLACK :           Tracking the black line
* WRITE:            Tracking the white line
* **************************/
enum SensorBWMode {
	BLACK,
	WHITE
};

/******Defination of the Sensor BW Mode command
* BLACK_CMD :           Command arduino track the black line
* WRITE_CMD:            Command arduino track white line
* **************************/
enum SensorBWModeCMD {
	BLACK_CMD,
	WHITE_CMD,
};

/******************************************************************/

// Modbus object
ModbusSerial mb;

uint16_t input_Chassis_Mode;
int16_t input_Left_Wheel_T;
int16_t input_Right_Wheel_T;
uint16_t input_Trackline_T_Mode;
uint16_t input_BW_mode;

uint16_t output_Chassis_mode;
uint16_t output_Trackline_T_mode;
int16_t output_Left_Wheel_T;
int16_t output_Right_Wheel_T;
uint16_t output_BW_mode;


// Enable debug
//#define SERIAL_DEBUG

// Enable tracking line exception detect
#define TRACKING_EXCEPT_DETECT

// Enable init sensor DEBUG
#define SERIAL_SENSOR_INIT_DEBUG

// Serial Debug tunnle baud
#define SERIAL_BAUD_            115200

/********************** Pin I/O Define *******************

+-----+
+----[PWR]-------------------| USB |--+
|                            +-----+  |
|           GND/RST2  [ ] [ ]         |
|         MOSI2/SCK2  [ ] [ ]  SCL[ ] |   D0mj
|            5V/MISO2 [ ] [ ]  SDA[ ] |   D1
|                             AREF[ ] |
|                              GND[ ] |
| [ ]N/C                    SCK/13[ ]~|   B7
| [ ]v.ref                 MISO/12[ ]~|   B6
| [ ]RST                   MOSI/11[ ]~|   B5
| [ ]3V3      +----------+      10[X]~|   B4    Right Wheel Vref
| [ ]5v       | ARDUINO  |       9[X]~|   H6    Left Wheel Vref
| [ ]GND      |   MEGA   |       8[ ]~|   H5
| [ ]GND      +----------+            |
| [ ]Vin                         7[ ]~|   H4
|                                6[ ]~|   H3
| [ ]A0                          5[ ]~|   E3
| [X]A1                          4[ ]~|   G5
| [X]A2                     INT5/3[ ]~|   E5
| [X]A3                     INT4/2[ ]~|   E4
| [X]A4                       TX>1[ ]~|   E1
| [X]A5                       RX<0[ ]~|   E0
| [X]A6                               |
| [X]A7                     TX3/14[ ] |   J1
|                           RX3/15[ ] |   J0
| [X]A8                     TX2/16[ ] |   H1
| [X]A9                     RX2/17[ ] |   H0
| [X]A10               TX1/INT3/18[ ] |   D3
| [X]A11               RX1/INT2/19[ ] |   D2
| [X]A12           I2C-SDA/INT1/20[ ] |   D1
| [X]A13           I2C-SCL/INT0/21[ ] |   D0
| [ ]A14                              |
| [ ]A15                              |   Ports:
|                RST SCK MISO         |    22=A0  23=A1
|         ICSP   [ ] [ ] [ ]          |    24=A2  25=A3
|                [ ] [ ] [ ]          |    26=A4  27=A5
|                GND MOSI 5V          |    28=A6  29=A7
| G                                   |    30=C7  31=C6
| N 5 5 4 4 4 4 4 3 3 3 3 3 2 2 2 2 5 |    32=C5  33=C4
| D 2 0 8 6 4 2 0 8 6 4 2 0 8 6 4 2 V |    34=C3  35=C2
|         ~ ~                         |    36=C1  37=C0
| @ # # # # # # # # X X X X X X X X @ |    38=D7  39=G2
| @ # # # # # # # # # # # # # # # # @ |    40=G1  41=G0
|           ~                         |    42=L7  43=L6
| G 5 5 4 4 4 4 4 3 3 3 3 3 2 2 2 2 5 |    44=L5  45=L4
| N 3 1 9 7 5 3 1 9 7 5 3 1 9 7 5 3 V |    46=L3  47=L2
| D                                   |    48=L1  49=L0    SPI:
|                                     |    50=B3  51=B2     50=MISO 51=MOSI
|     2560                ____________/    52=B1  53=B0     52=SCK  53=SS
\_______________________/

Digitial Pin:
24: Left Wheel      INPUT1
22: Left Wheel      INPUT2

28: Right Wheel     INPUT1
26: Right Wheel     INPUT2

34: White init      BUTTON
36: Black init      BUTTON

ADC Pin:
A1 - A13 Sensors

LED Pin:
32: Red
30: Green

********************** Pin I/O Define end ********************/

/* define the wheel output pin */
// Left wheel
#define PIN_WHEEL_LEFT_VREF     (9)
#define PIN_WHEEL_LEFT_INPUT_1  (24)
#define PIN_WHEEL_LEFT_INPUT_2  (22)

// Right wheel
#define PIN_WHEEL_RIGHT_VREF    (10)
#define PIN_WHEEL_RIGHT_INPUT_1 (28)
#define PIN_WHEEL_RIGHT_INPUT_2 (26)

/* define the wheel output pin */
#define BUTTON_WHITE_INIT       (34)
#define BUTTON_BLACK_INIT       (36)

/* define the LED output Pin  */
#define LED_RED									(32)
#define LED_GREEN								(30)

/**** define the sensor information ****/
#define SENSOR_START_PIN        (1)
#define SENSOR_COUNT            (13)
#define SENSOR_SAMPLE_COUNT     (1000)
#define SENSOR_WHITE_THRESHOLD  (100)
#define SENSOR_BLACK_THRESHOLD  (100 * SENSOR_COUNT - 100)
#define TIME_RATIO              (0.01)

// Sensor mapping
#define SENSOR_VALUE_MAX 100
#define SENSOR_VALUE_MIN 0

uint16_t sensor_value[SENSOR_COUNT] = { 0 };
uint16_t sensor_max_limit[SENSOR_COUNT] = { 0 };
uint16_t sensor_min_limit[SENSOR_COUNT] = { 0 };
const float SENSOR_WEIGHT[SENSOR_COUNT] =
{ -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };

// Robot sensor status flag
bool is_sensor_min_inited = false;
bool is_sensor_max_inited = false;


/****** define the PID controller information******/
#define ERROR_MAX       			(6.0)
#define ERROR_MIN       			(-6.0)

#define ERROR_RATE_MAX  			(1.0)
#define ERROR_RATE_MIN  			(-1.0)

#define BASE_HIGH_TORQUE			(255)
#define BASE_MED_TORQUE				(211)
#define BASE_LOW_TORQUE      	(100)

#define SLOW_BASE_THRESHOLD		(6)

#define SLOW_RATIO						(1.0)

#define K_P										(120)
#define K_D										(70)

// set the motor status
void motor_cmd(int _left_motor, int _right_motor);

// Init the pin I/O
void init_pin();

// Init the ModbusSerial
void init_modbus();

// compute error using sensor_value
float get_error(bool _is_track_black);

// update the sensor_value
float get_sensor_data();

// Sync the global variable to the ModbusSerial context
void modbus_sync();


#ifdef TRACKING_EXCEPT_DETECT
bool except_detect(bool _is_track_black);
#endif // TRACKING_EXCEPT_DETECT


void doSensorMaxInit();
void doSensorMinInit();

void setup()
{
	// Setup the serial debug tunnel
	Serial.begin(SERIAL_BAUD_);

	/* Setup the pin I/O mode */
	init_pin();

	/* Setup the ModbusSerial */
	init_modbus();

	while (true) {

		if (digitalRead(BUTTON_WHITE_INIT))
			doSensorMaxInit();

		if (digitalRead(BUTTON_BLACK_INIT))
			doSensorMinInit();

		if (is_sensor_max_inited == true) digitalWrite(LED_RED, LOW);
		else digitalWrite(LED_RED,HIGH);

		if (is_sensor_min_inited == true) digitalWrite(LED_GREEN, LOW);
		else digitalWrite(LED_GREEN,HIGH);

		if (is_sensor_max_inited && is_sensor_min_inited) break;
	}


}

float error_last(0);
float error(0);
unsigned long time_stamp = millis();
unsigned long time_stamp_old = millis();


float period_time = 0;

void loop()
{


	// TODO: Change the sync tesk to the Serial3 Interrupt
	// Sync the register with Raspberry Pi 2 ROS
	modbus_sync();

	float error_rate(0);
	float error_modify(0);

	time_stamp_old = time_stamp;
	time_stamp = millis();
	// period_time = float(time_stamp - time_stamp_old) * TIME_RATIO;
	period_time = float(time_stamp - time_stamp_old);

	get_sensor_data();

	/********************* Modbus Control ************************/

  bool is_track_black(false);

	if(input_BW_mode == SensorBWModeCMD::BLACK_CMD) {
		output_BW_mode = SensorBWMode::BLACK;
		is_track_black = true;
		mb.Hreg(StateHoldRegister::SENSOR_BW_MODE, output_BW_mode);
	} else if (input_BW_mode == SensorBWModeCMD::WHITE_CMD) {
		output_BW_mode = SensorBWMode::WHITE;
		is_track_black = false;
		mb.Hreg(StateHoldRegister::SENSOR_BW_MODE, output_BW_mode);
	}

	/*************************************************************/

	// Store the old error value
	error_last = error;
	// Get the new error value via sensor_value
	error = get_error(is_track_black);
	// Compute the error rate
	error_rate = error - error_last;

	// If there is any exception while tracking the line, then use old error
#ifdef TRACKING_EXCEPT_DETECT
	if (except_detect(is_track_black) == true) {
		error = error_last;
		error_rate = 0;
	}
#endif // TRACKING_EXCEPT_DETECT


	// Trim the Maximum and Minimum of error_rate and error
	if (error_rate < ERROR_RATE_MIN)
		error_rate = ERROR_RATE_MIN;
	else if (error_rate > ERROR_RATE_MAX)
		error_rate = ERROR_RATE_MAX;

	if (error < ERROR_MIN)
		error_modify = ERROR_MIN ;
	else if (error > ERROR_MAX)
		error_modify = ERROR_MAX ;
	else
		error_modify = error;

	/********************* Modbus Control ************************/

	int trackline_torque = BASE_MED_TORQUE;

	if (input_Trackline_T_Mode == TrackingTorqueModeCMD::TORQUE_HIGH_CMD) {
		trackline_torque = BASE_HIGH_TORQUE;

		output_Trackline_T_mode = input_Trackline_T_Mode;
		mb.Hreg(StateHoldRegister::TRACKLINE_TORQUE_MODE, output_Trackline_T_mode);
	}
	else if (input_Trackline_T_Mode == TrackingTorqueModeCMD::TORQUE_MED_CMD) {
		trackline_torque = BASE_MED_TORQUE;

		output_Trackline_T_mode = input_Trackline_T_Mode;
		mb.Hreg(StateHoldRegister::TRACKLINE_TORQUE_MODE, output_Trackline_T_mode);
	}
	else if (input_Trackline_T_Mode == TrackingTorqueModeCMD::TORQUE_LOW_CMD) {
		trackline_torque = BASE_LOW_TORQUE;

	output_Trackline_T_mode = input_Trackline_T_Mode;
	mb.Hreg(StateHoldRegister::TRACKLINE_TORQUE_MODE, output_Trackline_T_mode);
  }

	/*************************************************************/



	int output_kp = error_modify * K_P;
	int output_kd = error_rate * K_D;
	//Serial.println(output);
	int speed;
	if(abs(error_modify) < SLOW_BASE_THRESHOLD) {
		speed = trackline_torque;
	} else {
		speed = trackline_torque - (abs(output_kp) - SLOW_BASE_THRESHOLD * K_P) * SLOW_RATIO;
	}


	// NOTE: We don't use D controller,so that we dont put it on torque variable
	int left_torque = speed + (int)output_kp;
	int right_torque = speed - 1 * (int)output_kp;


	/********************* Modbus Control ************************/

	// The diff wheel chassis use CMD STATE to determine which toqure variable will give motor
	if (input_Chassis_Mode == ChassisModeCMD::MODE_TRACK_LINE_CMD) {

		// If there is TRACK_LINE_MODE, then P controller output will be used
		motor_cmd(left_torque, right_torque);

		/**** Sync the CMD to report register ****/
		output_Chassis_mode = ChassisMode::MODE_TRACK_LINE;
		mb.Hreg(StateHoldRegister::CHASSIS_MODE, output_Chassis_mode);

		output_Left_Wheel_T = left_torque;
		output_Right_Wheel_T = right_torque;
		mb.Hreg(StateHoldRegister::LEFT_WHEEL_TORQUE, output_Left_Wheel_T);
		mb.Hreg(StateHoldRegister::RIGHT_WHEEL_TORQUE, output_Right_Wheel_T);
	}

	if (input_Chassis_Mode == ChassisModeCMD::MODE_CONTROLLABLE_CMD) {

		// If there is CONTROLLABLE_MODE, then ROS Torque CMD will be used
		motor_cmd(input_Left_Wheel_T, input_Right_Wheel_T);

		/**** Sync the CMD to report register ****/
		output_Chassis_mode = ChassisMode::MODE_CONTROLLABLE;
		mb.Hreg(StateHoldRegister::CHASSIS_MODE, output_Chassis_mode);

		output_Left_Wheel_T = input_Left_Wheel_T;
		output_Right_Wheel_T = input_Right_Wheel_T;
		mb.Hreg(StateHoldRegister::LEFT_WHEEL_TORQUE, output_Left_Wheel_T);
		mb.Hreg(StateHoldRegister::RIGHT_WHEEL_TORQUE, output_Right_Wheel_T);
	}

	if (input_Chassis_Mode == ChassisModeCMD::MODE_STOP_CMD) {

		// If there is STOP_MODE, then The Wheel will stop
		motor_cmd(0, 0);

		/**** Sync the CMD to report register ****/
		output_Left_Wheel_T = 0;
		output_Right_Wheel_T = 0;
		mb.Hreg(StateHoldRegister::LEFT_WHEEL_TORQUE, output_Left_Wheel_T);
		mb.Hreg(StateHoldRegister::RIGHT_WHEEL_TORQUE, output_Right_Wheel_T);

		output_Chassis_mode = ChassisMode::MODE_STOP;
		mb.Hreg(StateHoldRegister::CHASSIS_MODE, output_Chassis_mode);
	}

	/*************************************************************/

#ifdef SERIAL_SENSOR_DEBUG
  for (int i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(sensor_value[i]); //debug
      Serial.print('\t');
  }
	Serial.println('\t');
#endif // SERIAL_SENSOR_DEBUG

#ifdef SERIAL_DEBUG


	Serial.print(period_time);
	Serial.print('\t');
	Serial.print(error_modify);
	Serial.print('\t');
	Serial.print(error_rate);
	Serial.print('\t');
	Serial.print(output_kp);
	Serial.print('\t');
	Serial.print(speed - (int)output_kp);
	Serial.print('\t');
	Serial.print(speed + (int)output_kp);
	Serial.print('\t');

	Serial.println('\t');
#endif // SERIAL_DEBUG

}

void init_modbus() {
	// Config Mosbus Serial
	mb.config(&Serial3, 115200, SERIAL_8N2);

	// Config slaves ID
	mb.setSlaveId(MB_SLAVER_ID_);

	// Add the mapping register to specifid Register

	mb.addHreg(StateHoldRegister::CHASSIS_MODE);
	mb.addHreg(StateHoldRegister::LEFT_WHEEL_TORQUE);
	mb.addHreg(StateHoldRegister::RIGHT_WHEEL_TORQUE);
	mb.addHreg(StateHoldRegister::TRACKLINE_TORQUE_MODE);
	mb.addHreg(StateHoldRegister::SENSOR_BW_MODE);
	mb.addHreg(StateHoldRegister::CMD_CHASSIS_MODE);
	mb.addHreg(StateHoldRegister::CMD_LEFT_WHEEL_TORQUE);
	mb.addHreg(StateHoldRegister::CMD_RIGHT_WHEEL_TORQUE);
	mb.addHreg(StateHoldRegister::CMD_TRACKLINE_TORQUE_MODE);
	mb.addHreg(StateHoldRegister::CMD_SENSOR_BW_MODE);

	for (int i = 0; i < SENSOR_COUNT; i++) {
		mb.addHreg(MB_SENSOR_ADDRESS + i);
	}

	// initial for all ModbusSerial mb to be zero

	mb.Hreg(StateHoldRegister::CHASSIS_MODE, ChassisMode::MODE_STOP);
	mb.Hreg(StateHoldRegister::LEFT_WHEEL_TORQUE, 0);
	mb.Hreg(StateHoldRegister::RIGHT_WHEEL_TORQUE, 0);
	mb.Hreg(StateHoldRegister::TRACKLINE_TORQUE_MODE, TrackingTorqueMode::TORQUE_MED);
	mb.Hreg(StateHoldRegister::SENSOR_BW_MODE, SensorBWMode::WHITE);
	mb.Hreg(StateHoldRegister::CMD_CHASSIS_MODE, ChassisModeCMD::MODE_STOP_CMD);
	mb.Hreg(StateHoldRegister::CMD_LEFT_WHEEL_TORQUE, 0);
	mb.Hreg(StateHoldRegister::CMD_RIGHT_WHEEL_TORQUE, 0);
	mb.Hreg(StateHoldRegister::CMD_TRACKLINE_TORQUE_MODE, TrackingTorqueModeCMD::TORQUE_MED_CMD);
	mb.Hreg(StateHoldRegister::CMD_SENSOR_BW_MODE, SensorBWModeCMD::WHITE_CMD);

	// Initial all ModbusSerial SENSOR_REGISTER

	for (int i = 0; i < SENSOR_COUNT; i++) {
		mb.Hreg(MB_SENSOR_ADDRESS + i, 0);
	}
}

void init_pin() {

	/* MOTOR LEFT CONTROL PIN */
	pinMode(PIN_WHEEL_LEFT_VREF, OUTPUT);
	pinMode(PIN_WHEEL_LEFT_INPUT_1, OUTPUT);
	pinMode(PIN_WHEEL_LEFT_INPUT_2, OUTPUT);

	analogWrite(PIN_WHEEL_LEFT_VREF, 0);
	digitalWrite(PIN_WHEEL_LEFT_INPUT_1, HIGH);
	digitalWrite(PIN_WHEEL_LEFT_INPUT_2, HIGH);

	/* MOTOR RIGHT CONTROL PIN */
	pinMode(PIN_WHEEL_RIGHT_VREF, OUTPUT);
	pinMode(PIN_WHEEL_RIGHT_INPUT_1, OUTPUT);
	pinMode(PIN_WHEEL_RIGHT_INPUT_1, OUTPUT);

	analogWrite(PIN_WHEEL_RIGHT_VREF, 0);
	digitalWrite(PIN_WHEEL_RIGHT_INPUT_1, HIGH);
	digitalWrite(PIN_WHEEL_RIGHT_INPUT_2, HIGH);

	/* INIT BUTTON PIN */
	pinMode(BUTTON_WHITE_INIT, INPUT);
	pinMode(BUTTON_BLACK_INIT, INPUT);


	/* STAT SIGNAL LED PIN */
	pinMode(LED_RED, OUTPUT);
	pinMode(LED_GREEN, OUTPUT);


}

float get_error(bool _is_track_black) {
	float sum(0);
	float weight_sum(0);
	if(_is_track_black) {
		for (int i = 0, error = 0; i < SENSOR_COUNT; i++) {
			weight_sum += (SENSOR_WEIGHT[i]) * (100 - sensor_value[i]);
					// NOTE: the sensor read black will be 0, and white will be 100 so need inverse
			sum += 100 - sensor_value[i];
		}
		return weight_sum / sum;
  }
	else {
		for (int i = 0, error = 0; i < SENSOR_COUNT; i++) {
			weight_sum += (SENSOR_WEIGHT[i]) * (sensor_value[i]);
					// NOTE: the sensor read black will be 0, and white will be 100 so need inverse
			sum += sensor_value[i];
		}
		return weight_sum / sum;
	}

}

float get_sensor_data() {
	for (int i = 0; i < SENSOR_COUNT; i++) {
		int sensor_read = analogRead(i + SENSOR_START_PIN);

		// Term To max and min
		if (sensor_max_limit[i] < sensor_read)
			sensor_read = sensor_max_limit[i];
		else if (sensor_min_limit[i] > sensor_read)
			sensor_read = sensor_min_limit[i];

		sensor_read = map(sensor_read, sensor_min_limit[i], sensor_max_limit[i], SENSOR_VALUE_MIN, SENSOR_VALUE_MAX);

		sensor_value[i] = sensor_read;
	}
}

#ifdef TRACKING_EXCEPT_DETECT
bool except_detect(bool _is_track_black) {
	int sensor_sum;
	sensor_sum = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		sensor_sum += sensor_value[i];
	}

	if(_is_track_black)
		return (sensor_sum > SENSOR_BLACK_THRESHOLD);
	else
		return (sensor_sum < SENSOR_WHITE_THRESHOLD);
}
#endif // TRACKING_EXCEPT_DETECT


void motor_cmd(int _left_motor, int _right_motor) {
	if (_left_motor >= 0) {
		digitalWrite(PIN_WHEEL_LEFT_INPUT_1, LOW);
		digitalWrite(PIN_WHEEL_LEFT_INPUT_2, HIGH);

		if (_left_motor > 255)
			_left_motor = 255;

	}
	else {
		digitalWrite(PIN_WHEEL_LEFT_INPUT_1, HIGH);
		digitalWrite(PIN_WHEEL_LEFT_INPUT_2, LOW);

		if (abs(_left_motor) > 255)
			_left_motor = 255;
		else
			_left_motor = abs(_left_motor);

	}

	if (_right_motor >= 0) {
		digitalWrite(PIN_WHEEL_RIGHT_INPUT_1, LOW);
		digitalWrite(PIN_WHEEL_RIGHT_INPUT_2, HIGH);

		if (_right_motor > 255)
			_right_motor = 255;

	}
	else {
		digitalWrite(PIN_WHEEL_RIGHT_INPUT_1, HIGH);
		digitalWrite(PIN_WHEEL_RIGHT_INPUT_2, LOW);

		if (abs(_right_motor) > 255)
			_right_motor = 255;
		else
			_right_motor = abs(_right_motor);

	}

	analogWrite(PIN_WHEEL_LEFT_VREF, _left_motor);
	analogWrite(PIN_WHEEL_RIGHT_VREF, _right_motor);
}

void doSensorMaxInit() {

	long int sensor_sum[SENSOR_COUNT] = { 0 };

	for (int i = 0; i < SENSOR_SAMPLE_COUNT; i++) {
		for (int sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {
			sensor_sum[sensor_index] += analogRead(sensor_index + 1);
		}
	}

#ifdef SERIAL_SENSOR_INIT_DEBUG
	Serial.print("Sensor MAX= ");
#endif // SERIAL_SENSOR_INIT_DEBUG

	for (int i = 0; i < SENSOR_COUNT; i++) {
		sensor_max_limit[i] = sensor_sum[i] / SENSOR_SAMPLE_COUNT;

#ifdef SERIAL_SENSOR_INIT_DEBUG
		char buffer[20];
		sprintf(buffer, "%2d :%4d, ", i, sensor_max_limit[i]);
		Serial.write(buffer);
#endif // SERIAL_SENSOR_INIT_DEBUG

	}

#ifdef SERIAL_SENSOR_INIT_DEBUG
	Serial.println("\n");
#endif // SERIAL_SENSOR_INIT_DEBUG

	is_sensor_max_inited = true;
}

void doSensorMinInit() {

	long int sensor_sum[SENSOR_COUNT] = { 0 };

	for (int i = 0; i < SENSOR_SAMPLE_COUNT; i++) {
		for (int sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {
			sensor_sum[sensor_index] += analogRead(sensor_index + 1);
		}
	}

#ifdef SERIAL_SENSOR_INIT_DEBUG
	Serial.print("Sensor MIN= ");
#endif // SERIAL_SENSOR_INIT_DEBUG

	for (int i = 0; i < SENSOR_COUNT; i++) {
		sensor_min_limit[i] = sensor_sum[i] / SENSOR_SAMPLE_COUNT;

#ifdef SERIAL_SENSOR_INIT_DEBUG
		char buffer[20];
		sprintf(buffer, "%2d :%4d, ", i, sensor_min_limit[i]);
		Serial.write(buffer);
#endif // SERIAL_SENSOR_INIT_DEBUG

	}

#ifdef SERIAL_SENSOR_INIT_DEBUG
	Serial.println("\n");
#endif // SERIAL_SENSOR_INIT_DEBUG

	is_sensor_min_inited = true;
}

void modbus_sync() {
	// sync all state immediately
	mb.task();

	// Read all ModbusSerial mb context to global variable
	input_Chassis_Mode = mb.Hreg(StateHoldRegister::CMD_CHASSIS_MODE);
	input_Left_Wheel_T = mb.Hreg(StateHoldRegister::CMD_LEFT_WHEEL_TORQUE);
	input_Right_Wheel_T = mb.Hreg(StateHoldRegister::CMD_RIGHT_WHEEL_TORQUE);
	input_Trackline_T_Mode = mb.Hreg(StateHoldRegister::CMD_TRACKLINE_TORQUE_MODE);
	input_BW_mode = mb.Hreg(StateHoldRegister::CMD_SENSOR_BW_MODE);
	// Write all global variable to ModbusSerial mb context
	output_Left_Wheel_T = mb.Hreg(StateHoldRegister::LEFT_WHEEL_TORQUE);
	output_Right_Wheel_T = mb.Hreg(StateHoldRegister::RIGHT_WHEEL_TORQUE);
	output_Chassis_mode = mb.Hreg(StateHoldRegister::CHASSIS_MODE);
	output_Trackline_T_mode = mb.Hreg(StateHoldRegister::TRACKLINE_TORQUE_MODE);
	output_BW_mode = mb.Hreg(StateHoldRegister::SENSOR_BW_MODE);

	// Write All sensor data to the ModbusSerial mb context
	for (int i = 0; i < SENSOR_COUNT; i++) {
		mb.Hreg(MB_SENSOR_ADDRESS + i, sensor_value[i]);
	}
}
