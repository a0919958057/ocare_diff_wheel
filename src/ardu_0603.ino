#include <Arduino.h>

// Enable debug
#define SERIAL_DEBUG

// Enable tracking line exception detect
//#define TRACKING_EXCEPT_DETECT

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
| [ ]3V3      +----------+      10[X]~|   B4    Left Wheel Vref
| [ ]5v       | ARDUINO  |       9[X]~|   H6    Right Wheel Vref
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
| @ X X # # # # # # # # # # # # X X @ |    38=D7  39=G2
| @ X X # # # # # # # # # # # # X X @ |    40=G1  41=G0
|           ~                         |    42=L7  43=L6
| G 5 5 4 4 4 4 4 3 3 3 3 3 2 2 2 2 5 |    44=L5  45=L4
| N 3 1 9 7 5 3 1 9 7 5 3 1 9 7 5 3 V |    46=L3  47=L2
| D                                   |    48=L1  49=L0    SPI:
|                                     |    50=B3  51=B2     50=MISO 51=MOSI
|     2560                ____________/    52=B1  53=B0     52=SCK  53=SS
\_______________________/

Digitial Pin:
22: Left Wheel      INPUT1
23: Left Wheel      INPUT2

24: Right Wheel     INPUT1
25: Right Wheel     INPUT2

52: White init      BUTTON
53: Black init      BUTTON

ADC Pin:
A1 - A13 Sensors

LED Pin:
51: Red
50: Green

********************** Pin I/O Define end ********************/

/* define the wheel output pin */
// Left wheel
#define PIN_WHEEL_LEFT_VREF     (10)
#define PIN_WHEEL_LEFT_INPUT_1  (22)
#define PIN_WHEEL_LEFT_INPUT_2  (23)

// Right wheel
#define PIN_WHEEL_RIGHT_VREF    (9)
#define PIN_WHEEL_RIGHT_INPUT_1 (24)
#define PIN_WHEEL_RIGHT_INPUT_2 (25)

/* define the wheel output pin */
#define BUTTON_WHITE_INIT       (52)
#define BUTTON_BLACK_INIT       (53)

/* define the LED output Pin  */
#define LED_RED									(51)
#define LED_GREEN								(50)

/**** define the sensor information ****/
#define SENSOR_START_PIN        1
#define SENSOR_COUNT            13
#define SENSOR_SAMPLE_COUNT     1000
#define SENSOR_WHITE_THRESHOLD  50
#define TIME_RATIO              0.01

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
#define ERROR_MAX       (2.0)
#define ERROR_MIN       (-2.0)

#define ERROR_RATE_MAX  (0.5)
#define ERROR_RATE_MIN  (-0.5)

#define BASE_SPEED      (150)
#define SLOW_RATIO		(2)

#define K_P				(60)
#define K_D				(0)

// set the motor status
void motor_cmd(int _left_motor, int _right_motor);

// Init the pin I/O
void init_pin();

// compute error using sensor_value
float get_error();

// update the sensor_value
float get_sensor_data();


#ifdef TRACKING_EXCEPT_DETECT
bool except_detect();
#endif // TRACKING_EXCEPT_DETECT


void doSensorMaxInit();
void doSensorMinInit();

void setup()
{
	// Setup the serial debug tunnel
	Serial.begin(SERIAL_BAUD_);

	/* Setup the pin I/O mode */
	init_pin();

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

	float error_rate(0);
	float error_modify(0);

	time_stamp_old = time_stamp;
	time_stamp = millis();
	// period_time = float(time_stamp - time_stamp_old) * TIME_RATIO;
	period_time = float(time_stamp - time_stamp_old);

	get_sensor_data();

	// Store the old error value
	error_last = error;
	// Get the new error value via sensor_value
	error = get_error();
	// Compute the error rate
	error_rate = error - error_last;

	// If there is any exception while tracking the line, then use old error
#ifdef TRACKING_EXCEPT_DETECT
	if (except_detect() == true) {
		error = error_last;
		error_rate = error / 2;
	}
#endif // TRACKING_EXCEPT_DETECT


	// �d���L�j�L�p��ERROR �M ERROR RATE
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

	int output_kp = error_modify * K_P;
	int output_kd = error_rate * K_D;
	//Serial.println(output);
	int speed;
	speed = BASE_SPEED - abs(output_kp) * SLOW_RATIO;
	motor_cmd(speed + (int)output_kp, speed - 1 * (int)output_kp );

#ifdef SERIAL_DEBUG

	//   for (int i = 0; i < SENSOR_COUNT; i++) {
	//       Serial.print(sensor_value[i]); //debug
	//       Serial.print('\t');
	//   }
	//Serial.println('\t');
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

float get_error() {
	float sum(0);
	float weight_sum(0);
	for (int i = 0, error = 0; i < SENSOR_COUNT; i++) {
		weight_sum += (SENSOR_WEIGHT[i]) * (100 - sensor_value[i]);
		sum += 100 - sensor_value[i];
	}
	return weight_sum / sum;
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
bool except_detect() {
	int sensor_white;
	sensor_white = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (sensor_value[i] < SENSOR_WHITE_THRESHOLD) sensor_white++;
	}

	return (sensor_white == SENSOR_COUNT);

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
