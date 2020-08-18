//Based on https://github.com/m5stack/M5-ProductExampleCodes/tree/master/AtomBase/AtomFLY

//Press the Atom button, test the propeller rotates in turn, and the serial monitor outputs the IMU status

#include <M5Atom.h>
#include "AtomFly.h"

CRGB led(0, 0, 0);

double r_rand = 180 / PI;

float pitch, roll, yaw;
double last_pitch = 0, last_roll = 0;

uint8_t mode = 0;
uint32_t on_time = 0, off_time = 0;

//PID calculations
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

//PID gain and limit settings
float pid_p_gain_roll = 0.5;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 10.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)


#define SERIAL_DEBUG 0

#if SERIAL_DEBUG
  #define SERIAL_PRINTF Serial.printf
#else
  #define SERIAL_PRINTF 
#endif

#define FLY_DURATION (8 * 1000)

void setup()
{
    M5.begin(true, false, true);

    fly.begin();
    if ( fly.initFly() != 0)
    {
        Serial.println("initFly() failed");
        led = CRGB(0,255,0);
        M5.dis.drawpix(0, led);
        while(1)
        {
            delay(100);
        }
    }

    Serial.println("initFly() ok");
    led = CRGB(255,0,0);
    M5.dis.drawpix(0, led);

    //PID reset
    pid_roll_setpoint = 0;
    pid_pitch_setpoint = 0;

    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    //pid_i_mem_yaw = 0;
    //pid_last_yaw_d_error = 0;

    //fly.CalibrateGyro();
}

void calculate_pid()
{
  //PID reset, temporary
  pid_roll_setpoint = 0;
  pid_pitch_setpoint = 0;

  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;

  //Roll calculations
  pid_error_temp = roll - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

  if (pid_i_mem_roll > pid_max_roll)
    pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < -pid_max_roll)
    pid_i_mem_roll = -pid_max_roll;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if (pid_output_roll < -pid_max_roll)
    pid_output_roll = -pid_max_roll;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = pitch - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < -pid_max_pitch)
    pid_i_mem_pitch = -pid_max_pitch;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < -pid_max_pitch)
    pid_output_pitch = -pid_max_pitch;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
//   pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
//   pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
//   if (pid_i_mem_yaw > pid_max_yaw)
//     pid_i_mem_yaw = pid_max_yaw;
//   else if (pid_i_mem_yaw < -pid_max_yaw)
//     pid_i_mem_yaw = -pid_max_yaw;

//   pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
//   if (pid_output_yaw > pid_max_yaw)
//     pid_output_yaw = pid_max_yaw;
//   else if (pid_output_yaw < -pid_max_yaw)
//     pid_output_yaw = -pid_max_yaw;

//   pid_last_yaw_d_error = pid_error_temp;

  pid_output_yaw = 0;
}

void loop()
{
    fly.getAttitude(&pitch, &roll, &yaw);

    calculate_pid();

    //TODO throttle needs to come from remote control
    //  also update pwm values based on direction headed
    int throttle = 950;

    int esc_A = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW) //A
    int esc_B = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW) //B
    int esc_C = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW) //C
    int esc_D = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW) //D

    //scale down pwm values
    esc_A /= 10;
    esc_B /= 10;
    esc_C /= 10;
    esc_D /= 10;

    SERIAL_PRINTF("p %.2f  r %.2f  y %.2f ", pitch, roll, yaw);
    SERIAL_PRINTF("A %d  B %d  C %d  D %d\n", esc_A, esc_B, esc_C, esc_D);

    if (mode == 1)
    {
        if (millis() - on_time > FLY_DURATION)
        {
            fly.WritePWM(AtomFly::kMotor_A, 0);
            fly.WritePWM(AtomFly::kMotor_B, 0);
            fly.WritePWM(AtomFly::kMotor_C, 0);
            fly.WritePWM(AtomFly::kMotor_D, 0);
            mode = 0;

            led = CRGB(0,255,0);
            M5.dis.drawpix(0, led);

            SERIAL_PRINTF("turn off\n");
        }
        else if (millis() - on_time < 2000 )
        {
          //wait 2 sec before turn on
        }
        else if (millis() - on_time < 4000 )
        {
          //next 2 sec, warm up

          fly.WritePWM(AtomFly::kMotor_A, 40);
          fly.WritePWM(AtomFly::kMotor_B, 40);
          fly.WritePWM(AtomFly::kMotor_C, 40);
          fly.WritePWM(AtomFly::kMotor_D, 40);
        }
        else
        {
            fly.WritePWM(AtomFly::kMotor_A, esc_A);
            fly.WritePWM(AtomFly::kMotor_B, esc_B);
            fly.WritePWM(AtomFly::kMotor_C, esc_C);
            fly.WritePWM(AtomFly::kMotor_D, esc_D);
        }
    }
    else if (M5.Btn.wasPressed())
    {
        SERIAL_PRINTF("turn on\n");

        led = CRGB(255,0,0);
        M5.dis.drawpix(0, led);

        mode = 1;
        on_time = millis();
    }

#if 0 //update LED based on TOF
    // float arc = atan2(pitch, roll) * r_rand + 180;
    // float dist = sqrt(pitch * pitch + roll * roll);

    //SERIAL_PRINTF("p %.2f\t r %.2f\t a %.2f\t v %.2f\n", pitch, roll, arc, val);

    // uint16_t tofData = fly.readTOF();
    // tofData = (tofData > 500) ? 500 : tofData;

    // float val = (dist * 6) > 100 ? 100 : dist * 6;

    // led = HSVtoRGB(arc, val, 100 - (tofData / 5));
    // M5.dis.drawpix(0, led);
#endif

    M5.update();

#if SERIAL_DEBUG
    //delay(10);
#endif
}
