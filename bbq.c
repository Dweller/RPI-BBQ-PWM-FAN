/* Hardware PWM fan control */
/* https://github.com/dstmar/WiringPi-PWM-Fan-Control */

#include <wiringPi.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <signal.h>
#include <sys/time.h>
#include <string.h>
#include <sys/socket.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <microhttpd.h>
#include "MAX6675.h"

// These values are fixed by Homekit
//
typedef enum {
    CoolingState_Off    =   0,
    CoolingState_Heat,
    CoolingState_Cool,
    CoolingState_Auto,
} CoolingState ;

static char* CoolingStateStr[] = { "Off", "Heat", "Cool", "Auto" };

static CoolingState targetState     = 0;
static double targetTemp            = 47.0;
static CoolingState currentState    = CoolingState_Heat;
static double currentTemp           = -0.0;
static bool End                     = false;

static pthread_t heartTid;
const int pwm_pin = 1;              // GPIO 1 as per WiringPi, GPIO18 as per BCM 
const int pi_freq = 54000000;       // Base frequency of PI - 54 MHz for Pi4B (19.2MHz for older models)
const int pwm_freq = 25000;         // Fan PWM Frequency in Hz
const int tach_pin = 3;             // GPIO 3 as per WiringPi, GPIO22 as per BCM
const int tach_pulse = 2;           // Number of pulses per fan revolution
const int refresh_time = 1;         // Seconds to wait between updates
const int bindPort = 80;            // Port used by socket
const int debug = 1;                // Set to 1 to print debug messages or 0 to run silently

static int range = 0;
static int rpm   = 0;

static int currentSpeed = 0;
static int currentRPM = 0;
static struct timeval currentTach;

// Sets fan percentage speed 
//
void set_speed(int speed)
{
    if (currentSpeed != speed)
    {
        speed = (speed > 100) ? 100 : (speed < 0) ? 0 : speed; // make sure speed is in range 0-100
        int duty = range * speed / 100;
        pwmWrite(pwm_pin, duty);
        currentSpeed = speed;
        if (debug) printf("speed : %d%%\n",speed);
    }
}

// Get fan RPM 
//
void get_rpm(void)
{
  struct timeval current_time;
  gettimeofday(&current_time, NULL);
  int dt = 1000000 * (current_time.tv_sec - currentTach.tv_sec) + current_time.tv_usec - currentTach.tv_usec;
  currentTach = current_time;
  if (dt > 0) rpm =  1000000 / dt / tach_pulse * 60;
}

// Print error message and exit 
//
void error(char *message)
{
  printf("Error : %s\n", message);
  exit(1);
}

// Get clock and range for PWM frequency 
//
int get_clock(void)
{
  int clock;
  int ratio = pi_freq / pwm_freq;
  int max_range = pi_freq / 2;
  if (pi_freq % pwm_freq == 0) // try to find an exact match for pwm_freq
  {
    for (int r = (ratio < max_range) ? ratio - 1 : max_range; r > 0; r--)
    {
      if (ratio % r == 0 && ratio / r >= 2)
      {
        if (ratio / r <= 4095)
        {
          range = r;
          clock = ratio / range;
        }
        break;
      }
    }
  }
  if (range == 0) // no exact match, get similar frequency
  {
    clock = ratio / max_range; // estimate clock for maximum range
    clock = (clock < 2) ? 2 : (clock > 4095) ? 4095 : clock; // make sure clock is in range 2-4095
    range = ratio / clock;
    if (range > max_range || range < 1) error("can't achieve this PWM frequency");
    if (debug) printf("No exact clock and range found - using %d Hz\n", pi_freq / clock / range);
  }
  if (debug) printf("clock div : %d, range : %d\n", clock, range);

  return clock;
}

// Graceful shutdown
//
void shutdownTrap(int signum)
{
  printf("Shutting down...\n");
  End = true;
  pthread_join(heartTid, NULL);
  pwmWrite(pwm_pin, 0);
  usleep(1000000);
  pinMode(pwm_pin, INPUT);
  pullUpDnControl(pwm_pin, PUD_DOWN);
  exit(0);
}

// Setup GPIO for tachometer and PWM 
//
void setup_gpio(void)
{
  if (wiringPiSetup() == -1) error("wiringPiSetup failed");
  if (refresh_time <= 0) error("refresh_time must be at least 1");

  // setup rpm tachometer
  if (tach_pulse <= 0) error("tach_pulse must be at least 1");

  pinMode(tach_pin, INPUT);
  pullUpDnControl(tach_pin, PUD_DOWN);
  gettimeofday(&currentTach, NULL);
  wiringPiISR(tach_pin, INT_EDGE_RISING, &get_rpm);

  int clock = get_clock();
  pinMode(pwm_pin, PWM_OUTPUT);
  pullUpDnControl(pwm_pin, PUD_OFF);
  pwmSetMode(PWM_MODE_MS);
  pwmSetRange(range);
  pwmSetClock(clock);
  pwmWrite(pwm_pin, 0);
}

// heart thread is what reads the themmocouple and controls the fan
//
void * heartThread(void* arg)
{
    MAX6675 max6675 = MAX6675Setup(0);

    setup_gpio();

    int prev_rpm = -1;
    struct timeval prev_tach = currentTach;

    while(!End) {
        currentTemp = MAX6675GetTempC(max6675);

        if(debug) printf("currentTemp:%0.2f,targetTemp:%0.2f\n", currentTemp, targetTemp);

        if(currentState != currentState) {
            if(targetState == CoolingState_Off) {
                set_speed(0);
            }
            currentState = targetState;
        }
        else if (currentTemp > targetTemp) {
            set_speed(0);
        }
        else {
            double left = ((targetTemp - currentTemp) * 100.0) / targetTemp;
            if(left > 0.0) {
                set_speed(left);
            }
        }

        if (!timercmp(&prev_tach, &currentTach, !=))
        rpm = 0;

        if (debug && prev_rpm != rpm) printf("rpm : %d\n", rpm);

        prev_rpm = rpm;
        prev_tach = currentTach;

        usleep(1000000);
    }

    MAX6675Free(max6675);

    return arg;
}

int parse_qs (void *arg, enum MHD_ValueKind kind, const char *key, const char *val)
{
    int* valu = arg;

    if(!strcmp(key, "value")) {
        *valu = atoi(val);
    }
    return MHD_YES;
}

static int qs_proc (void *cls,
    struct MHD_Connection*        conn,
    const char*                    url,
    const char*                 method,
    const char*                version,
    const char*            upload_data, 
    size_t*           upload_data_size, 
    void **                        ptr)
{
    struct MHD_Response *res = NULL;

    if(url == NULL) {
        return MHD_NO;
    }

    int valu=INT_MIN;
    MHD_get_connection_values (conn, MHD_GET_ARGUMENT_KIND, parse_qs, &valu);

    char* body = NULL;
    if (!strcmp(url, "/status")) {
        asprintf(&body, "{\"targetHeatingCoolingState\": %d,\"targetTemperature\": %.2f,\"currentHeatingCoolingState\": %d,\"currentTemperature\": %.2f}", 
            targetState, targetTemp,
            currentState, currentTemp);
    }
    else if(valu != INT_MIN) {
        if (!strcmp(url, "/targetTemperature")) {
            targetTemp = valu;
            body = calloc(1, 1);
        }
        else if(!strcmp(url, "/targetHeatingCoolingState")) {
            targetState = valu;
            body = calloc(1, 1);
        }
        else if(!strcmp(url, "/currentTempreture")) {
            currentTemp = valu;
            body = calloc(1, 1);
        }
        asprintf(&body, "{\"targetHeatingCoolingState\": %d,\"targetTemperature\": %.2f,\"currentHeatingCoolingState\": %d,\"currentTemperature\": %.2f}", 
            targetState, targetTemp,
            currentState, currentTemp);
    }

    if(body == NULL) {
        return MHD_NO;
    }

    res = MHD_create_response_from_buffer(strlen(body), body,  MHD_RESPMEM_MUST_FREE);
    MHD_add_response_header(res, MHD_HTTP_HEADER_CONTENT_ENCODING, "application/json");
    MHD_add_response_header(res, "Connection", "close");

    int ret = MHD_queue_response (conn, MHD_HTTP_OK, res);
    MHD_destroy_response (res);
    return ret;
}

int main (void)
{
    // shutdown interrupts
    signal(SIGINT, shutdownTrap);
    signal(SIGTERM, shutdownTrap);

    // Start up the heartbeat thead which reads the therocouple and spins the fan.
    //
    pthread_create(&heartTid, NULL, heartThread, NULL);

    struct MHD_Daemon* d = MHD_start_daemon (  MHD_USE_THREAD_PER_CONNECTION
                                            | MHD_USE_INTERNAL_POLLING_THREAD
                                            | MHD_USE_DEBUG,
                                            bindPort, NULL, NULL, &qs_proc, 0, MHD_OPTION_END);
    pthread_join(heartTid, NULL);

    if (d == NULL) {
        return 1;
    }
    return 0;
}

