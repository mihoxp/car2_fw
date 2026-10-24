#include <stdio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/u_int32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>

#include <math.h>
#include <malloc.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/sync.h"

#include "udp_transport.h"
#include "motor_config.h"
#include "ultrasonic.h"
#include "ws2812.h"
#include "ssd1306.h"
#include "ir.h"
#include "buzzer.h"
#include "ble_v7rc.h"
#include "servo.h"
#include "sound.h"
#include "gps.h"
#include "sbus.h"

#define ROS_DOMAIN_ID 20
#define PWM_WRAP      999
#define RANGE_PERIOD_MS 100
#define BATTERY_PERIOD_MS 1000
#define BATTERY_ADC_PIN   28    // same as YAHBOOM's sensors.check_battery_level()

// Light follow, based on YAHBOOM's "5.8 Light follow" lesson (needs the jumper
// cap on the Light header). Measured on this car: light1 (GP27) is the RIGHT
// sensor, light2 (GP26) the LEFT one, and a brighter light gives a LOWER value.
#define LIGHT1_ADC_PIN    27
#define LIGHT2_ADC_PIN    26
#define LIGHT_PERIOD_MS   100
#define FOLLOW_PERIOD_MS  50

// Line tracking, based on YAHBOOM's "4.3 Tracking sensor" lesson: four digital
// sensors, 0 = black line, 1 = white floor (needs the jumper cap on the
// tracking header). The lesson lists them as GP2..GP5 left to right, but on
// this car the physical order, left to right, measured with tape, is:
static const uint LINE_PINS[4] = {3, 2, 5, 4};
#define TRACK_PERIOD_MS   20
#define TRACK_FAST        0.40f   // straight, and outer wheel when turning
#define TRACK_SLOW        0.12f   // inner wheel on a gentle turn
#define TRACK_REVERSE     0.25f   // inner wheel backwards on a sharp turn
#define TRACK_SEARCH_MS   1000    // keep turning this long after losing the line

// Ultrasonic avoiding, based on YAHBOOM's "5.6 Ultrasonic avoiding" lesson
// (forward; when close: back up, turn right, check again). Distances and
// times are larger than YAHBOOM's 10 cm / 0.2 s because the motors need 60%
// duty to move on the floor and the range only updates at 10 Hz.
#define AVOID_PERIOD_MS   20
#define AVOID_NEAR_M      0.15f
#define AVOID_MID_M       0.30f
#define AVOID_FORWARD     0.30f   // ~72% actual; 0.20 (68%) stalled on the floor
#define AVOID_BACK        0.25f
#define AVOID_TURN        0.30f
#define AVOID_BACK_MS     700
#define AVOID_TURN_MS     800
#define COLOR_AVOID_FAR   0x003000   // green, like YAHBOOM
#define COLOR_AVOID_MID   0x302000   // yellow
#define FOLLOW_DUTY       (120.0f / 255.0f)   // YAHBOOM's Car_Run(120, ...)
// Brightness = 65535 - reading. A side must be this many times brighter than
// the other to turn; wider than YAHBOOM's 1.1 because the two sensors differ.
#define FOLLOW_TURN_RATIO 1.3f
// Brighter side below this = no light to follow (stop); hysteresis so the car
// does not flicker between stop and go at the edge.
#define FOLLOW_LIGHT_ON   18000
#define FOLLOW_LIGHT_OFF  15000

enum agent_state { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };

#define OLED_PERIOD_MS 1000

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_subscription_t sub;
static rclc_executor_t executor;
static geometry_msgs__msg__Twist twist;
static rcl_publisher_t range_pub;
static rcl_timer_t range_timer;
static sensor_msgs__msg__Range range_msg;
static char range_frame_id[] = "car2_ultrasonic";
static float last_range_m = -1.0f;
static rcl_publisher_t temp_pub;
static sensor_msgs__msg__Temperature temp_msg;
static char temp_frame_id[] = "car2_rp2040";
static rcl_publisher_t battery_pub;
static rcl_timer_t battery_timer;
static std_msgs__msg__UInt16 battery_msg;
static rcl_subscription_t leds_sub;
static std_msgs__msg__UInt32MultiArray leds_msg;
// micro-ROS has no dynamic allocation for incoming sequences: preallocate them
static uint32_t leds_data[WS2812_NUM_LEDS];
static std_msgs__msg__MultiArrayDimension leds_dim[2];
static char leds_dim_label[2][16];
static rcl_subscription_t auto_lights_sub;
static std_msgs__msg__Bool auto_lights_msg;
static rcl_publisher_t light_pub;
static rcl_timer_t light_timer;
static std_msgs__msg__UInt16MultiArray light_msg;
static uint16_t light_data[2];
static rcl_publisher_t line_pub;
static std_msgs__msg__UInt8MultiArray line_msg;
static uint8_t line_data[4];
// One topic picks the drive mode, so autonomous modes can never run together
static rcl_subscription_t mode_sub;
static std_msgs__msg__String mode_msg;
static char mode_buf[32];
static rcl_subscription_t buzzer_sub;
static std_msgs__msg__UInt16MultiArray buzzer_msg;     // [freq_hz, duration_ms]
static uint16_t buzzer_data[2];
static std_msgs__msg__MultiArrayDimension buzzer_dim[2];
static char buzzer_dim_label[2][16];
static rcl_subscription_t servo_sub;
static std_msgs__msg__UInt16MultiArray servo_msg;      // angles for S1..S3
static uint16_t servo_data[SERVO_COUNT];
static std_msgs__msg__MultiArrayDimension servo_dim[2];
static char servo_dim_label[2][16];
static rcl_publisher_t sound_pub;
static std_msgs__msg__UInt16 sound_msg;
static rcl_publisher_t gps_pub;
static sensor_msgs__msg__NavSatFix gps_msg;
static char gps_frame_id[] = "car2_gps";
static rcl_publisher_t nmea_pub;
static std_msgs__msg__String nmea_msg;
static char nmea_buf[100];
static bool nmea_ready;     // publish only while connected
static rcl_publisher_t ir_pub;
static std_msgs__msg__UInt8 ir_msg;
enum drive_mode { MODE_MANUAL, MODE_LIGHT_FOLLOW, MODE_LINE_TRACK, MODE_AVOID };
enum avoid_state { AVOID_GO, AVOID_BACKING, AVOID_TURNING };
static enum avoid_state avoid_state;
static absolute_time_t avoid_until;
static enum drive_mode mode = MODE_MANUAL;
static bool follow_has_light;
static int track_last_turn;              // +1 left, -1 right, 0 straight
static absolute_time_t track_last_seen;

// Automatic turn signals / head / brake lights driven by cmd_vel.
// LED layout seen from above (mapped by hand on the car): two columns,
// chain runs L3 L2 L1 R1 R2 R3 R4 L4
//            front
//          3       4
//          2       5
//          1       6
//          8       7
//            rear
#define LIGHTS_PERIOD_MS   50
#define BLINK_HALF_MS      500   // 1 Hz, like a car indicator
#define BRAKE_MS           1000
#define COLOR_AMBER        0x302000
#define COLOR_HEAD         0x202020
#define COLOR_BRAKE        0x300000
#define COLOR_REVERSE      0x202020
// Zero-based LED indexes for each group
static const uint8_t LEDS_FRONT[] = {2, 3};
static const uint8_t LEDS_REAR[]  = {7, 6};
static const uint8_t LEDS_LEFT[]  = {2, 1, 0, 7};
static const uint8_t LEDS_RIGHT[] = {3, 4, 5, 6};
#define GROUP(g) (g), sizeof(g)

// Obstacle warning with hysteresis so it does not flicker at the edge
#define NEAR_ON_M          0.15f
#define NEAR_OFF_M         0.18f
#define COLOR_WARN         0x300000
#define NEAR_CLEAR_COUNT   10      // far readings in a row (~1 s) needed to clear;
                                   // very close objects sometimes echo off the wall behind

static bool auto_lights = true;
static bool too_close;
static int far_count;
static bool oled_dirty;

// Called once per new ultrasonic reading. Turns on at once, but clears only
// after several far readings so a single bad echo does not drop the warning.
static void warning_update(float d)
{
    bool was_close = too_close;
    bool valid = d > 0 && d <= 4.0f;
    if (valid && d < NEAR_ON_M) {
        too_close = true;
        far_count = 0;
    } else if (too_close && (!valid || d > NEAR_OFF_M)) {
        if (++far_count >= NEAR_CLEAR_COUNT) {
            too_close = false;
        }
    } else {
        far_count = 0;
    }
    if (too_close != was_close) {
        printf("too close: %d (%.3f m)\n", too_close, d);
        oled_dirty = true;   // show the change right away
    }
}
static float cmd_linear, cmd_angular;
static absolute_time_t brake_until;

static absolute_time_t last_cmd_time;
static bool moving;
static bool led_on;

static void pwm_pin_init(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_clkdiv(slice, (float)clock_get_hz(clk_sys) / ((PWM_WRAP + 1) * MOTOR_PWM_HZ));
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice, true);
}

static void motor_set(uint pin_a, uint pin_b, bool invert, float duty)
{
    if (invert) {
        duty = -duty;
    }
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    float mag = duty < 0 ? -duty : duty;
    // On the floor the N20 motors stall below ~60%: start any requested motion
    // at MOTOR_MIN_DUTY and scale the rest, so small commands still move the car
    if (mag > 0.01f) {
        mag = MOTOR_MIN_DUTY + (1.0f - MOTOR_MIN_DUTY) * mag;
    } else {
        mag = 0;
    }
    uint16_t level = (uint16_t)(mag * PWM_WRAP);
    pwm_set_gpio_level(pin_a, duty > 0 ? level : 0);
    pwm_set_gpio_level(pin_b, duty < 0 ? level : 0);
}

// Drive each wheel directly; duty is -1.0 .. 1.0
static void motors_wheels(float left, float right)
{
#if MOTOR_ENABLED
    motor_set(LEFT_PIN_A, LEFT_PIN_B, LEFT_INVERT, left);
    motor_set(RIGHT_PIN_A, RIGHT_PIN_B, RIGHT_INVERT, right);
#else
    (void)left;
    (void)right;
#endif
}

static void motors_drive(float linear, float angular)
{
#if MOTOR_ENABLED
    float left = linear - angular * WHEEL_SEPARATION_M / 2.0f;
    float right = linear + angular * WHEEL_SEPARATION_M / 2.0f;
    motor_set(LEFT_PIN_A, LEFT_PIN_B, LEFT_INVERT, left / MAX_WHEEL_SPEED_MPS);
    motor_set(RIGHT_PIN_A, RIGHT_PIN_B, RIGHT_INVERT, right / MAX_WHEEL_SPEED_MPS);
#else
    (void)linear;
    (void)angular;
#endif
}

static void motors_init(void)
{
#if MOTOR_ENABLED
    pwm_pin_init(LEFT_PIN_A);
    pwm_pin_init(LEFT_PIN_B);
    pwm_pin_init(RIGHT_PIN_A);
    pwm_pin_init(RIGHT_PIN_B);
#endif
    motors_drive(0, 0);
}

#if PWM_DEBUG
// Print each motor pin's PWM setup and its measured duty (pad level sampled
// through the input buffer), to tell a firmware problem from a hardware one.
static void pwm_debug_pin(const char *name, uint pin)
{
    uint slice = pwm_gpio_to_slice_num(pin);
    uint chan = pwm_gpio_to_channel(pin);
    uint32_t cc = pwm_hw->slice[slice].cc;
    uint16_t level = chan ? (cc >> 16) : (cc & 0xffff);
    bool enabled = pwm_hw->slice[slice].csr & PWM_CH0_CSR_EN_BITS;
    uint high = 0;
    const uint samples = 4000;
    for (uint i = 0; i < samples; i++) {
        high += gpio_get(pin);
        busy_wait_us_32(1);
    }
    printf("  %s GP%-2u func=%u slice=%u%c en=%d top=%lu div=%lu.%lu level=%u measured=%u%%\n",
           name, pin, gpio_get_function(pin), slice, chan ? 'B' : 'A', enabled,
           (unsigned long)pwm_hw->slice[slice].top,
           (unsigned long)(pwm_hw->slice[slice].div >> 4),
           (unsigned long)(pwm_hw->slice[slice].div & 0xf),
           level, high * 100 / samples);
}

static void pwm_debug(void)
{
    printf("[pwm] MOTOR_ENABLED=%d clk_sys=%lu Hz\n", MOTOR_ENABLED,
           (unsigned long)clock_get_hz(clk_sys));
    pwm_debug_pin("L_A", LEFT_PIN_A);
    pwm_debug_pin("L_B", LEFT_PIN_B);
    pwm_debug_pin("R_A", RIGHT_PIN_A);
    pwm_debug_pin("R_B", RIGHT_PIN_B);
}
#endif

// Manual drive command from /car2/cmd_vel or the V7RC BLE remote
static bool ir_driving;     // the current drive command came from the IR remote
static absolute_time_t ir_hold_until;

static void drive_cmd(float linear, float angular)
{
    if (mode != MODE_MANUAL) {
        // Manual driving always wins over the autonomous modes
        mode = MODE_MANUAL;
        printf("mode manual: drive command received\n");
        oled_dirty = true;
    }
    ir_driving = false;     // any other drive command takes over from the IR remote
    motors_drive(linear, angular);
#if PWM_DEBUG
    static uint count;
    if (count++ % 10 == 0) {
        pwm_debug();
    }
#endif
    bool was_moving = fabsf(cmd_linear) > 0.01f || fabsf(cmd_angular) > 0.1f;
    cmd_linear = linear;
    cmd_angular = angular;
    if (was_moving && fabsf(cmd_linear) <= 0.01f && fabsf(cmd_angular) <= 0.1f) {
        brake_until = make_timeout_time_ms(BRAKE_MS);
    }
    last_cmd_time = get_absolute_time();
    moving = true;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
}

static void cmd_vel_cb(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = msgin;
    printf("cmd_vel: %.2f, %.2f, %.2f\n", m->linear.x, m->linear.y, m->angular.z);
    drive_cmd(m->linear.x, m->angular.z);
}

#define V7RC_DEADBAND   50      // ignore stick values within 1500 +- 50
// Full stick = full wheel speed; full steering = wheels at +-100% in opposite directions
#define V7RC_MAX_ANGULAR (2.0f * MAX_WHEEL_SPEED_MPS / WHEEL_SEPARATION_M)

// IR remote (car's NEC remote, codes verified 2026-10-24): drive while a key is
// held; the remote repeats every ~108 ms, so stop when repeats stop
#define IR_KEY_UP       0x40
#define IR_KEY_DOWN     0x19
#define IR_KEY_LEFT     0x07
#define IR_KEY_RIGHT    0x09
#define IR_KEY_OK       0x15
#define IR_KEY_1        0x0C    // line_track
#define IR_KEY_2        0x18    // avoid
#define IR_KEY_3        0x5E    // light_follow
#define IR_HOLD_MS      250
#define IR_SPEED        0.5f    // fraction of full speed

#define SERVO_V7RC_DEADBAND 10
#define SERVO_V7RC_MIN_SPAN 100     // the second stick only sends ~1500 +- 100 (measured)
#define SERVO_CENTER        90

static float v7rc_axis(int ch)
{
    int d = ch - 1500;
    if (d > -V7RC_DEADBAND && d < V7RC_DEADBAND) {
        return 0.0f;
    }
    float v = d / 500.0f;
    return v > 1.0f ? 1.0f : v < -1.0f ? -1.0f : v;
}

static bool set_mode(const char *name);

static void ir_drive_key(uint8_t key)
{
    float linear = 0.0f, angular = 0.0f;
    const char *mode_name = NULL;
    switch (key) {
    case IR_KEY_1: mode_name = "line_track"; break;
    case IR_KEY_2: mode_name = "avoid"; break;
    case IR_KEY_3: mode_name = "light_follow"; break;
    }
    if (mode_name) {
        if (set_mode(mode_name)) {
            buzzer_beep(2000, 80);  // no screen needed to know it worked
        }
        return;
    }
    switch (key) {
    case IR_KEY_UP:    linear = IR_SPEED * MAX_WHEEL_SPEED_MPS; break;
    case IR_KEY_DOWN:  linear = -IR_SPEED * MAX_WHEEL_SPEED_MPS; break;
    case IR_KEY_LEFT:  angular = IR_SPEED * V7RC_MAX_ANGULAR; break;
    case IR_KEY_RIGHT: angular = -IR_SPEED * V7RC_MAX_ANGULAR; break;
    case IR_KEY_OK:    drive_cmd(0.0f, 0.0f); return;
    default:           return;
    }
    drive_cmd(linear, angular);
    ir_driving = true;
    ir_hold_until = make_timeout_time_ms(IR_HOLD_MS);
}

static void ir_drive_update(void)
{
    if (!ir_driving) {
        return;
    }
    if (ir_repeat()) {
        // Key still held: keep going and refresh the safety timeout
        ir_hold_until = make_timeout_time_ms(IR_HOLD_MS);
        last_cmd_time = get_absolute_time();
    } else if (time_reached(ir_hold_until)) {
        drive_cmd(0.0f, 0.0f);
    }
}

// HOTRC HT-10A through its SBUS receiver (mapped 2026-10-24, Mode 2):
// right stick drives like the V7RC stick, left stick sets servos S1 / S2
#define SBUS_CENTER     992
#define SBUS_HALF_RANGE 800     // 192..1792
#define SBUS_DEADBAND   40
#define SBUS_TIMEOUT_MS 200     // receiver stopped sending frames
#define SBUS_CH_STEER   0       // CH1: right stick, right = 192
#define SBUS_CH_THROTTLE 1      // CH2: right stick, up = 1792
#define SBUS_CH_SERVO2  2       // CH3: left stick up / down -> S2
#define SBUS_CH_SERVO1  3       // CH4: left stick left / right -> S1

static float sbus_axis(uint16_t v)
{
    int d = (int)v - SBUS_CENTER;
    if (d > -SBUS_DEADBAND && d < SBUS_DEADBAND) {
        return 0.0f;
    }
    float a = (float)d / SBUS_HALF_RANGE;
    return a > 1.0f ? 1.0f : a < -1.0f ? -1.0f : a;
}

static void sbus_update(void)
{
    static bool sbus_moving, servo_moved[2];
    static absolute_time_t last_frame;
    static uint16_t last_ch[4];
    uint16_t ch[SBUS_CHANNELS];
    bool fs;
    if (!sbus_get(ch, &fs)) {
        // Transmitter off or out of range: some receivers just stop sending
        if (sbus_moving && absolute_time_diff_us(last_frame, get_absolute_time()) > SBUS_TIMEOUT_MS * 1000) {
            printf("sbus: no frames, stopping\n");
            sbus_moving = false;
            drive_cmd(0.0f, 0.0f);
        }
        return;
    }
    last_frame = get_absolute_time();
    for (int i = 0; i < 4; i++) {
        last_ch[i] = ch[i];
    }
    static absolute_time_t next_print;
    if (time_reached(next_print)) {
        next_print = make_timeout_time_ms(5000);
        printf("sbus ch1-4: %u %u %u %u%s\n", last_ch[0], last_ch[1], last_ch[2], last_ch[3],
               fs ? " FAILSAFE" : "");
    }
    if (fs) {
        if (sbus_moving) {
            printf("sbus: failsafe, stopping\n");
            sbus_moving = false;
            drive_cmd(0.0f, 0.0f);
        }
        return;
    }

    // Left stick -> servos, back to 90 on release; like V7RC, only the first
    // centered frame returns them so an idle transmitter does not override others
    const int servo_ch[2] = {SBUS_CH_SERVO1, SBUS_CH_SERVO2};
    for (int i = 0; i < 2; i++) {
        float a = sbus_axis(ch[servo_ch[i]]);
        if (a == 0.0f) {
            if (servo_moved[i]) {
                servo_set(i, SERVO_CENTER);
                servo_moved[i] = false;
            }
            continue;
        }
        // stick right (low value, like CH1) = larger S1 angle, stick up = larger S2 angle
        int offset = (int)(a * SERVO_CENTER);
        servo_set(i, SERVO_CENTER + (i == 0 ? -offset : offset));
        servo_moved[i] = true;
    }

    float linear = sbus_axis(ch[SBUS_CH_THROTTLE]) * MAX_WHEEL_SPEED_MPS;
    float angular = sbus_axis(ch[SBUS_CH_STEER]) * V7RC_MAX_ANGULAR;   // right = 192 = turn right
    bool idle = linear == 0.0f && angular == 0.0f;
    if (idle && !sbus_moving) {
        return;
    }
    sbus_moving = !idle;
    drive_cmd(linear, angular);
}

static void v7rc_update(void)
{
    int ch[V7RC_CHANNELS];
    if (!ble_v7rc_get(ch)) {
        return;
    }
    int ch1 = ch[0], ch2 = ch[1];
    // ch3 / ch4 (second stick, springs back to center) set the angle of
    // servos S1 / S2: full stick = 0 or 180 degrees, released = back to 90.
    // The stick's range is much smaller than 1000..2000, so the largest
    // deflection seen so far counts as full stick
    static int span[2] = {SERVO_V7RC_MIN_SPAN, SERVO_V7RC_MIN_SPAN};
    // Like driving, only the first centered frame returns the servo, so an
    // idle phone does not override /car2/servo
    static bool servo_moved[2];
    for (int i = 0; i < 2; i++) {
        int d = ch[2 + i] - 1500;
        int a = d < 0 ? -d : d;
        if (a > span[i]) {
            span[i] = a;
            printf("v7rc: ch%d full range now 1500 +- %d\n", 3 + i, a);
        }
        if (a < SERVO_V7RC_DEADBAND) {
            if (servo_moved[i]) {
                servo_set(i, SERVO_CENTER);
                servo_moved[i] = false;
            }
            continue;
        }
        // stick right = larger S1 angle, stick up (< 1500) = larger S2 angle
        int offset = d * SERVO_CENTER / span[i];
        servo_set(i, SERVO_CENTER + (i == 0 ? offset : -offset));
        servo_moved[i] = true;
    }
    // On the phone, throttle up gives ch2 < 1500 (tested 2026-10-24)
    float linear = -v7rc_axis(ch2) * MAX_WHEEL_SPEED_MPS;
    float angular = -v7rc_axis(ch1) * V7RC_MAX_ANGULAR;     // stick right = turn right
    // V7RC keeps sending centered sticks: only the first one is a stop, so an
    // idle phone does not override /car2/cmd_vel or cancel an autonomous mode
    static bool ble_moving;
    bool idle = linear == 0.0f && angular == 0.0f;
    if (idle && !ble_moving) {
        return;
    }
    ble_moving = !idle;
    drive_cmd(linear, angular);
}

static void range_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }
    float d = ultrasonic_read_m();
    last_range_m = d;
    warning_update(d);
    int64_t now_ns = rmw_uros_epoch_nanos();
    range_msg.header.stamp.sec = now_ns / 1000000000;
    range_msg.header.stamp.nanosec = now_ns % 1000000000;
    range_msg.range = (d < 0 || d > range_msg.max_range) ? INFINITY : d;
    rcl_ret_t rc = rcl_publish(&range_pub, &range_msg, NULL);
    (void)rc;
}

static void range_msg_init(void)
{
    range_msg.header.frame_id.data = range_frame_id;
    range_msg.header.frame_id.size = sizeof(range_frame_id) - 1;
    range_msg.header.frame_id.capacity = sizeof(range_frame_id);
    range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    range_msg.field_of_view = 0.26f;    // ~15 degrees, typical HC-SR04 beam
    range_msg.min_range = 0.02f;
    range_msg.max_range = 4.0f;
}

// Raw battery ADC scaled to 0-65535 like MicroPython's read_u16(), so it can be
// compared with YAHBOOM's thresholds (<20000 low, >=25000 full)
static uint16_t adc_read_u16(uint pin)
{
    // Keep the 2 kHz sound sampling interrupt from switching the ADC input
    uint32_t irq = save_and_disable_interrupts();
    adc_select_input(pin - 26);
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += adc_read();
    }
    restore_interrupts(irq);
    uint16_t raw12 = sum / 16;
    return (raw12 << 4) | (raw12 >> 8);
}

// RP2040 die temperature from ADC input 4 (datasheet 4.9.5):
// T = 27 - (V - 0.706) / 0.001721, with V from the 3.3 V reference
static float chip_temp_c(void)
{
    uint32_t irq = save_and_disable_interrupts();
    adc_select_input(4);
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += adc_read();
    }
    restore_interrupts(irq);
    float v = (sum / 16.0f) * 3.3f / 4096.0f;
    return 27.0f - (v - 0.706f) / 0.001721f;
}

static uint16_t battery_read_u16(void)
{
    return adc_read_u16(BATTERY_ADC_PIN);
}

// Sensors left to right, 1 = on the black line
static void line_read(uint8_t on_line[4])
{
    for (int i = 0; i < 4; i++) {
        on_line[i] = !gpio_get(LINE_PINS[i]);
    }
}

static void light_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }
    light_data[0] = adc_read_u16(LIGHT1_ADC_PIN);
    light_data[1] = adc_read_u16(LIGHT2_ADC_PIN);
    rcl_ret_t rc = rcl_publish(&light_pub, &light_msg, NULL);
    sound_msg.data = sound_take_level();
    rc = rcl_publish(&sound_pub, &sound_msg, NULL);
    line_read(line_data);
    rc = rcl_publish(&line_pub, &line_msg, NULL);
    (void)rc;
}

static void stop_and_brake(void)
{
    motors_wheels(0, 0);
    if (fabsf(cmd_linear) > 0.01f || fabsf(cmd_angular) > 0.1f) {
        brake_until = make_timeout_time_ms(BRAKE_MS);
    }
    cmd_linear = 0;
    cmd_angular = 0;
}

static bool set_mode(const char *name)
{
    if (strcmp(name, "manual") == 0) {
        mode = MODE_MANUAL;
    } else if (strcmp(name, "light_follow") == 0) {
        mode = MODE_LIGHT_FOLLOW;
    } else if (strcmp(name, "line_track") == 0) {
        mode = MODE_LINE_TRACK;
    } else if (strcmp(name, "avoid") == 0) {
        mode = MODE_AVOID;
    } else {
        printf("unknown mode: %s (use manual, light_follow, line_track, avoid)\n", name);
        return false;
    }
    printf("mode: %s\n", name);
    oled_dirty = true;
    follow_has_light = false;
    track_last_turn = 0;
    track_last_seen = get_absolute_time();
    avoid_state = AVOID_GO;
    ir_driving = false;     // a released IR key must not cancel the new mode
    stop_and_brake();
    return true;
}

static void mode_cb(const void *msgin)
{
    const std_msgs__msg__String *m = msgin;
    set_mode(m->data.data ? m->data.data : "");
}

static bool range_valid(float d)
{
    return d > 0 && d <= 4.0f;
}

// Non-blocking version of YAHBOOM's loop so ROS, the OLED and the safety
// checks keep running while the car backs up and turns.
static void avoid_step(void)
{
    switch (avoid_state) {
    case AVOID_GO:
        if (range_valid(last_range_m) && last_range_m < AVOID_NEAR_M) {
            printf("avoid: obstacle at %.2f m, backing up\n", last_range_m);
            avoid_state = AVOID_BACKING;
            avoid_until = make_timeout_time_ms(AVOID_BACK_MS);
            motors_wheels(-AVOID_BACK, -AVOID_BACK);
            cmd_linear = -0.1f;
            cmd_angular = 0;
        } else {
            motors_wheels(AVOID_FORWARD, AVOID_FORWARD);
            cmd_linear = 0.1f;
            cmd_angular = 0;
        }
        break;
    case AVOID_BACKING:
        if (time_reached(avoid_until)) {
            avoid_state = AVOID_TURNING;
            avoid_until = make_timeout_time_ms(AVOID_TURN_MS);
            motors_wheels(AVOID_TURN, -AVOID_TURN);    // spin right, like Car_Right
            cmd_linear = 0;
            cmd_angular = -1.0f;
        }
        break;
    case AVOID_TURNING:
        if (time_reached(avoid_until)) {
            avoid_state = AVOID_GO;    // the next step re-checks the distance
        }
        break;
    }
}

static void track_drive(float left, float right, int turn)
{
    motors_wheels(left, right);
    cmd_linear = 0.1f;
    cmd_angular = turn;
    track_last_turn = turn;
}

static void track_step(void)
{
    if (too_close) {
        stop_and_brake();
        return;
    }
    uint8_t b[4];
    line_read(b);
    bool any = b[0] || b[1] || b[2] || b[3];
    if (any) {
        track_last_seen = get_absolute_time();
    }
    if ((b[1] && b[2]) || (b[0] && b[3])) {
        track_drive(TRACK_FAST, TRACK_FAST, 0);            // centered, or a crossing
    } else if (b[0]) {
        track_drive(-TRACK_REVERSE, TRACK_FAST, 1);        // sharp left
    } else if (b[3]) {
        track_drive(TRACK_FAST, -TRACK_REVERSE, -1);       // sharp right
    } else if (b[1]) {
        track_drive(TRACK_SLOW, TRACK_FAST, 1);            // gentle left
    } else if (b[2]) {
        track_drive(TRACK_FAST, TRACK_SLOW, -1);           // gentle right
    } else if (absolute_time_diff_us(track_last_seen, get_absolute_time()) < TRACK_SEARCH_MS * 1000) {
        // Line lost: keep going the way it was last seen
        if (track_last_turn > 0) {
            motors_wheels(-TRACK_REVERSE, TRACK_FAST);
        } else if (track_last_turn < 0) {
            motors_wheels(TRACK_FAST, -TRACK_REVERSE);
        } else {
            motors_wheels(TRACK_FAST, TRACK_FAST);
        }
    } else {
        stop_and_brake();
    }
}

// No light -> stop; one side clearly brighter -> only the wheel on the darker
// side drives, so the car turns toward the light; otherwise straight.
// cmd_linear / cmd_angular are set only so the automatic lights follow along.
static void follow_step(void)
{
    if (too_close) {
        stop_and_brake();
        return;
    }
    float right = 65535 - adc_read_u16(LIGHT1_ADC_PIN);
    float left = 65535 - adc_read_u16(LIGHT2_ADC_PIN);
    float brightest = left > right ? left : right;
    follow_has_light = brightest > (follow_has_light ? FOLLOW_LIGHT_OFF : FOLLOW_LIGHT_ON);
    if (!follow_has_light) {
        stop_and_brake();
    } else if (left > right * FOLLOW_TURN_RATIO) {
        motors_wheels(0, FOLLOW_DUTY);      // light on the left: right wheel only
        cmd_linear = 0.1f;
        cmd_angular = 1.0f;
    } else if (right > left * FOLLOW_TURN_RATIO) {
        motors_wheels(FOLLOW_DUTY, 0);      // light on the right: left wheel only
        cmd_linear = 0.1f;
        cmd_angular = -1.0f;
    } else {
        motors_wheels(FOLLOW_DUTY, FOLLOW_DUTY);
        cmd_linear = 0.1f;
        cmd_angular = 0;
    }
}

static void light_msg_init(void)
{
    light_msg.data.data = light_data;
    light_msg.data.capacity = 2;
    light_msg.data.size = 2;
    line_msg.data.data = line_data;
    line_msg.data.capacity = 4;
    line_msg.data.size = 4;
    mode_msg.data.data = mode_buf;
    mode_msg.data.capacity = sizeof(mode_buf);
    mode_msg.data.size = 0;
}

static void battery_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }
    battery_msg.data = battery_read_u16();
    rcl_ret_t rc = rcl_publish(&battery_pub, &battery_msg, NULL);
    int64_t now_ns = rmw_uros_epoch_nanos();
    temp_msg.header.stamp.sec = now_ns / 1000000000;
    temp_msg.header.stamp.nanosec = now_ns % 1000000000;
    temp_msg.temperature = chip_temp_c();
    rc = rcl_publish(&temp_pub, &temp_msg, NULL);
    (void)rc;
}

static const char *battery_level(uint16_t v)
{
    // Same thresholds as YAHBOOM's sensors.check_battery_level()
    if (v < 400) return "NONE";
    if (v < 20000) return "LOW";
    if (v < 25000) return "MED";
    return "FULL";
}

static void oled_update(enum agent_state state)
{
    char line[17];
    const char *ip = wifi_ip_str();
    ssd1306_clear();
    if (ip == NULL) {
        ssd1306_text(0, "WIFI CONNECTING");
    } else if (strlen(ip) <= 13) {
        snprintf(line, sizeof(line), "IP %s", ip);
        ssd1306_text(0, line);
    } else {
        ssd1306_text(0, ip);
    }
    uint16_t bat = battery_read_u16();
    snprintf(line, sizeof(line), "BAT %u %s", bat, battery_level(bat));
    ssd1306_text(1, line);
    ssd1306_text(2, too_close ? "!! TOO CLOSE !!" :
                    state != AGENT_CONNECTED ? "ROS WAIT AGENT" :
                    mode == MODE_LIGHT_FOLLOW ? "ROS OK  FOLLOW" :
                    mode == MODE_LINE_TRACK ? "ROS OK  TRACK" :
                    mode == MODE_AVOID ? "ROS OK  AVOID" : "ROS OK");
    // Reuse the range timer's reading while connected, so two pings never overlap
    float d = state == AGENT_CONNECTED ? last_range_m : ultrasonic_read_m();
    if (state != AGENT_CONNECTED) {
        last_range_m = d;
        warning_update(d);
    }
    if (d < 0 || d > range_msg.max_range) {
        snprintf(line, sizeof(line), "DIST ---");
    } else {
        int mm = (int)(d * 1000.0f + 0.5f);
        snprintf(line, sizeof(line), "DIST %d.%d cm", mm / 10, mm % 10);
    }
    ssd1306_text(3, line);
    ssd1306_invert(too_close);
    ssd1306_show();
}

static void leds_cb(const void *msgin)
{
    const std_msgs__msg__UInt32MultiArray *m = msgin;
    printf("leds: %u values\n", (unsigned)m->data.size);
    auto_lights = false;
    ws2812_set_rgb(m->data.data, m->data.size);
}

static void auto_lights_cb(const void *msgin)
{
    const std_msgs__msg__Bool *m = msgin;
    printf("auto_lights: %d\n", m->data);
    auto_lights = m->data;
    if (!auto_lights) {
        ws2812_set_rgb(NULL, 0);
    }
}

static void paint(uint32_t *rgb, const uint8_t *idx, size_t n, uint32_t color)
{
    for (size_t i = 0; i < n; i++) {
        rgb[idx[i]] = color;
    }
}

static void lights_update(void)
{
    static uint32_t shown[WS2812_NUM_LEDS];
    static bool shown_valid;
    if (!auto_lights) {
        shown_valid = false;
        return;
    }
    uint32_t rgb[WS2812_NUM_LEDS] = {0};
    if (too_close || mode == MODE_AVOID) {
        // Avoid mode shows the distance like YAHBOOM: green far, yellow mid, red near
        uint32_t c = COLOR_WARN;
        if (!too_close && mode == MODE_AVOID) {
            c = (!range_valid(last_range_m) || last_range_m >= AVOID_MID_M) ? COLOR_AVOID_FAR : COLOR_AVOID_MID;
        }
        for (int i = 0; i < WS2812_NUM_LEDS; i++) {
            rgb[i] = c;
        }
    } else if (cmd_linear > 0.01f) {
        paint(rgb, GROUP(LEDS_FRONT), COLOR_HEAD);
    } else if (cmd_linear < -0.01f) {
        paint(rgb, GROUP(LEDS_REAR), COLOR_REVERSE);
    } else if (!time_reached(brake_until)) {
        paint(rgb, GROUP(LEDS_REAR), COLOR_BRAKE);
    }
    bool blink_on = (to_ms_since_boot(get_absolute_time()) / BLINK_HALF_MS) % 2 == 0;
    if (too_close || mode == MODE_AVOID) {
        // warning / avoid colors override turn signals
    } else if (cmd_angular > 0.1f) {
        paint(rgb, GROUP(LEDS_LEFT), blink_on ? COLOR_AMBER : 0);
    } else if (cmd_angular < -0.1f) {
        paint(rgb, GROUP(LEDS_RIGHT), blink_on ? COLOR_AMBER : 0);
    }
    if (shown_valid && memcmp(rgb, shown, sizeof(rgb)) == 0) {
        return;
    }
    ws2812_set_rgb(rgb, WS2812_NUM_LEDS);
    memcpy(shown, rgb, sizeof(rgb));
    shown_valid = true;
}

static void buzzer_cb(const void *msgin)
{
    const std_msgs__msg__UInt16MultiArray *m = msgin;
    uint16_t freq = m->data.size > 0 ? m->data.data[0] : 0;
    uint16_t ms = m->data.size > 1 ? m->data.data[1] : 200;
    printf("buzzer: %u Hz, %u ms\n", freq, ms);
    buzzer_beep(freq, ms);
}

static void servo_cb(const void *msgin)
{
    const std_msgs__msg__UInt16MultiArray *m = msgin;
    printf("servo: %u values\n", (unsigned)m->data.size);
    for (size_t i = 0; i < m->data.size && i < SERVO_COUNT; i++) {
        servo_set(i, m->data.data[i]);
    }
}

static void servo_msg_init(void)
{
    servo_msg.data.data = servo_data;
    servo_msg.data.capacity = SERVO_COUNT;
    servo_msg.data.size = 0;
    for (int i = 0; i < 2; i++) {
        servo_dim[i].label.data = servo_dim_label[i];
        servo_dim[i].label.capacity = sizeof(servo_dim_label[i]);
        servo_dim[i].label.size = 0;
    }
    servo_msg.layout.dim.data = servo_dim;
    servo_msg.layout.dim.capacity = 2;
    servo_msg.layout.dim.size = 0;
}

static void gps_msg_init(void)
{
    gps_msg.header.frame_id.data = gps_frame_id;
    gps_msg.header.frame_id.size = sizeof(gps_frame_id) - 1;
    gps_msg.header.frame_id.capacity = sizeof(gps_frame_id);
    gps_msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;
}

// Publish every GGA (1 Hz on a NEO-6M), also without a fix so it is visible
// that the module is talking: status -1 = no fix yet
static void gps_publish(const gps_fix_t *f)
{
    int64_t now_ns = rmw_uros_epoch_nanos();
    gps_msg.header.stamp.sec = now_ns / 1000000000;
    gps_msg.header.stamp.nanosec = now_ns % 1000000000;
    gps_msg.status.status = f->quality == 0 ? sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX :
                            f->quality == 2 ? sensor_msgs__msg__NavSatStatus__STATUS_SBAS_FIX :
                                              sensor_msgs__msg__NavSatStatus__STATUS_FIX;
    gps_msg.latitude = f->quality ? f->latitude : NAN;
    gps_msg.longitude = f->quality ? f->longitude : NAN;
    gps_msg.altitude = f->quality ? f->altitude : NAN;
    memset(gps_msg.position_covariance, 0, sizeof(gps_msg.position_covariance));
    if (f->quality && f->hdop > 0) {
        // Rough estimate: ~5 m horizontal error per unit of HDOP, vertical twice that
        double h = (double)f->hdop * 5.0;
        gps_msg.position_covariance[0] = h * h;
        gps_msg.position_covariance[4] = h * h;
        gps_msg.position_covariance[8] = 4.0 * h * h;
        gps_msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_APPROXIMATED;
    } else {
        gps_msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
    }
    rcl_ret_t rc = rcl_publish(&gps_pub, &gps_msg, NULL);
    (void)rc;
}

// Raw sentences on /car2/gps_nmea, to see satellites in view (GSV) etc.
static void gps_sentence(const char *sentence)
{
    if (!nmea_ready) {
        return;
    }
    size_t n = strlen(sentence);
    if (n >= sizeof(nmea_buf)) {
        n = sizeof(nmea_buf) - 1;
    }
    memcpy(nmea_buf, sentence, n);
    nmea_buf[n] = '\0';
    nmea_msg.data.data = nmea_buf;
    nmea_msg.data.size = n;
    nmea_msg.data.capacity = sizeof(nmea_buf);
    rcl_ret_t rc = rcl_publish(&nmea_pub, &nmea_msg, NULL);
    (void)rc;
}

static void buzzer_msg_init(void)
{
    buzzer_msg.data.data = buzzer_data;
    buzzer_msg.data.capacity = 2;
    buzzer_msg.data.size = 0;
    for (int i = 0; i < 2; i++) {
        buzzer_dim[i].label.data = buzzer_dim_label[i];
        buzzer_dim[i].label.capacity = sizeof(buzzer_dim_label[i]);
        buzzer_dim[i].label.size = 0;
    }
    buzzer_msg.layout.dim.data = buzzer_dim;
    buzzer_msg.layout.dim.capacity = 2;
    buzzer_msg.layout.dim.size = 0;
}

static void leds_msg_init(void)
{
    leds_msg.data.data = leds_data;
    leds_msg.data.capacity = WS2812_NUM_LEDS;
    leds_msg.data.size = 0;
    for (int i = 0; i < 2; i++) {
        leds_dim[i].label.data = leds_dim_label[i];
        leds_dim[i].label.capacity = sizeof(leds_dim_label[i]);
        leds_dim[i].label.size = 0;
    }
    leds_msg.layout.dim.data = leds_dim;
    leds_msg.layout.dim.capacity = 2;
    leds_msg.layout.dim.size = 0;
}

// Step that failed in create_entities(), for the debug print
#define TRY(x) do { \
        if ((x) != RCL_RET_OK) { \
            printf("create entities: line %d failed\n", __LINE__); \
            return false; \
        } \
    } while (0)

static bool create_entities(void)
{
    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    TRY(rcl_init_options_init(&init_options, allocator));
    rcl_ret_t init_rc = rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
    if (init_rc == RCL_RET_OK) {
        init_rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    }
    // rclc copies the options: free them on every attempt, or each failed
    // reconnect leaks memory until "PANIC Out of memory"
    rcl_ret_t fini_rc = rcl_init_options_fini(&init_options);
    (void)fini_rc;
    TRY(init_rc);
    TRY(rclc_node_init_default(&node, "car2", "car2", &support));
    TRY(rclc_subscription_init_default(&sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    TRY(rclc_subscription_init_default(&leds_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32MultiArray), "leds"));
    TRY(rclc_subscription_init_default(&auto_lights_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "auto_lights"));
    TRY(rclc_subscription_init_default(&mode_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "mode"));
    TRY(rclc_subscription_init_default(&buzzer_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray), "buzzer"));
    TRY(rclc_subscription_init_default(&servo_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray), "servo"));
    TRY(rclc_publisher_init_default(&ir_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "ir"));
    TRY(rclc_publisher_init_best_effort(&gps_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "gps"));
    TRY(rclc_publisher_init_best_effort(&nmea_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "gps_nmea"));
    TRY(rclc_publisher_init_best_effort(&line_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), "line"));
    TRY(rclc_publisher_init_best_effort(&light_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray), "light"));
    TRY(rclc_publisher_init_best_effort(&sound_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "sound"));
    TRY(rclc_timer_init_default(&light_timer, &support, RCL_MS_TO_NS(LIGHT_PERIOD_MS),
            light_timer_cb));
    TRY(rclc_publisher_init_best_effort(&range_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "range"));
    TRY(rclc_timer_init_default(&range_timer, &support, RCL_MS_TO_NS(RANGE_PERIOD_MS),
            range_timer_cb));
    TRY(rclc_publisher_init_best_effort(&battery_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "battery"));
    TRY(rclc_publisher_init_best_effort(&temp_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "temperature"));
    TRY(rclc_timer_init_default(&battery_timer, &support, RCL_MS_TO_NS(BATTERY_PERIOD_MS),
            battery_timer_cb));
    executor = rclc_executor_get_zero_initialized_executor();
    TRY(rclc_executor_init(&executor, &support.context, 9, &allocator));
    TRY(rclc_executor_add_subscription(&executor, &sub, &twist, &cmd_vel_cb, ON_NEW_DATA));
    TRY(rclc_executor_add_subscription(&executor, &leds_sub, &leds_msg, &leds_cb, ON_NEW_DATA));
    TRY(rclc_executor_add_subscription(&executor, &auto_lights_sub, &auto_lights_msg, &auto_lights_cb, ON_NEW_DATA));
    TRY(rclc_executor_add_timer(&executor, &range_timer));
    TRY(rclc_executor_add_timer(&executor, &battery_timer));
    TRY(rclc_executor_add_subscription(&executor, &mode_sub, &mode_msg, &mode_cb, ON_NEW_DATA));
    TRY(rclc_executor_add_timer(&executor, &light_timer));
    TRY(rclc_executor_add_subscription(&executor, &buzzer_sub, &buzzer_msg, &buzzer_cb, ON_NEW_DATA));
    TRY(rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_cb, ON_NEW_DATA));
    // Stamp sensor data with the agent's clock
    rmw_ret_t sync = rmw_uros_sync_session(1000);
    (void)sync;
    return true;
}

static void destroy_entities(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // Best-effort cleanup: the agent may already be gone
    rcl_ret_t rc = rcl_subscription_fini(&sub, &node);
    rc = rcl_subscription_fini(&leds_sub, &node);
    rc = rcl_subscription_fini(&auto_lights_sub, &node);
    rc = rcl_publisher_fini(&range_pub, &node);
    rc = rcl_timer_fini(&range_timer);
    rc = rcl_publisher_fini(&battery_pub, &node);
    rc = rcl_publisher_fini(&temp_pub, &node);
    rc = rcl_timer_fini(&battery_timer);
    rc = rcl_subscription_fini(&mode_sub, &node);
    rc = rcl_publisher_fini(&line_pub, &node);
    rc = rcl_publisher_fini(&light_pub, &node);
    rc = rcl_publisher_fini(&sound_pub, &node);
    rc = rcl_timer_fini(&light_timer);
    rc = rcl_subscription_fini(&buzzer_sub, &node);
    rc = rcl_subscription_fini(&servo_sub, &node);
    rc = rcl_publisher_fini(&ir_pub, &node);
    rc = rcl_publisher_fini(&gps_pub, &node);
    rc = rcl_publisher_fini(&nmea_pub, &node);
    rc = rclc_executor_fini(&executor);
    rc = rcl_node_fini(&node);
    rc = rclc_support_fini(&support);
    (void)rc;
}

int main(void)
{
    stdio_init_all();
    motors_init();
    ultrasonic_init();
    adc_init();
    adc_gpio_init(BATTERY_ADC_PIN);
    adc_gpio_init(LIGHT1_ADC_PIN);
    adc_gpio_init(LIGHT2_ADC_PIN);
    for (int i = 0; i < 4; i++) {
        gpio_init(LINE_PINS[i]);
        gpio_set_dir(LINE_PINS[i], GPIO_IN);
    }
    if (!ssd1306_init()) {
        printf("oled init failed\n");
    }
    oled_update(WAITING_AGENT);
    range_msg_init();
    leds_msg_init();
    light_msg_init();
    buzzer_msg_init();
    buzzer_init();
    servo_msg_init();
    servo_init();
    sound_init();
    gps_msg_init();
    temp_msg.header.frame_id.data = temp_frame_id;
    temp_msg.header.frame_id.size = sizeof(temp_frame_id) - 1;
    temp_msg.header.frame_id.capacity = sizeof(temp_frame_id);
    temp_msg.variance = 4.0;    // datasheet: roughly +-2 C after calibration, more without
    adc_set_temp_sensor_enabled(true);
    gps_init();
    sbus_init();
    gps_set_sentence_cb(gps_sentence);
    ir_init();

    rmw_uros_set_custom_transport(
        false,  // UDP is packet oriented: no stream framing
        NULL,
        pico_udp_transport_open,
        pico_udp_transport_close,
        pico_udp_transport_write,
        pico_udp_transport_read);

    while (!wifi_connect()) {
        sleep_ms(2000);
    }
    // After cyw43 init, so the WiFi driver has already claimed its PIO
    if (!ws2812_init()) {
        printf("ws2812 init failed: no free PIO state machine\n");
    }
    ble_v7rc_init();

    enum agent_state state = WAITING_AGENT;
    absolute_time_t retry_at = get_absolute_time();
    int missed_pings = 0;
    absolute_time_t next_ping = get_absolute_time();
    absolute_time_t next_oled = get_absolute_time();
    absolute_time_t next_lights = get_absolute_time();
    absolute_time_t next_follow = get_absolute_time();
    absolute_time_t next_track = get_absolute_time();
    absolute_time_t next_avoid = get_absolute_time();
    last_cmd_time = get_absolute_time();

    while (true) {
        switch (state) {
        case WAITING_AGENT:
            if (!time_reached(retry_at)) {
                break;
            }
            if (rmw_uros_ping_agent(500, 1) == RMW_RET_OK) {
                state = AGENT_AVAILABLE;
            } else {
                wifi_connect();
                sleep_ms(500);
            }
            break;
        case AGENT_AVAILABLE:
            if (create_entities()) {
                printf("agent connected\n");
                state = AGENT_CONNECTED;
                missed_pings = 0;
                nmea_ready = true;
            } else {
                destroy_entities();
                state = WAITING_AGENT;
                // Let the agent drop the half-made session first (without
                // blocking the loop, so V7RC driving and its timeout keep working)
                retry_at = make_timeout_time_ms(1000);
            }
            break;
        case AGENT_CONNECTED:
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
            if (absolute_time_diff_us(next_ping, get_absolute_time()) >= 0) {
                next_ping = make_timeout_time_ms(1000);
                // Busy BLE traffic (V7RC sticks) shares the radio with WiFi and
                // can delay a ping: only reconnect after 3 misses in a row,
                // since every reconnect leaves a stale node in the agent
                if (rmw_uros_ping_agent(200, 3) != RMW_RET_OK) {
                    if (++missed_pings >= 3) {
                        state = AGENT_DISCONNECTED;
                    } else {
                        printf("agent ping missed (%d)\n", missed_pings);
                    }
                } else {
                    missed_pings = 0;
                }
            }
            break;
        case AGENT_DISCONNECTED:
            nmea_ready = false;
            printf("agent lost, reconnecting\n");
            mode = MODE_MANUAL;
            motors_drive(0, 0);
            cmd_linear = 0;
            cmd_angular = 0;
            destroy_entities();
            state = WAITING_AGENT;
            break;
        }

        buzzer_update();
        v7rc_update();
        gps_fix_t fix;
        if (gps_poll(&fix)) {
            printf("gps: quality %d, %d sats, hdop %.1f, %.6f %.6f, %.1f m (%lu ok, %lu bad)\n",
                   fix.quality, fix.satellites, fix.hdop, fix.latitude, fix.longitude, fix.altitude,
                   (unsigned long)gps_sentences(), (unsigned long)gps_bad_checksums());
            if (state == AGENT_CONNECTED) {
                gps_publish(&fix);
            }
        }
        static absolute_time_t next_ble_log;
        if (time_reached(next_ble_log)) {
            next_ble_log = make_timeout_time_ms(5000);
            printf("ble: hci state %d, %s\n", ble_v7rc_state(),
                   ble_v7rc_connected() ? "connected" : "not connected");
            printf("gps: %lu bytes, %lu sentences ok, %lu bad\n", (unsigned long)gps_rx_bytes(),
                   (unsigned long)gps_sentences(), (unsigned long)gps_bad_checksums());
            struct mallinfo mi = mallinfo();
            printf("heap: %d bytes used, %d free in arena\n", mi.uordblks, mi.fordblks);
            printf("sbus: %lu frames, %lu errors\n", (unsigned long)sbus_frames(),
                   (unsigned long)sbus_errors());
        }
        ir_drive_update();
        sbus_update();
        uint8_t ir_cmd;
        if (ir_get(&ir_cmd)) {
            printf("ir: 0x%02x\n", ir_cmd);
            ir_drive_key(ir_cmd);
            if (state == AGENT_CONNECTED) {
                ir_msg.data = ir_cmd;
                rcl_ret_t rc = rcl_publish(&ir_pub, &ir_msg, NULL);
                (void)rc;
            }
        }

        if (time_reached(next_oled)) {
            next_oled = make_timeout_time_ms(OLED_PERIOD_MS);
            oled_update(state);
        }

        // Safety stop when commands stop arriving
        if (moving && absolute_time_diff_us(last_cmd_time, get_absolute_time()) > CMD_TIMEOUT_MS * 1000) {
            motors_drive(0, 0);
            moving = false;
            if (fabsf(cmd_linear) > 0.01f || fabsf(cmd_angular) > 0.1f) {
                brake_until = make_timeout_time_ms(BRAKE_MS);
            }
            cmd_linear = 0;
            cmd_angular = 0;
        }

        if (mode == MODE_LIGHT_FOLLOW && time_reached(next_follow)) {
            next_follow = make_timeout_time_ms(FOLLOW_PERIOD_MS);
            follow_step();
        }
        if (mode == MODE_LINE_TRACK && time_reached(next_track)) {
            next_track = make_timeout_time_ms(TRACK_PERIOD_MS);
            track_step();
        }
        if (mode == MODE_AVOID && time_reached(next_avoid)) {
            next_avoid = make_timeout_time_ms(AVOID_PERIOD_MS);
            avoid_step();
        }

        if (time_reached(next_lights)) {
            next_lights = make_timeout_time_ms(LIGHTS_PERIOD_MS);
            if (oled_dirty) {
                oled_dirty = false;
                next_oled = get_absolute_time();
            }
            lights_update();
        }
    }
}
