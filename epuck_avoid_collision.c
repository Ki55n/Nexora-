#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <math.h>
#include <stdbool.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

// Sensor thresholds (tuned for high sensitivity)
#define FRONT_OBSTACLE 85.0
#define SIDE_OBSTACLE 75.0
#define DEAD_END 90.0

int main(int argc, char **argv) {
  wb_robot_init();

  // Devices
  WbDeviceTag ps[8];
  const char *ps_names[8] = {
    "ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7"
  };

  for (int i = 0; i < 8; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  WbDeviceTag left_motor  = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  double left_speed, right_speed;

  while (wb_robot_step(TIME_STEP) != -1) {

    double v[8];
    for (int i = 0; i < 8; i++)
      v[i] = wb_distance_sensor_get_value(ps[i]);

    // Sensor grouping
    bool front =
      v[0] > FRONT_OBSTACLE ||
      v[7] > FRONT_OBSTACLE ||
      v[1] > FRONT_OBSTACLE;

    bool left_wall =
      v[5] > SIDE_OBSTACLE ||
      v[6] > SIDE_OBSTACLE;

    bool right_wall =
      v[2] > SIDE_OBSTACLE ||
      v[3] > SIDE_OBSTACLE;

    bool dead_end =
      v[0] > DEAD_END &&
      v[7] > DEAD_END &&
      v[6] > DEAD_END &&
      v[1] > DEAD_END;

    // ==========================
    // DEFAULT: FULL SPEED AHEAD
    // ==========================
    left_speed  = MAX_SPEED;
    right_speed = MAX_SPEED;

    // ==========================
    // DEAD END → RETRACE
    // ==========================
    if (dead_end) {
      // Reverse hard
      left_speed  = -0.8 * MAX_SPEED;
      right_speed = -0.8 * MAX_SPEED;
    }

    // ==========================
    // FRONT BLOCKED → TURN RIGHT
    // ==========================
    else if (front) {
      left_speed  =  0.6 * MAX_SPEED;
      right_speed = -0.6 * MAX_SPEED;
    }

    // ==========================
    // LEFT WALL FOLLOWING (maze-safe)
    // ==========================
    else if (left_wall && !right_wall) {
      // slight right correction
      left_speed  = MAX_SPEED;
      right_speed = 0.6 * MAX_SPEED;
    }

    else if (!left_wall && right_wall) {
      // drift left to find wall
      left_speed  = 0.6 * MAX_SPEED;
      right_speed = MAX_SPEED;
    }

    // ==========================
    // OPEN SPACE → FULL SPEED
    // ==========================
    else {
      left_speed  = MAX_SPEED;
      right_speed = MAX_SPEED;
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}

