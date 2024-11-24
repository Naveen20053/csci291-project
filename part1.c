#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/gps.h>
#include <webots/led.h>

#define SIM_STEP 64
#define MAX_VELOCITY 6.28
#define LIGHT_LIMIT 500.0
#define OBSTACLE_LIMIT 100.0
#define GPS_TOLERANCE 0.01

bool detect_dead_end();

int main(int argc, char **argv) {
  wb_robot_init();
  
  WbDeviceTag motor_left = wb_robot_get_device("left wheel motor");
  WbDeviceTag motor_right = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(motor_left, INFINITY);
  wb_motor_set_position(motor_right, INFINITY);
  
  double light_readings[11];
  double dead_end_positions[10][3];
  double motor_speed_left = MAX_VELOCITY;
  double motor_speed_right = MAX_VELOCITY;
  int dead_end_index = 0;
  double highest_light = 0;
  int brightest_index = 0;
  
  WbDeviceTag sensors[8];
  char sensor_name[50];
  for (int i = 0; i < 8; i++) {
    sprintf(sensor_name, "ps%d", i);
    sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[i], SIM_STEP);
  }
  
  WbDeviceTag light = wb_robot_get_device("ls0");
  wb_light_sensor_enable(light, SIM_STEP);

  WbDeviceTag gps_sensor = wb_robot_get_device("gps");
  wb_gps_enable(gps_sensor, SIM_STEP);

  int dead_end_counter = 0;
  double dead_end_timer = 0;

  bool detect_dead_end() {
    double front_distance = wb_distance_sensor_get_value(sensors[0]);
    double current_time = wb_robot_get_time();

    if (front_distance > 100) {
      if (dead_end_counter == 0 || (current_time - dead_end_timer) > 1.7) {
        dead_end_timer = current_time;
        dead_end_counter++;
      }
    }

    if (dead_end_counter >= 2) {
      dead_end_counter = 0;
      return true;
    }

    if ((current_time - dead_end_timer) > 10.0) {
      dead_end_counter = 0;
    }
    
    return false;
  }
  
  while (wb_robot_step(SIM_STEP) != -1) {
    const double *gps_coords = wb_gps_get_values(gps_sensor);

    bool wall_on_left = wb_distance_sensor_get_value(sensors[5]) > OBSTACLE_LIMIT;
    bool left_turn = wb_distance_sensor_get_value(sensors[6]) > OBSTACLE_LIMIT;
    bool wall_ahead = wb_distance_sensor_get_value(sensors[7]) > OBSTACLE_LIMIT;

    double current_light = wb_light_sensor_get_value(light);

    if (detect_dead_end()) {
      if (dead_end_index < 10) {
        dead_end_index++;
        light_readings[dead_end_index] = current_light;

        dead_end_positions[dead_end_index - 1][0] = gps_coords[0];
        dead_end_positions[dead_end_index - 1][1] = gps_coords[1];
        dead_end_positions[dead_end_index - 1][2] = gps_coords[2];

        printf("%d  ", dead_end_index);
        printf("Dead end detected\n");
        printf("Light sensor value: %f\n", light_readings[dead_end_index]);
        printf("Dead-end coordinates: x = %f, y = %f, z = %f\n", gps_coords[0], gps_coords[1], gps_coords[2]);
      }

      if (dead_end_index == 10) {
        for (int i = 1; i <= 10; i++) {
          if (light_readings[i] > highest_light) {
            highest_light = light_readings[i];
            brightest_index = i;
          }
        }
        printf("Maximum light intensity %f at dead end %d\n", highest_light, brightest_index);
        printf("Coordinates of the brightest dead end: x = %f, y = %f, z = %f\n\n",
               dead_end_positions[brightest_index - 1][0],
               dead_end_positions[brightest_index - 1][1],
               dead_end_positions[brightest_index - 1][2]);
        dead_end_index++;
      }

