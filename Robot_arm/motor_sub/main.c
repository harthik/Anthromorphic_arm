#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define BLINK_GPIO 2
#define M1_IN1_PIN 13
#define M1_IN2_PIN 12
#define M1_IN3_PIN 14
#define M1_IN4_PIN 27
#define M2_IN1_PIN 26
#define M2_IN2_PIN 25
#define M2_IN3_PIN 33
#define M2_IN4_PIN 32
#define M3_IN1_PIN 4
#define M3_IN2_PIN 16
#define M3_IN3_PIN 17
#define M3_IN4_PIN 5

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "ROS_NODE";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGW(TAG, "Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc);}}

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;

void rotateMotor(int degrees, int in1_pin, int in2_pin, int in3_pin, int in4_pin) {
    int stepSequence[4][4] = {
        {1, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 1},
        {1, 0, 0, 1}
    };

    int numSteps = abs(degrees * 2048) / 360;

    for (int i = 0; i < numSteps; i++) {
        int stepIndex;
        if (degrees < 0) {
            ESP_LOGI(TAG,"forward");
            stepIndex = i % 4;
        } else {
            ESP_LOGI(TAG,"reverse");
            stepIndex = (4 - (i % 4)) % 4;
        }
        gpio_set_level(in1_pin, stepSequence[stepIndex][0]);
        gpio_set_level(in2_pin, stepSequence[stepIndex][1]);
        gpio_set_level(in3_pin, stepSequence[stepIndex][2]);
        gpio_set_level(in4_pin, stepSequence[stepIndex][3]);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    gpio_set_level(in1_pin, 0);
    gpio_set_level(in2_pin, 0);
    gpio_set_level(in3_pin, 0);
    gpio_set_level(in4_pin, 0);
}

void rotateMotorTask1(void *arg) {
    int degrees = *(int *)arg;
    ESP_LOGI(TAG, "rotate motor a %d", degrees);
    rotateMotor(degrees, M1_IN1_PIN, M1_IN2_PIN, M1_IN3_PIN, M1_IN4_PIN);
    free(arg); // Free allocated memory
    vTaskDelete(NULL);
}

void rotateMotorTask2(void *arg) {
    int degrees = *(int *)arg;
    ESP_LOGI(TAG, "rotate motor b %d", degrees);
    rotateMotor(degrees, M2_IN1_PIN, M2_IN2_PIN, M2_IN3_PIN, M2_IN4_PIN);
    free(arg); // Free allocated memory
    vTaskDelete(NULL);
}

void rotateMotorTask3(void *arg) {
    int degrees = *(int *)arg;
    ESP_LOGI(TAG, "rotate motor c %d", degrees);
    rotateMotor(degrees, M3_IN1_PIN, M3_IN2_PIN, M3_IN3_PIN, M3_IN4_PIN);
    free(arg); // Free allocated memory
    vTaskDelete(NULL);
}

void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    if (msg->data.size >= 3) {
        int *degrees1 = malloc(sizeof(int));
        int *degrees2 = malloc(sizeof(int));
        int *degrees3 = malloc(sizeof(int));

        if (degrees1 == NULL || degrees2 == NULL || degrees3 == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed");
            free(degrees1);
            free(degrees2);
            free(degrees3);
            return;
        }

        *degrees1 = msg->data.data[0];
        *degrees2 = msg->data.data[1];
        *degrees3 = msg->data.data[2];

        xTaskCreate(rotateMotorTask1, "motor_task1", 2048, degrees1, 5, NULL);
        xTaskCreate(rotateMotorTask2, "motor_task2", 2048, degrees2, 5, NULL);
        xTaskCreate(rotateMotorTask3, "motor_task3", 2048, degrees3, 5, NULL);
    }
}

void micro_ros_task(void *arg) {
    ESP_LOGI(TAG, "micro_ros_task started");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodiscovery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    // RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "esp32", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "joint_angles"));

    // Allocate memory for the message
    msg.data.capacity = 12; // Change this value based on your needs
    msg.data.data = (int32_t *)malloc(msg.data.capacity * sizeof(int32_t));
    msg.data.size = 0;

    // Create executor.
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    unsigned int rcl_wait_timeout = 1000; // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    // Spin forever.
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // Free resources.
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    // Free allocated memory
    free(msg.data.data);

    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG, "app_main started");

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M1_IN1_PIN) | (1ULL << M1_IN2_PIN) |
                        (1ULL << M1_IN3_PIN) | (1ULL << M1_IN4_PIN) |
                        (1ULL << M2_IN1_PIN) | (1ULL << M2_IN2_PIN) |
                        (1ULL << M2_IN3_PIN) | (1ULL << M2_IN4_PIN) |
                        (1ULL << M3_IN1_PIN) | (1ULL << M3_IN2_PIN) |
                        (1ULL << M3_IN3_PIN) | (1ULL << M3_IN4_PIN)
    };
    gpio_config(&io_config);

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}