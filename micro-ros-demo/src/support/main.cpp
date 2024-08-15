// Main file to configure RCL Executor and run a configured node with attached handlers

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <support/helper.hpp>
#include <config/config.hpp>

#define EXECUTOR_SPIN_PERIOD_MS 100

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This code is only avaliable for Arduino framework with serial transport.
#endif

rclc_executor_t executor;
rclc_support_t support; rcl_allocator_t allocator;
rcl_node_t node;
bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

bool create_entities() {
    // initialize rcl, create init_options
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Initialize node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    init_all_handlers(support, node);

    // Initialize executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, HANDLER_COUNT, &allocator));

    // Attach node to executor
    attach_all_to_executor(executor);

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);
  RCCHECK(rcl_node_fini(&node));
  rclc_support_fini(&support);
}

/**
 * @brief runs once at the start of execution, sets up ROS Client Library and nodes
 */
void setup() {
    // Configure serial transport
    ID_Serial.begin(115200);
    set_microros_serial_transports(ID_Serial);

    state = WAITING_AGENT;
}

/**
 * @brief runs every tick, spins the executor
 */
void loop() {
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroy_entities();
            };
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_SPIN_PERIOD_MS));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}