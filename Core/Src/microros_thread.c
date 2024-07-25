#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usart.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

static void microros_thread(void *argument);

osThreadId_t microros_thread_handler;
const osThreadAttr_t microros_thread_attributes = {
  .name = "microros_thread",
  .stack_size = 2000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void microros_thread_init(void)
{
    microros_thread_handler = osThreadNew(microros_thread, NULL, &microros_thread_attributes);
    if (microros_thread_handler == NULL)
    {
        Error_Handler();
    }
}

void microros_thread(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    UNUSED(argument);
    /* Infinite loop */
    // micro-ROS configuration

    rmw_uros_set_custom_transport(
      true,
      (void *) &huart3,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        // printf("Error on default allocators (line %d)\n", __LINE__);
    }

    // micro-ROS app

    rcl_publisher_t publisher;
    std_msgs__msg__Int32 msg;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    allocator = rcl_get_default_allocator();

    //create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node
    rclc_node_init_default(&node, "cubemx_node", "", &support);

    // create publisher
    rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "cubemx_publisher");

    msg.data = 0;

    for(;;)
    {
      rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
      if (ret != RCL_RET_OK)
      {
        // printf("Error publishing (line %d)\n", __LINE__);
      }

      msg.data++;
      osDelay(10);
    }
    /* USER CODE END StartDefaultTask */
}
