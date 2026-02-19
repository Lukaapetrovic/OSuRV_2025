**Official Documentation Resources**



**ESP-IDF WiFi Documentation:**



**Main WiFi Driver Guide:** 

*https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi.html*



**WiFi API Reference:** 

*https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp\_wifi.html*



**Official SoftAP Example:**

The Espressif team provides a complete working example that's very similar to what you're doing. You can find it on GitHub at: 

*https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting\_started/softAP*



This example is particularly valuable because it shows best practices for WiFi initialization, event handling, and configuration. Your code actually follows this pattern quite closely, which is excellent.



**Detailed Tutorial:**

There's a comprehensive tutorial on the Espressif Developer Portal that walks through creating a SoftAP step-by-step: 

*https://developer.espressif.com/blog/2025/04/soft-ap-tutorial/*



This tutorial is particularly good for understanding the event loop concept and why WiFi applications are structured the way they are.



**Additional Resources for Your Specific Needs**

For UDP socket programming on ESP32, you'll want to look at the lwIP documentation. lwIP is the lightweight IP stack that ESP-IDF uses for networking. The ESP-IDF provides examples in the *examples/protocols/sockets* directory.



**For FreeRTOS task management** (which you're using for your motor control and UDP tasks), the ESP-IDF documentation has a good section on FreeRTOS: *https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos\_idf.html*



**For LEDC (PWM control)**, which you're using for the motors: *https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html*

