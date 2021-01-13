
#include <Wire.h>

#define BUF_SIZE 80
uint8_t buf[BUF_SIZE];

void setup() {
  Serial.begin(115200);  // USB to UART
  Serial1.begin(115200); // Hardware UART
}

void redirect_ext_uart_to_usb_uart(void) {
  int n = Serial1.available();
  if (n) {
    n = Serial1.readBytes(buf, n < BUF_SIZE ? n : BUF_SIZE);
    Serial.write(buf, n);
  }
}

void redirect_usb_uart_to_ext_uart(void) {
  int n = Serial.available();
  if (n) {
    n = Serial.readBytes(buf, n < BUF_SIZE ? n : BUF_SIZE);
    Serial1.write(buf, n);
  }
}

void loop() {
  redirect_ext_uart_to_usb_uart();
  redirect_usb_uart_to_ext_uart();
}
