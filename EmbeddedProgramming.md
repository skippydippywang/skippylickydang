---
title: Embedded Programming
nav_order: 1
---

# Embedded Programming
{: .no_toc }

I was introduced to WOKWI which is an online electronic simulator. In WOKWI, I was able to porgram different microcontrollers with their repective programming languages to simulate circuits. For my first excersise I programmed an ESP32-C3 in C++ language. For the next 2 excersies, instead of using C++ to program, I used micropython to progam the ESP32-C3 & the RP2040. For Excersie 4 I was tasked to use Generative AI chat to create a program for the ESP32-C3 to read input values from a potentiometer and use it to control the intensity of an LED. For Excersie 5 I was tasked to use Generative AI chat to create a program for the Raspberry Pi Pico to control a neopixel ring display using a pushbutton.

For My Challenge exersice 1, I was tasked to develop a portable environmental monitoring device, comprising of an ESP32-C3 board with 128x64 OLED screen which will sample and display the ambient temperature and humidity using GEN AI to create the code and simulate it in WokWi.
The prompt that I use was "Can you create me a code for an ESP32-C3 board that uses a 128x64 OLED screen which will sample and display the ambient temperature and humidity and a DHT22 Sensor.".

### The code that I used:

```C++
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DHT22 Configuration
#define DHTPIN 2          // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22     // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);

  // Initialize DHT sensor
  dht.begin();

  // Initialize OLED display
  // Address 0x3C is common for 128x64 OLEDs
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();
  delay(2000);
}

void loop() {
  // Wait a few seconds between measurements
  delay(2000);

  float h = dht.readHumidity();
  float t = dht.readTemperature(); // Celsius

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Clear previous data
  display.clearDisplay();
  
  // Header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ENV MONITOR");
  display.drawLine(0, 12, 128, 12, SSD1306_WHITE);

  // Display Temperature
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("Temp: ");
  display.setTextSize(2);
  display.print(t, 1);
  display.cp437(true);
  display.write(167); // Degree symbol
  display.print("C");

  // Display Humidity
  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print("Hum:  ");
  display.setTextSize(2);
  display.print(h, 1);
  display.print("%");

  // Update screen
  display.display();
}
```


For My Challenge exersice 2, I was tasked to develop an automated sanitizer dispensing unit which comprises of a Pi Pico board, an ultrasonic sensor, a servo motor and an LED indicator. When the user places his hand(s) at a distance of < 3 cm from the ultrasonic sensor, the servo motor will be activated to dispense the sanitizer lotion. The LED will light up at the same time to indicate that sanitizer lotion is being dispensed. I then used GEN AI to create the code and simulate it in WokWi.
The prompt that I use was " Write me MicroPython code for a Pi Pico, When an HC-SR04 sensor detects an object closer than 3cm, rotate a servo motor and turn on an LED. Return servo to 0 and turn LED off after dispensing.".

### The code that I used:

```Python
import machine
import utime

# Pin Setup
trig = machine.Pin(15, machine.Pin.OUT)
echo = machine.Pin(14, machine.Pin.IN)
led = machine.Pin(13, machine.Pin.OUT)
servo = machine.PWM(machine.Pin(16))
servo.freq(50)

def set_servo_angle(angle):
    # Converts angle (0-180) to duty cycle for SG90
    duty = int((angle / 180 * 6500) + 1600)
    servo.duty_u16(duty)

def get_distance():
    trig.low()
    utime.sleep_us(2)
    trig.high()
    utime.sleep_us(10)
    trig.low()
    
    while echo.value() == 0:
        signaloff = utime.ticks_us()
    while echo.value() == 1:
        signalon = utime.ticks_us()
        
    timepassed = signalon - signaloff
    distance = (timepassed * 0.0343) / 2
    return distance

# Initialize: Servo at "closed" position
set_servo_angle(0)
led.value(0)

while True:
    dist = get_distance()
    print(f"Distance: {dist:.2f} cm")
    
    if dist < 3:
        print("Hand detected! Dispensing...")
        led.value(1)          # Turn on LED
        set_servo_angle(90)   # Press the pump
        utime.sleep(1)        # Wait for lotion to flow
        set_servo_angle(0)    # Release pump
        led.value(0)          # Turn off LED
        utime.sleep(2)        # Cooldown to prevent double-dispensing
        
    utime.sleep(0.1)

```











```yaml
collections:
  tests:
    permalink: "/:collection/:path/"
    output: true
  tutorials:
    permalink: "/:collection/:path/"
    output: true

just_the_docs:
  collections:
    tests:
      name: Tests
    tutorials:
      name: Tutorials
```

When *all* your pages are in a single collection, its name is not displayed.

The navigation for each collection is a separate name space for page titles: a page in one collection cannot be a child of a page in a different collection, or of a normal page.

