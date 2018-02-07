# clock-message-board
An IoT clock that will also display messages posted to an MQTT topic. Uses an ESP8266, MAX7219 LED display matrix modules, and the MD_MAX72xx and MD_PAROLA libraries, as well as WS2812B LED strip. All code and related files are copyright 2018, James Petersen, published under an MIT license. See [https://github.com/jptrsn/clock-message-board/blob/master/LICENSE](https://github.com/jptrsn/clock-message-board/blob/master/LICENSE) for information about use, modification, and sharing restrictions. Generally, unless you want to make money from this project, you are free to use, modify or share anything found within. The project was originally designed to integrate well with [Home Assistant](http://www.homeassistant.io) and [Node Red](https://nodered.org/), but should work with any MQTT broker that supports JSON objects. If you do not have an MQTT broker configured, you may want to look at [CloudMQTT](https://www.cloudmqtt.com/) or [Adafruit IO](https://io.adafruit.com/) to get started without any additional hardware.

## Features
- Automatic time syncronization with NTP servers.
- Includes readings from HTU21D (SI7201) Temperature/Humidity sensor (attached through I2C)
- Over-The-Air Updates (OTA) *(board must have sufficient flash memory; tested on a WeMos D1 Mini)*
- Subscribe to an MQTT topic to display other relevant information using a JSON object (optional, it is not required)
- Auto-reconnect to WiFi and/or MQTT
- Status LEDs around frame using individually-addressable WS2812B strip

## Dependencies
Some basic libraries are required, which you can install through the library manager.
- Wire.h
- ESP8266WiFi.h
- SPI.h
- WiFiUdp.h

In addition to the above libraries, the code depends on some third-party libraries developed by these very nice people:
- MD_MAX72XX library by majicDesigns: [https://majicdesigns.github.io/MD_MAX72XX/]
- MD_Parola library by majicDesigns: [https://majicdesigns.github.io/MD_Parola/index.html]
- HTU21D library from enjoyneering: [https://github.com/enjoyneering/HTU21D]
- ~~SparkFun HTU21D library: [https://learn.sparkfun.com/tutorials/htu21d-humidity-sensor-hookup-guide/htu21d-library-and-functions]~~ *(deprecated as of v0.2)*
- ~~NTP Client library by Fabrice Weinberg: [https://github.com/arduino-libraries/NTPClient]~~ *(deprecated as of v0.2)*
- TimeLib library by Paul Stoffregen [https://github.com/PaulStoffregen/Time]
- Timezone library by Jack Christensen [https://github.com/JChristensen/Timezone]
- PubSub Client library by Nick O'Leary: [https://pubsubclient.knolleary.net/]
- ArduinoJson library by Benoit Blanchon: [https://bblanchon.github.io/ArduinoJson/]
- ArduinoOTA library by Ivan Grokhotkov and Miguel Angel Ajo [https://github.com/esp8266/Arduino/tree/master/libraries/ArduinoOTA]
- FastLED library for WS2812Bs: [http://fastled.io/]

## Setting it up
A sample configuration file has been included. This allows you to set default properties. As provided, the code will run on a **Wemos D1 Mini board** but it should be
compatible with any ESP8266 module that supports SPI. If you are using a different board, you may need to change the default values.

**By default, the sketch asks to include `config.h`, which is not included in this repository.** You may want to either create a new `config.h` file, or change the name of `example_config.h`.

` #define  DEBUG  0`
Allows you to turn on/off debugging output to serial monitor. Useful for developing over USB, but not useful when running in stand-alone mode. Potentially causes conflicts with SPI or WS2812 communication (depending on your configuration) - recommended to set to 0 unless actively developing the software.

## The Message object
The MQTT topic should contain a JSON object with a "message" property. The full structure of the JSON object is:
```
{
    "message": {{String}},      [required]
    "frameDelay": {{Integer}},  [optional]
    "repeat": {{Integer}},      [optional]
    "textEffect": {
        "in": {{Integer}},      [optional]
        "out": {{Integer}},     [optional]
        "pause": {{Integer}}    [optional]
    }
}
```

Additional parameters are available that allow you to customize a single message. Text effects, repeat, frame rate, and pause will be set for the message, then restored to defaults after the message is no longer displayed. To change defaults, use the `config.h` file.

### Customize a message
If you wish to change the way a message is rendered, add any of the optional properties to the JSON object that is published to your MQTT message topic.

#### frameDelay _(Integer)_
The number of milliseconds between each animation frame. This is useful if you want to slow down scrolling text.

#### repeat _(Integer)_
The number of times to repeat a given message. Can be useful, since animation will draw your eye to the clock, but generally not before part of the message has animated off screen (if it's a long message).

#### textEffect _(JSON object)_

##### textEffect.in _(Integer)_
The animation to use to transition the display from blank to showing the message.

##### textEffect.out _(Integer)_
The animation to use to transition the display from showing the message to a blank state.

##### textEffect.pause _(Integer)_
The number of milliseconds to wait while the display is showing the message. For scrolling messages, this is not required. Generally, long messages
should not define a pause, while shorter messages that can fit on the entire display at one time can be paused.

#### Text Effect Array
The text effect arguments are the index of the text effect defined in the array `effects`. See the list below for a quick reference, or look at the sketch code. Please note that if you modify this array, the resultant behaviour may not match your specified effects.

```
  PA_PRINT,             // 0
  PA_SCAN_HORIZ,        // 1
  PA_SCROLL_LEFT,       // 2
  PA_WIPE,              // 3
  PA_SCROLL_UP_LEFT,    // 4
  PA_SCROLL_UP,         // 5
  PA_FADE,              // 6
  PA_OPENING_CURSOR,    // 7
  PA_GROW_UP,           // 8
  PA_SCROLL_UP_RIGHT,   // 9
  PA_BLINDS,            // 10
  PA_CLOSING,           // 11
  PA_GROW_DOWN,         // 12
  PA_SCAN_VERT,         // 13
  PA_SCROLL_DOWN_LEFT,  // 14
  PA_WIPE_CURSOR,       // 15
  PA_DISSOLVE,          // 16
  PA_MESH,              // 17
  PA_OPENING,           // 18
  PA_CLOSING_CURSOR,    // 19
  PA_SCROLL_DOWN_RIGHT, // 20
  PA_SCROLL_RIGHT,      // 21
  PA_SLICE,             // 22
  PA_SCROLL_DOWN,       // 23
  ```

## OTA updates
The display will let you know when it begins the software update, display an animation as the update is applied, confirm the success of the update, then reset itself. This should allow zero-touch updates to be pushed to the device.

## WS2812B LED strip
There is an option to include a WS2812B strip of LEDs, designed to encompass the face of the message board. The code is an almost exact duplicate of the amazing [Bruh Automation](http://www.bruhautomation.com) and specifically his MQTT JSON lights project. [https://github.com/bruhautomation/ESP-MQTT-JSON-Digital-LEDs](https://github.com/bruhautomation/ESP-MQTT-JSON-Digital-LEDs).

The WS2812B LEDs attached to `WS_PIN` will behave the same as any other MQTT JSON light. You can follow the tutorial videos in the repository linked above to get a better understanding of how to use the light component in Home Assistant.

**Please note** that using a basic ESP8266 module with WS2812B LEDs can be difficult, and some pins will not work properly due to interrupts. An ESP07 has been tested, and GPIO 03 is confirmed to work, while GPIO 04, GPIO 05 and GPIO 12 are all confirmed to not work. If you are having difficulty getting the WS2812s to work, please ensure that you are using a supported pin. The software author will not provide any support for problems with the WS2812 LEDs.

## 3D Printed Case ##
A sample 3D printed case is included in the repository. It was created on [Autodesk Tinkercad](http://www.tinkercad.com), and can be printed directly from the included .stl file. Alternatively, you can [get the source file directly](https://www.tinkercad.com/things/1tm8isQnekp-clock-message-board-case) in order to remix or modify the case in any way. The 3D model is covered by a [Creative Commons CC-BY-NC-SA 3.0 License](https://creativecommons.org/licenses/by-nc-sa/3.0/legalcode)

![CC-BY-NC-SA 3.0](https://i.creativecommons.org/l/by-nc/4.0/88x31.png)

## Roadmap/Wishlist
- Schematic and PCB design for other ESP modules (ESP-07, ESP-12, or ESP-32)
- ~~Add support for RGB LED strips~~ (Added January 2018)
- Special character handling in message string
- Zones for displaying multiple metrics concurrently
- Use WiFi Manager library for handling WiFi credentials and connections
