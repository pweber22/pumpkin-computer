# pumpkin-computer
Computer for dropping a pumpkin from a moving airplane to hit a target. Computer calculates when and where to drop, then guides the plane to that location and times the drop.\
Uses a Teensy 4.0 and a [PA1010D GPS breakout from Adafruit](https://learn.adafruit.com/adafruit-mini-gps-pa1010d-module).\
Display is a 32x128 ssd1306 OLED display from [Amazon](https://smile.amazon.com/dp/B08L7QW7SR?ref=ppx_yo2_dt_b_product_details&th=1).\
Schematic and potential pcb layout Eagle files are in the PumpkinSchematics directory, and the arduino code is in the PumpkinComputer directory.\
Uses Wire, Adafruit_GFX, Adafruit_SSD1306, and Adafruit_GPS Libraries.
