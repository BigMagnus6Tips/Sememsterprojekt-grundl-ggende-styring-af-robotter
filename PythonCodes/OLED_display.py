from machine import I2C, Pin
import PythonCodes.PF4.ssd1306_OLED as ssd1306  # Import the SSD1306 driver

# Initialize I2C with correct pins (SCL and SDA)
i2c = I2C(0, scl=Pin(13), sda=Pin(12))  # Replace with your correct pins

# Initialize the SSD1306 OLED display (128x64 resolution)
display = ssd1306.SSD1306_I2C(128, 64, i2c)

# Clear the display
display.fill(0)

# Draw shapes on the screen
display.fill_rect(0, 0, 32, 32, 1)  # Draw a filled square at the top-left corner
display.fill_rect(2, 2, 28, 28, 0)  # Draw a smaller square inside the first square
display.vline(9, 8, 22, 1)  # Draw a vertical line
display.vline(16, 2, 22, 1)  # Draw another vertical line
display.vline(23, 8, 22, 1)  # Draw a third vertical line
display.fill_rect(26, 24, 2, 4, 1)  # Draw a small rectangle at the bottom-right corner

# Display text on the screen
display.text('MicroPython', 40, 0, 1)  # Draw text "MicroPython" starting at (40, 0)
display.text('SSD1306', 40, 12, 1)  # Draw text "SSD1306" starting at (40, 12)
display.text('OLED 128x64', 40, 24, 1)  # Draw text "OLED 128x64" starting at (40, 24)

# Update the display to show the changes
display.show()

# Link til side hvordan den bruges: https://docs.micropython.org/en/latest/esp8266/tutorial/ssd1306.html
# Link til datasheet: https://m.eleparts.co.kr/data/goods_attach/202303/good-pdf-12538505-1.pdf

