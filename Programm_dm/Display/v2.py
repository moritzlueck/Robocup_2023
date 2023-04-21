from picographics import PicoGraphics, DISPLAY_TUFTY_2040
from pimoroni import Button
from machine import Pin, I2C
import time
import ustruct

display = PicoGraphics(display=DISPLAY_TUFTY_2040)
display.set_backlight(1)
i2c = I2C(0, scl = Pin(5), sda = Pin(4), freq = 100000)

i2c_adress = 0x20
data_size = 3

button_a = Button(7, invert=False)
button_b = Button(8, invert=False)
button_c = Button(9, invert=False)
button_up = Button(22, invert=False)
button_down = Button(6, invert=False)
button_boot = Button(23, invert=True)

white = display.create_pen(255, 255, 255)
checkpoint_color = display.create_pen(255, 255, 255)
victm_color = display.create_pen(255, 0, 0)
wall_color = display.create_pen(255, 255, 255)
background_color = display.create_pen(0, 0, 0)
black_tile_color = display.create_pen(0, 0, 0)
robot_color = display.create_pen(255,140,0)
text_color = display.create_pen(255, 255, 255)

arrow_up = [(0, 9), (9, 0), (10, 0), (19, 9), (12, 9), (12, 19), (7, 19), (7, 9)]
arrow_right = [(10, 0), (19, 9), (19, 10), (10, 19), (10, 12), (0, 12), (0, 7), (10, 7)]
arrow_down = [(19, 10), (10, 19), (9, 19), (0, 10), (6, 10), (6, 0), (12, 0), (12, 10)]
arrow_left = [(9, 19), (0, 10), (0, 9), (9, 0), (9, 6), (19, 6), (19, 12), (9, 12)]

#bitmask
wall_front_mask = 0b00000001
wall_right_mask = 0b00000010
wall_back_mask = 0b00000100
wall_left_mask = 0b00001000
victm_mask = 0b00010000
black_tile_mask = 0b00100000
checkpoint_mask = 0b01000000
visited_mask = 0b10000000

calibration_mask = 0b00000000
run_mask = 0b01000000
lop_mask = 0b10000000
end_mask = 0b11000000
status_mask = 0b11000000
direction_mask = 0b11000000
position_mask = 0b00111111

start_position = 12
tile_size = 26
tile_count = 9
wall_thickness = 2
frame_thickness = 2
gap_to_wall = 2
text_size = 3

status = "calibration"

map = [[0] * start_position * 2 for i in range(start_position * 2)]
data = [0,0,0]
direction = 0
position_x = 12
position_y = 12
map_value = 0

color = 255

def get_values():
    global data
    global direction
    global position_x
    global position_y
    global map
    global color
    global status
    
    #get data from mega and convert it fromt hex to decimal
    i2c_data = i2c.readfrom(i2c_adress, data_size)
    data = get_decimal_list(i2c_data)
    #print(data)
    
    #check if rund has started
    status = check_status()
    print(status)
    
    if status == "calibration":
        color = data[0]
        
    elif status == "run":
        position_x = (data[1] & position_mask)
        position_y = (data[2] & position_mask)
        direction = ((data[1] & direction_mask) >> 6)
        map[position_x][position_y] = data[0]
        
    elif status == "lop" or status == "end":
        #move robot by button press
        if (button_up.is_pressed):
            position_y += 1
            time.sleep(0.1)
        if (button_down.is_pressed):
            position_y -= 1
            time.sleep(0.1)
        if (button_b.is_pressed):
            position_x += 1
            time.sleep(0.1)
        if (button_a.is_pressed):
            position_x -= 1
            time.sleep(0.1)
        if (button_c.is_pressed):
            position_x = (data[1] & position_mask)
            position_y = (data[2] & position_mask)
    
def get_decimal_list(hex_list):
    decimal_list = []
    for hex_value in hex_list:
        decimal_value = int(hex_value)
        decimal_list.append(decimal_value)
    return decimal_list
    
def draw_tile(position_x_input, position_y_input, value_input):   
    tile_start_x = frame_thickness + position_x_input * tile_size
    tile_start_y = frame_thickness + position_y_input * tile_size
    third_tile = int((tile_size - wall_thickness * 2 - gap_to_wall * 2) / 3)
    
    display.set_pen(wall_color)
    if (value_input & wall_front_mask):
        display.rectangle(tile_start_x, tile_start_y, tile_size, wall_thickness)
    if (value_input & wall_right_mask):
        display.rectangle(tile_start_x + tile_size - wall_thickness, tile_start_y, wall_thickness, tile_size)
    if (value_input & wall_back_mask):
        display.rectangle(tile_start_x, tile_start_y + tile_size - wall_thickness, tile_size, wall_thickness)
    if (value_input & wall_left_mask):
        display.rectangle(tile_start_x, tile_start_y, wall_thickness, tile_size)
    
    display.set_pen(victm_color)
    if (value_input & victm_mask):
        display.rectangle(tile_start_x + wall_thickness + gap_to_wall, tile_start_y + wall_thickness + gap_to_wall, tile_size - wall_thickness * 2 - gap_to_wall * 2, tile_size - wall_thickness * 2 - gap_to_wall * 2)
        display.set_pen(background_color)
        display.rectangle(tile_start_x + wall_thickness + gap_to_wall, tile_start_y + wall_thickness + gap_to_wall, third_tile, third_tile)
        display.rectangle(tile_start_x + wall_thickness + gap_to_wall, tile_start_y + wall_thickness + gap_to_wall + third_tile * 2, third_tile, third_tile)
        display.rectangle(tile_start_x + wall_thickness + gap_to_wall + third_tile * 2, tile_start_y + wall_thickness + gap_to_wall, third_tile, third_tile)
        display.rectangle(tile_start_x + wall_thickness + gap_to_wall + third_tile * 2, tile_start_y + wall_thickness + gap_to_wall + third_tile * 2, third_tile, third_tile)
    
    display.set_pen(black_tile_color)
    if (value_input & black_tile_mask):
        display.rectangle(tile_start_x + wall_thickness + gap_to_wall, tile_start_y + wall_thickness + gap_to_wall, tile_size - wall_thickness * 2 - gap_to_wall * 2, tile_size - wall_thickness * 2 - gap_to_wall * 2)

    display.set_pen(checkpoint_color)
    if (value_input & checkpoint_mask):
        display.rectangle(tile_start_x + wall_thickness + gap_to_wall, tile_start_y + wall_thickness + gap_to_wall, tile_size - wall_thickness * 2 - gap_to_wall * 2, tile_size - wall_thickness * 2 - gap_to_wall * 2)

def draw_values():
    if status == "calibration":
        display.set_pen(text_color)
        display.text(str(color), 0, 0, 320, 10)
    elif status == "run" or status == "lop" or status == "end":
        start_x = 238
        start_y = 0
        display.set_pen(wall_color)
        if (direction == 0):
           display.polygon(get_rel_cords(scale(arrow_up, 3), start_x + 10, 0))
        elif (direction == 1):
           display.polygon(get_rel_cords(scale(arrow_right, 3), start_x + 10, 0))
        elif (direction == 2):
           display.polygon(get_rel_cords(scale(arrow_down, 3), start_x + 10, 0))
        elif (direction == 3):
           display.polygon(get_rel_cords(scale(arrow_left, 3), start_x + 10, 0))
        
        #display.set_font('bitmap6')
        #display.text(str(direction), 319, 0, 320, text_size)
        display.text(str(position_x), 238, 60, 320, text_size)
        display.text(str(position_y), 238, 80, 320, text_size)
        #display.text(str(map_value), 238, 60, 320, text_size)
    
def draw_robot():
    display.set_pen(robot_color)
    display.rectangle(int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2)), int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2)), tile_size - wall_thickness * 2, tile_size - wall_thickness * 2)
    display.set_pen(background_color)
    if (direction == 0):
        display.polygon(get_rel_cords(arrow_up, int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2) + 1), int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2) + 1)))
    elif (direction == 1):
        display.polygon(get_rel_cords(arrow_right, int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2) + 1), int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2) + 1)))
    elif (direction == 2):
        display.polygon(get_rel_cords(arrow_down, int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2) + 1), int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2) + 1)))
    elif (direction == 3):
        display.polygon(get_rel_cords(arrow_left, int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2) + 1), int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2) + 1)));
    
def draw_map():
    
    display.set_pen(background_color)
    display.clear()

    display.set_pen(wall_color)
    display.rectangle(0, 0, tile_size * tile_count + frame_thickness * 2, tile_size * tile_count + frame_thickness * 2)
    display.set_pen(background_color)
    display.rectangle(frame_thickness, frame_thickness, tile_size * tile_count, tile_size * tile_count)
    
    for i in range(tile_count):
        for j in range(tile_count):
            if ((position_x + i - 4) < len(map) and (position_y + (tile_count - 1 - j) - 4) < len(map)):
                draw_tile(i, j, map[position_x + i - 4][position_y + (tile_count - 1 - j) - 4])
    
    draw_robot()
    
def check_status():
    if (data[2] & status_mask) == calibration_mask:
        return "calibration"
    elif (data[2] & status_mask) == run_mask:
        return "run"
    elif (data[2] & status_mask) == lop_mask:
        return "lop"
    elif (data[2] & status_mask) == end_mask:
        return "end"

def get_rel_cords (list_input, value_x_input, value_y_input):
    return [(i[0] + value_x_input, i[1] + value_y_input) for i in list_input]

def scale (list_input, value_input):
    return [(i[0] * value_input, i[1] * value_input) for i in list_input]

def mirror (list_input):
    return [(19 - i[0], 19 - i[1]) for i in list_input]

while True:
    get_values()
    if status == "run" or status == "lop" or status == "end":
        draw_map()
    draw_values()
    display.update()