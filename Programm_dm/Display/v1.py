from picographics import PicoGraphics, DISPLAY_TUFTY_2040
from pimoroni import Button
from machine import Pin, I2C
import time
import ustruct

i2c = I2C(0, scl = Pin(5), sda = Pin(4), freq = 100000)

i2c_adress = 0x20
data_size = 5
data = [0,0,0,0,0]

display = PicoGraphics(display=DISPLAY_TUFTY_2040)
display.set_backlight(0.5)

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

start_position = 12
tile_size = 26
tile_count = 9
wall_thickness = 2
frame_thickness = 2
gap_to_wall = 2
text_size = 3

run_started = 1

map = [[0] * start_position * 2 for i in range(start_position * 2)]
data = [0, 11, 13, 0, 0, 0, 0, 0, 0, 0]
direction = data[0]
position_x = data[1]
position_y = data[2]
map_value = data[3]
wall_relative = data[4]

#temp
#map[12][12] = 70
#map[12][13] = 26
#map[12][14] = 3
#map[11][12] = 20
#map[11][13] = 11
#map[11][14] = 5
#map[10][12] = 12
#map[10][13] = 74
#map[10][14] = 25

def get_values():
    global data
    #print("I2C Connection failed")
    i2c_data = i2c.readfrom(0x20, 5)
    data = get_data(i2c_data)
    print(data)
    global position_x
    global position_y
    run_started = check_run_start
    direction = data[0]
    position_x = data[1]
    position_y = data[2]
    map[position_x][position_y] = data[3]
    wall_relative = data[4]
    print(position_x)
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
        position_x = data[1]
        position_y = data[2]
    
def get_data(i2c_data):
    i2c_data = str(i2c_data)
    i2c_data = i2c_data.split("'")
    i2c_data = i2c_data[1].split("\\")
    counter = 0
    for dataaaaaaa in i2c_data:
        i2c_data[counter] = int("0"+dataaaaaaa, 0)
        counter += 1
    i2c_data.remove(0)    
    return i2c_data
    
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

def draw_values(run):
    if run == True:
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
        display.text(str(position_x), 238, 20, 320, text_size)
        display.text(str(position_y), 238, 40, 320, text_size)
        #display.text(str(map_value), 238, 60, 320, text_size)
        #display.text(str(wall_relative), 260, 80, 320, text_size)
    else:
        display.set_pen(robot_color)
        display.rectangle(110, 70, 100, 100)
    
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
    
    draw_values(run = True)
    
    display.set_pen(wall_color)
    display.rectangle(0, 0, tile_size * tile_count + frame_thickness * 2, tile_size * tile_count + frame_thickness * 2)
    display.set_pen(background_color)
    display.rectangle(frame_thickness, frame_thickness, tile_size * tile_count, tile_size * tile_count)
    
    for i in range(tile_count):
        for j in range(tile_count):
            if ((position_x + i - 4) < len(map) and (position_y + (tile_count - 1 - j) - 4) < len(map)):
                draw_tile(i, j, map[position_x + i - 4][position_y + (tile_count - 1 - j) - 4])
    
    draw_robot()
    
def check_run_start():
    if run_started:
        return True
    for val in data:
        if val != 255:
            return False
    return True

def get_rel_cords (list_input, value_x_input, value_y_input):
    return [(i[0] + value_x_input, i[1] + value_y_input) for i in list_input]

def scale (list_input, value_input):
    return [(i[0] * value_input, i[1] * value_input) for i in list_input]

def mirror (list_input):
    return [(19 - i[0], 19 - i[1]) for i in list_input]

while True:
    get_values()
    if (run_started):
        draw_map()
    display.update()

