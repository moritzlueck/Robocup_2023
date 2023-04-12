from picographics import PicoGraphics, DISPLAY_TUFTY_2040
display = PicoGraphics(display=DISPLAY_TUFTY_2040)

display.set_backlight(0.5)

start_position = 12
tile_size = 26
tile_count = 9
wall_thickness = 2
frame_thickness = 2
gap_to_wall = 2

text_size = 3

map = my_array = [[0] * start_position * 2 for i in range(start_position * 2)]
data = [255, 255, 255, 255, 255, 255, 255, 255]

white = display.create_pen(255, 255, 255)
checkpoint_color = display.create_pen(255, 255, 255)
victm_color = display.create_pen(255, 0, 0)
wall_color = display.create_pen(255, 255, 255)
background_color = display.create_pen(0, 0, 0)
black_tile_color = display.create_pen(0, 0, 0)
robot_color = display.create_pen(255,140,0)

#bitmask
wall_front_mask = 0b00000001
wall_right_mask = 0b00000010
wall_back_mask = 0b00000100
wall_left_mask = 0b00001000
victm_mask = 0b00010000
black_tile_mask = 0b00100000
checkpoint_mask = 0b01000000
visited_mask = 0b10000000

data = [0, 11, 13, 0, 0]

direction = data[0]
position_x = data[1]
position_y = data[2]
map_value = data[3]
wall_relative = data[4]

#temp
map[12][12] = 70
map[12][13] = 26
map[12][14] = 3
map[11][12] = 20
map[11][13] = 11
map[11][14] = 5
map[10][12] = 12
map[10][13] = 74
map[10][14] = 32

def get_values():
    direction = data[0]
    position_x = data[1]
    position_y = data[2]
    #map[position_x][position_y] = data[3]
    wall_relative = data[4]
    
def draw_tile(position_x_input, position_y_input, value_input):
    #position_y_input = tile_count - position_y_input
        
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
    display.set_pen(wall_color)
    display.set_font('bitmap6')

    display.text(str(direction), 260, 0, 320, text_size)
    display.text(str(position_x), 260, 20, 320, text_size)
    display.text(str(position_y), 260, 40, 320, text_size)
    display.text(str(map_value), 260, 60, 320, text_size)
    display.text(str(wall_relative), 260, 80, 320, text_size)
    
def draw_robot():
    display.set_pen(robot_color)
    display.rectangle(int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2)), int(frame_thickness + wall_thickness + tile_size * ((tile_count - 1) / 2)), tile_size - wall_thickness * 2, tile_size - wall_thickness * 2)
    
def draw_map():
    display.set_pen(wall_color)
    display.rectangle(0, 0, tile_size * tile_count + frame_thickness * 2, tile_size * tile_count + frame_thickness * 2)
    display.set_pen(background_color)
    display.rectangle(frame_thickness, frame_thickness, tile_size * tile_count, tile_size * tile_count)
    
    for i in range(tile_count):
        for j in range(tile_count):
            draw_tile(i, j, map[position_x + i - 4][position_y + (tile_count - 1 - j) - 4])
    
    draw_robot()

while True:
    get_values()
    draw_map()
    draw_values()
    display.update()