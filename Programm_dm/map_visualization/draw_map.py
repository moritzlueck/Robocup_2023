from PIL import Image, ImageDraw, ImageFont

map_size = 24
start_position = 12
tile_size = 36
wall_thickness = 1
gap_to_wall = 2

wall = (0,0,0)
victm = (255,0,0)
checkpoint = (224, 224, 224)
black_tile = (0, 0, 0)
    
tile_colors = {
    0b00100000: black_tile,
    0b00010000: victm,
    0b01000000: checkpoint
}

wall_front_mask = 0b00000001
wall_right_mask = 0b00000010
wall_back_mask = 0b00000100
wall_left_mask = 0b00001000
victm_mask = 0b00010000
black_tile_mask = 0b00100000
checkpoint_mask = 0b01000000

def get_min_max(array_input):
    x_min = map_size
    x_max = 0
    y_min = map_size
    y_max = 0
    for i, row in enumerate(map):
        for z, value in enumerate(row):
            x = i
            y = z
            if value:
                if x < x_min:
                    x_min = x
                if x > x_max:
                    x_max = x
                if y < y_min:
                    y_min = y
                if y > y_max:
                    y_max = y
    return (x_min, x_max, y_min, y_max)

def get_map():
    map = [[0] * start_position * 2 for i in range(start_position * 2)]
    file = open(file_name)
    log = file.read()
    file.close()
    rows = log.split("\n")
    for value in rows:
        if not value == "":
            value = value.split("|")
            x = int(value[0])
            y = int(value[1])
            map_value = int(value[2])
            map[x][y] = map_value
    return map

def get_file_names():
    file_name = input("Name of log file: ")
    output_file_name = input("Name of output file: ") + ".png"
    return file_name, output_file_name

###################################################################################################
#-----------------------------------------MAIN PROGRAM--------------------------------------------#
###################################################################################################

file_name, output_file_name = get_file_names()

map = get_map()

x_min, x_max, y_min, y_max = get_min_max(map)
render_map_size_x = x_max - x_min + 1
render_map_size_y = y_max - y_min + 1
y_min = map_size - y_max
y_max = y_min + render_map_size_y - tile_size
    
img = Image.new('RGB', ((tile_size * render_map_size_x), (tile_size * render_map_size_y)), color = 'white')
draw = ImageDraw.Draw(img)
    
for i, row in enumerate(map):
     for z, value in enumerate(row):
        x = i
        y = map_size - z
        tile_start_x = (x - x_min) * tile_size
        tile_start_y = (y - y_min) * tile_size
        element_start_x = tile_start_x + wall_thickness + gap_to_wall
        element_start_y = tile_start_y + wall_thickness + gap_to_wall
        element_end_x = tile_start_x + tile_size - 1 - wall_thickness - gap_to_wall
        element_end_y = tile_start_y + tile_size - 1 - wall_thickness - gap_to_wall
        element_third = 10

        if value & wall_front_mask:  
            draw.line((tile_start_x, tile_start_y, tile_start_x + tile_size - 1, tile_start_y), fill = wall, width = wall_thickness)
        if value & wall_right_mask:
            draw.line((tile_start_x + tile_size - 1, tile_start_y, tile_start_x + tile_size - 1, tile_start_y + tile_size - 1), fill = wall, width = wall_thickness)
        if value & wall_back_mask:
            draw.line((tile_start_x, tile_start_y + tile_size - 1, tile_start_x + tile_size - 1, tile_start_y + tile_size - 1), fill = wall, width = wall_thickness)
        if value & wall_left_mask:
            draw.line((tile_start_x, tile_start_y, tile_start_x, tile_start_y + tile_size - 1), fill = wall, width = wall_thickness)

        if value & black_tile_mask:
            draw.rectangle((element_start_x, element_start_y, element_end_x, element_end_y), fill = black_tile)
            #draw.rectangle((tile_start_x, tile_start_y, tile_start_x + tile_size - 1, tile_start_y + tile_size - 1), fill = black_tile)
        if value & checkpoint_mask:
            draw.rectangle((element_start_x, element_start_y, element_end_x, element_end_y), fill = checkpoint)
        if value & victm_mask:
            draw.rectangle((element_start_x, element_start_y, element_end_x, element_end_y), fill = victm)
            draw.rectangle((element_start_x, element_start_y, element_start_x + element_third - 1, element_start_y + element_third - 1), fill = "white")
            draw.rectangle((element_start_x + element_third * 2, element_start_y, element_start_x + element_third * 3, element_start_y + element_third - 1), fill = "white")
            draw.rectangle((element_start_x + element_third * 2, element_start_y + element_third *2, element_start_x + element_third * 3, element_start_y + element_third * 3), fill = "white")
            draw.rectangle((element_start_x, element_start_y + element_third * 2, element_start_x + element_third - 1, element_start_y + element_third * 3), fill = "white")
img.save(output_file_name)
img.show()