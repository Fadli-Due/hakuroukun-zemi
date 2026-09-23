from PIL import Image, ImageDraw
import math

corners = [
    ("C1", 7.344, -3.044),
    ("C2", 7.280, 11.155),
    ("C3", 10.461, 10.865),
    ("C4", 12.999, 8.305),
    ("C5", 15.549, 10.144),
    ("C6", 27.998, 8.218),
    ("C7", 25.199, -8.435),
    ("C8", 11.900, -7.214),
]

resolution = 0.05   # meters per pixel
margin = 2.0         # meters of padding around the outline

xs = [c[1] for c in corners]
ys = [c[2] for c in corners]
xmin, xmax = min(xs), max(xs)
ymin, ymax = min(ys), max(ys)

origin_x = xmin - margin
origin_y = ymin - margin
world_w = (xmax - xmin) + 2 * margin
world_h = (ymax - ymin) + 2 * margin

width_px = int(math.ceil(world_w / resolution))
height_px = int(math.ceil(world_h / resolution))

print(width_px, height_px, origin_x, origin_y)

def world_to_px(x, y):
    px = (x - origin_x) / resolution
    py = height_px - 1 - (y - origin_y) / resolution
    return (px, py)

poly_px = [world_to_px(x, y) for _, x, y in corners]

img = Image.new("L", (width_px, height_px), color=205)  # unknown everywhere
draw = ImageDraw.Draw(img)

draw.polygon(poly_px, fill=254)          # interior = free space

closed = poly_px + [poly_px[0]]           # close the loop back to first corner
draw.line(closed, fill=0, width=3)        # boundary wall

img.save("/home/systemlab/hakuroukun-zemi/src/hakuroukun_boustrophedon_with_cones/map_draw_test/df_area_outline.pgm")

yaml_text = f"""image: df_area_outline.pgm
resolution: {resolution}
origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

with open("/home/systemlab/hakuroukun-zemi/src/hakuroukun_boustrophedon_with_cones/map_draw_test/df_area_outline.yaml", "w") as f:
    f.write(yaml_text)

print("Saved PGM and YAML to map_draw_test/")

preview = img.convert("RGB").resize((width_px * 2, height_px * 2), Image.NEAREST)
preview.save("/home/systemlab/hakuroukun-zemi/src/hakuroukun_boustrophedon_with_cones/map_draw_test/df_area_outline_preview.png")

print("Saved PNG preview too")