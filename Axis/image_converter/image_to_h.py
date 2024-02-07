#!/usr/bin/env python

import os, sys
import argparse
from PIL import Image
from pathlib import Path

parser = argparse.ArgumentParser(description='Generate h file from image for SpinningLED')
parser.add_argument('image_file', action='store',
                    help='image filename')
parser.add_argument('h_file', action='store',
                    help='output h file')
parser.add_argument('-x', '--width', type=int,
                    help='image width')
parser.add_argument('-y', '--height', type=int,
                    help='image height')
parser.add_argument('-d', '--double-height', '--double-y', action='store_true',
                    help='doubles height from read value')
parser.add_argument('-F','--header',
                    help='header file to read width and height from')
parser.add_argument('--width-var', default="LEDS_X",
                    help='define variable containing image width in header file')
parser.add_argument('--height-var', default="LEDS_Y",
                    help='define variable containing image height in header file')
parser.add_argument('--png', action='store_true',
                    help='saves a png in addition')

args = parser.parse_args()

image_file = Path(args.image_file)
h_file = Path(args.h_file)

if args.double_height:
    args.height = 2 * args.height

# Open image and resize
im = Image.open(image_file)
im = im.convert("RGB")
im = im.resize((args.width, args.height))


# Convert color tuple to 16-bit RGB565
def rgb565(pixel):
    r = ((pixel[0] & 0xf8) << 8)
    g = ((pixel[1] & 0xfc) << 3)
    b = ((pixel[2] & 0xf8) >> 3)
    return r | g | b

# Save a stream
with h_file.open('w') as f:
    f.write("// Generated RGB565 image file generated from:\n")
    f.write(f"// {image_file.name}\n")
    f.write("//\n")
    f.write("// DO NOT MODIFY !!!\n")
    f.write("//\n")
    f.write(f"// Resizes for {args.width}x{args.height}\n")
    f.write("\n")
    f.write("#include <stdint.h>\n")
    f.write("\n")

    f.write(f"const uint16_t img_{image_file.stem}[] = {{\n")

    for x in range(args.width):
        f.write(f"    /*{x:4d}*/ ")
        for y in range(args.height):
            f.write(f"{rgb565(im.getpixel((x, y))):5d}")
            if x != args.width-1 or y != args.height-1:
                f.write(", ")
        f.write('\n')

    f.write("};\n")

# Do save the picture just in case
if (args.png):
    im.save(h_file.with_suffix(".png"))
