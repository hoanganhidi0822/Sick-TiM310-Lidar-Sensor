import cv2
import numpy as np

def combine_lines(lines, max_distance=10):
    """
    Combine closely aligned lines into a single continuous line.
    """
    if lines is None:
        return []

    combined_lines = []
    lines = sorted(lines, key=lambda line: (line[0][0], line[0][1]))  # Sort lines by their start point

    while lines:
        line = lines.pop(0)
        x1, y1, x2, y2 = line[0]
        combined = [x1, y1, x2, y2]
        to_remove = []
        for other_line in lines:
            ox1, oy1, ox2, oy2 = other_line[0]
            if (abs(ox1 - x2) < max_distance and abs(oy1 - y2) < max_distance) or \
               (abs(ox2 - x1) < max_distance and abs(oy2 - y1) < max_distance):
                combined = [min(x1, ox1), min(y1, oy1), max(x2, ox2), max(y2, oy2)]
                to_remove.append(other_line)
        for item in to_remove:
            lines.remove(item)
        combined_lines.append(combined)
    
    return combined_lines

# Load the image in grayscale mode
image_path = 'D:/Documents/Researches/2024_Project/SICK_TIM_3xx-master/output_image.png'
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Apply Canny edge detector to find edges in the image
edges = cv2.Canny(image, 50, 150)

# Use Hough Line Transform to detect lines in the edge-detected image
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

# Combine closely aligned lines into continuous lines
combined_lines = combine_lines(lines, max_distance=10)

# Convert grayscale image to color image to draw colored lines
color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

# Draw the combined lines on the image
for line in combined_lines:
    x1, y1, x2, y2 = line
    cv2.line(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw green lines

# Save the image with drawn lines
output_path = 'output_combined_lines.png'
cv2.imwrite(output_path, color_image)

print("Image processing complete. The output image with combined lines has been saved.")
