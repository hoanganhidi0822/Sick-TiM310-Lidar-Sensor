import cv2

# Load the image in grayscale mode
image_path = 'D:/Documents/Researches/2024_Project/SICK_TIM_3xx-master/1339.png'
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Create a mask where pixels are white (255)
mask = (image == 255)

# Set white pixels to black (0)
image[mask] = 0

# Save the modified image
output_path = 'output_image.png'
cv2.imwrite(output_path, image)

print("Image processing complete. The output image has been saved.")
