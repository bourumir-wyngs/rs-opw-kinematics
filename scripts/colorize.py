import csv
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np


def plot_color_distribution(image_path, code, output_csv_path="colors.csv"):
    """
    This function takes the path to a PNG image, extracts its color components,
    filters out fully white and fully black pixels, and plots the distribution
    of each color component (R, G, B). Additionally, it writes filtered pixel
    values to a CSV file as required.
    """
    # Load the image
    img = Image.open(image_path)
    img = img.convert("RGB")  # Convert the image to RGB format

    # Convert the image to a NumPy array for easier processing
    img_array = np.array(img)

    # Reshape the array into 2D where each row is [R, G, B]
    pixels = img_array.reshape(-1, 3)

    # Filter out fully white [255, 255, 255] and fully black [0, 0, 0] pixels
    pixels = pixels[(pixels != [255, 255, 255]).all(axis=1)]
    pixels = pixels[(pixels != [0, 0, 0]).all(axis=1)]

    # Split the pixels into separate R, G, and B components
    red_channel = pixels[:, 0]
    green_channel = pixels[:, 1]
    blue_channel = pixels[:, 2]

    # Write the data to CSV
    with open(output_csv_path, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Channel", "Red", "Green", "Blue"])  # Add a header
        for r, g, b in pixels:
           writer.writerow([code, r, g, b])  # Dominant red

    # Plot the distributions
    plt.figure(figsize=(12, 6))

    # Red channel
    plt.subplot(3, 1, 1)
    plt.hist(red_channel, range=(0, 256), bins=64, color='red', alpha=0.7, label="Red")
    plt.title("Red Channel Distribution")
    plt.xlabel("Intensity")
    plt.ylabel("Frequency")
    plt.legend()

    # Green channel
    plt.subplot(3, 1, 2)
    plt.hist(green_channel, range=(0, 256), bins=64, color='green', alpha=0.7, label="Green")
    plt.title("Green Channel Distribution")
    plt.xlabel("Intensity")
    plt.ylabel("Frequency")
    plt.legend()

    # Blue channel
    plt.subplot(3, 1, 3)
    plt.hist(blue_channel, range=(0, 256), bins=64, color='blue', alpha=0.7, label="Blue")
    plt.title("Blue Channel Distribution")
    plt.xlabel("Intensity")
    plt.ylabel("Frequency")
    plt.legend()

    # Adjust layout and show
    plt.tight_layout()
    plt.show(block=False)


# Call the function and generate the CSV and plots
plot_color_distribution("/home/audrius/opw/rs-opw-kinematics-private/color_samples/color_red.png", "r", output_csv_path="colors_red.csv")
plot_color_distribution("/home/audrius/opw/rs-opw-kinematics-private/color_samples/color_green.png", "g", output_csv_path="colors_green.csv")
plot_color_distribution("/home/audrius/opw/rs-opw-kinematics-private/color_samples/color_blue.png", "b", output_csv_path="colors_blue.csv")
plot_color_distribution("/home/audrius/opw/rs-opw-kinematics-private/color_samples/color_yellow.png", "y", output_csv_path="colors_yellow.csv")
plot_color_distribution("/home/audrius/opw/rs-opw-kinematics-private/color_samples/color_metal.png", "x", output_csv_path="colors_metal.csv")
plot_color_distribution("/home/audrius/opw/rs-opw-kinematics-private/color_samples/color_other.png", "x", output_csv_path="colors_other.csv")
plot_color_distribution("/home/audrius/opw/rs-opw-kinematics-private/color_samples/color_black.png", "x", output_csv_path="colors_black.csv")
plot_color_distribution("/home/audrius/opw/rs-opw-kinematics-private/color_samples/color_copper.png", "x", output_csv_path="colors_copper.csv")
plot_color_distribution("/home/audrius/opw/rs-opw-kinematics-private/color_samples/color_magenta.png", "m", output_csv_path="colors_magenta.csv")
#plot_color_distribution("/home/audrius/opw/rs-opw-kinematics-private/color_samples/color_magenta_s.png", "m", output_csv_path="colors_magenta_s.csv")


input("Press Enter to continue...")