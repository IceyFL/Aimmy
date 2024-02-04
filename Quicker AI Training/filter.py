import cv2
import os
import numpy as np
import shutil

def display_image_with_label(image_path, label_folder, filtered_images_folder, filtered_labels_folder):
    # Read the image
    image = cv2.imread(image_path)

    # Check if the image is loaded successfully
    if image is None:
        print(f"Error: Unable to read the image from {image_path}")
        return

    # Resize the image to fit the window while maintaining the aspect ratio
    aspect_ratio = image.shape[1] / image.shape[0]
    new_width = int(640)
    new_height = int(640 / aspect_ratio)

    resized_image = cv2.resize(image, (new_width, new_height))

    # Create a canvas with a fixed size of 640x640
    canvas = np.zeros((640, 640, 3), dtype=np.uint8)
    
    # Calculate the position to paste the resized image in the center
    x_offset = (640 - new_width) // 2
    y_offset = (640 - new_height) // 2

    # Paste the resized image onto the canvas
    canvas[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = resized_image

    # Load and draw bounding box information from the corresponding label file
    label_path = os.path.join(label_folder, os.path.splitext(os.path.basename(image_path))[0] + '.txt')
    if os.path.exists(label_path):
        with open(label_path, 'r') as label_file:
            for line in label_file:
                # Parse YOLO format (class, x_center, y_center, width, height)
                class_id, x_center, y_center, width, height = map(float, line.split())

                # Convert YOLO coordinates to absolute coordinates
                x_min = int((x_center - width / 2) * new_width)
                y_min = int((y_center - height / 2) * new_height)
                x_max = int((x_center + width / 2) * new_width)
                y_max = int((y_center + height / 2) * new_height)

                # Draw less bright green bounding box on the image
                cv2.rectangle(canvas, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

    # Display the image with bounding boxes
    cv2.imshow('Image Display', canvas)

def handle_user_input(image_path, label_path, filtered_images_folder, filtered_labels_folder):
    # Wait for key event for a specific duration (in milliseconds)
    key = cv2.waitKey(0) & 0xFF

    if key == ord('n'):
        # Delete the image and label files
        os.remove(image_path)
        os.remove(label_path)
        
    elif key == ord('y'):
        # Move the image to the filtered images folder
        filtered_image_path = os.path.join(filtered_images_folder, os.path.basename(image_path))
        shutil.move(image_path, filtered_image_path)
        # Move the label file to the filtered labels folder
        filtered_label_path = os.path.join(filtered_labels_folder, os.path.basename(label_path))
        shutil.move(label_path, filtered_label_path)

    elif key == ord('p'):
        # Move the image to the filtered images folder
        filtered_image_path = os.path.join(filtered_images_folder, os.path.basename(image_path))
        shutil.move(image_path, filtered_image_path)
        # delete the label
        os.remove(label_path)
        
    else:
        print("Invalid input. Skipping this image.")

    # Close the OpenCV window
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Specify the paths to the folders containing images, labels, and filtered folders
    images_folder_path = 'unfiltered images'
    labels_folder_path = 'unfiltered labels'
    filtered_images_folder_path = 'filtered images'
    filtered_labels_folder_path = 'filtered labels'

    # Create filtered folders if they don't exist
    os.makedirs(filtered_images_folder_path, exist_ok=True)
    os.makedirs(filtered_labels_folder_path, exist_ok=True)

    # Iterate through all files in the folder
    for filename in os.listdir(images_folder_path):
        if filename.endswith(('.jpg', '.jpeg', '.png')):  # Add more image extensions if needed
            image_path = os.path.join(images_folder_path, filename)
            label_path = os.path.join(labels_folder_path, os.path.splitext(os.path.basename(image_path))[0] + '.txt')

            print(f"Displaying image: {os.path.basename(image_path)}")
            display_image_with_label(image_path, labels_folder_path, filtered_images_folder_path, filtered_labels_folder_path)

            # Handle user input after displaying the image
            handle_user_input(image_path, label_path, filtered_images_folder_path, filtered_labels_folder_path)
