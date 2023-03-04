import os
# VisionImages\visionImg-556-0_mask.png

folder_path = "VisionImages"
def filter_images_in_folder(folder):
    # for filename in os.listdir(folder):
    #     if "Raw" in filename:
    #         file_path = f"{folder}\{filename}"
    #         os.remove(file_path)
    for i in range(100):
        filename = os.listdir(folder)[i]
        file_path = f"{folder}\{filename}"
        os.remove(file_path)


filter_images_in_folder(folder_path)