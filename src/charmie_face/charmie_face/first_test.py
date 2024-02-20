import os
import shutil

"""
Info to get device host:
    1- Go to the device location and open the terminal there
    2- write pwd to get the location
"""
destination = "/run/user/1000/gvfs/mtp:host=SAMSUNG_SAMSUNG_Android_52037149ea96c3a1/SanDisk SD card/temp/"

def save_text_file(type, data = ""):
    with open(destination + "temp.txt", "w") as file:
        file.write(f"{type}|{data}|")

def get_filename(file_name, extension):
    if not os.path.exists(destination + file_name + extension):
        return f"{file_name}{extension}"
    
    count = 1
    while True:
        new_filename = f"{file_name}_{count}{extension}"
        if not os.path.exists(destination + new_filename):
            return new_filename
        count += 1


def copy_file(source, file_name):
    shutil.copyfile(source, destination + file_name)

def classify_file(file_extension):
    if file_extension.lower() in ['.jpg', '.jpeg', '.png', '.bmp', '.gif']:
        return "img"
    elif file_extension.lower() in ['.mp4', '.avi', '.mkv', '.mov']:
        return "video"
    else:
        return "Unknown"

def main():
    while True:
        print("Press 1 to save text, 2 to load an file, 3 to upload an file.")
        choice = input("Your choice: ")

        if choice == '1':
            text_input = input("Enter the text: ")
            save_text_file("text", text_input)
            print("File saved successfully.")

        elif choice == '2':
            file_location = input("Enter the file name: ")

            file_name, file_extension = os.path.splitext(file_location)
            file_type = classify_file(file_extension)

            if file_type == 'Unknown':
                print("File type unknown. Please enter a valid file.")
                continue

            save_text_file(file_type, "media/" + file_name + file_extension)
            print("File saved successfully.")

        elif choice == '3':
            file_location = input("Enter the location of the file: ")

            if not os.path.exists(file_location):
                print("File not found. Please enter a valid location.")
                continue

            file_name, file_extension = os.path.splitext(os.path.basename(file_location))
            file_type = classify_file(file_extension)

            if file_type == 'Unknown':
                print("File type unknown. Please enter a valid file.")
                continue

            new_filename = get_filename(file_name, file_extension)

            copy_file(file_location, new_filename)
            save_text_file(file_type, "temp/" + new_filename)
            print("File copied successfully.")


        elif choice.lower() == 'exit':
            break

        else:
            print("Invalid choice. Please enter 1, 2, 3, or 'exit'.")

if __name__ == "__main__":
    main()