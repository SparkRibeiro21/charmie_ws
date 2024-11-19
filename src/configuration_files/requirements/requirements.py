import os
import subprocess
import pkg_resources



subprocess.run(['sudo', 'apt-get', 'install', 'python3-tk'],text=True)
import tkinter as tk
from tkinter import messagebox, ttk


from PIL import Image, ImageTk  # Make sure you have Pillow installed

# Define target versions
path_to_ws="/home/martins/Desktop/Charmie/charmie_ws"
ubuntu_target = '22.04'
python_version = "3.10.14"
ultralytics_target = '8.2.20'
tensorflow_target = '2.16.1'
cuda_target = '12.4'
pytorch_target = '2.1.2+cu121'
torchvision = '0.16.2+cu121'

# Check functions
def check_ubuntu_version(target_version):
    with open('/etc/os-release') as f:
        lines = f.readlines()
    for line in lines:
        if 'VERSION_ID' in line:
            installed_version = line.split('=')[1].strip().strip('"')
            return installed_version == target_version
    return False

def check_ultralytics_version(target_version):
    try:
        installed_version = pkg_resources.get_distribution("ultralytics").version
        return installed_version == target_version
    except pkg_resources.DistributionNotFound:
        return False

def check_tensorflow_version(target_version):
    try:
        installed_version = pkg_resources.get_distribution("tensorflow").version
        return installed_version == target_version
    except pkg_resources.DistributionNotFound:
        return False

def check_cuda_version(target_version):
    try:
        import torch
        installed_version = torch.version.cuda
        return installed_version == target_version
    except Exception:
        return False

def check_python_version(target_version):
    try:
        result = subprocess.run(['python3', '--version'], capture_output=True, text=True)
        version_line = [line for line in result.stdout.split('\n') if 'Python' in line][0]
        installed_version = version_line.split('Python')[1].strip()
        return installed_version == target_version
    except Exception:
        return False

def check_pytorch_version(target_version):
    try:
        import torch
        installed_version = torch.__version__
        return installed_version == target_version
    except:
        return False

def check_graphic_drivers():
    result = subprocess.run("nvidia-smi", capture_output=True, text=True)
    return 'NVIDIA-SMI' in result.stdout

# Install functions
def install_python_version(target_version):
    subprocess.run("sudo apt update", shell=True)
    subprocess.run(['sudo', 'apt', 'install', 'software-properties-common', '-y'])
    subprocess.run(['sudo', 'add-apt-repository', 'ppa:deadsnakes/ppa'])
    subprocess.run(['sudo', 'apt', 'update'])
    subprocess.run(['sudo', 'apt', 'install', 'python3.10', 'python3.10-venv', 'python3.10-dev'])
    subprocess.run(['sudo', 'rm', '/usr/bin/python3'])
    subprocess.run(['sudo', 'ln', '-s', 'python3.10', '/usr/bin/python3'])
    return check_python_version(target_version)

def install_pip():
    subprocess.run(['sudo', 'apt', 'install', 'python3-pip'])
    return True

def install_vscode():
    subprocess.run(['sudo', 'snap', 'install', 'code', '--classic'])
    return True

def install_ubuntu22():
    subprocess.run(['sudo', 'apt', 'update'])
    subprocess.run(['sudo', 'apt', 'upgrade'])
    subprocess.run(['sudo', 'apt', 'install', 'ubuntu-release-upgrader-core'])
    subprocess.run(['sudo', 'do-release-upgrade'])
    return check_ubuntu_version(ubuntu_target)

def install_ros2():
    global path_to_ws
    subprocess.run(['sudo', 'apt', 'install', 'software-properties-common'])
    subprocess.run(['sudo', 'add-apt-repository', 'universe'])
    subprocess.run(['sudo', 'apt', 'update', '&&', 'sudo', 'apt', 'install', 'curl', 'gnupg', 'lsb-release', '-y'])
    subprocess.run(['sudo', 'curl', '-sSL', 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key', '-o', '/usr/share/keyrings/ros-archive-keyring.gpg'])
    subprocess.run(['sudo', 'apt', 'update'])
    subprocess.run(['sudo', 'apt', 'upgrade'])
    subprocess.run(['sudo', 'apt', 'install', 'ros-humble-desktop','-y'])
    #subprocess.run(['echo', '"source /opt/ros/humble/setup.bash"', '>>', '~/.bashrc'])
    #RealSense
    subprocess.run(['sudo', 'apt', 'install','ros-humble-realsense2*','-y'])
    subprocess.run(['sudo', 'apt', 'install','ros-humble-realsense2-*','-y'])
    #subprocess.run(['pip', 'install', 'ros-humble-moveit'])
    #ROS2 pkgs
    subprocess.run(['sudo', 'apt', 'install', 'ros-humble-gazebo-ros','-y'])
    subprocess.run(['sudo', 'apt', 'install', 'ros-humble-camera-info-manager','-y'])
    subprocess.run(['sudo', 'apt', 'install', 'ros-humble-control-msgs','-y'])
    subprocess.run(['sudo', 'apt', 'install', 'ros-humble-hardware-interface','-y'])
    subprocess.run(['sudo', 'apt', 'install', 'ros-humble-controller-manager-msgs','-y'])
    subprocess.run(['sudo', 'apt', 'install', 'ros-humble-control-toolbox','-y'])
    subprocess.run(['sudo', 'apt', 'install', 'ros-humble-controller-manager','-y'])
    subprocess.run(['sudo', 'apt', 'install', 'ros-humble-moveit','-y'])
    #subprocess.run(['pip', 'install', 'ros-foxy-xacro', 'ros-foxy-joint-state-publisher-gui'])
    #subprocess.run(['pip', 'install', 'ros-foxy-gazebo-ros-pkgs'])
    #subprocess.run(['pip', 'install', 'ros-foxy-ros2-control', 'ros-foxy-ros2-controllers', 'ros-foxy-gazebo-ros2-control'])
    #colcon
    #sudo apt install python3-colcon-common-extensions -y
    subprocess.run(['sudo', 'apt', 'install', 'python3-colcon-common-extensions', '-y'])
    #subprocess.run(['sudo', 'echo', '"source', '/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"', '>>', '~/.bashrc'])
    bashrc_path = os.path.expanduser('~/.bashrc')
    with open(bashrc_path, 'a') as file:
        # Write the new line to the end of the file
        file.write('\nsource /opt/ros/humble/setup.bash\n')
        file.write('\nsource /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash\n')
        file.write(f'\nsource {path_to_ws}/install/setup.bash\n')
    subprocess.run(['cd', f'{path_to_ws}', '&&', 'colcon', 'build'],shell=True)
    return True

def install_requirements():
    subprocess.run(['pip', 'install', 'pyserial'])
    subprocess.run(['pip', 'install', 'pyPS4Controller'])
    subprocess.run(['pip', 'install', 'dynamixel-sdk'])
    subprocess.run(['pip', 'install', 'keras'])
    subprocess.run(['pip', 'install', 'SpeechRecognition'])
    subprocess.run(['pip', 'install', 'pulsectl'])
    subprocess.run(['sudo', 'apt-get', 'install', 'portaudio19-dev', '-y'])
    subprocess.run(['pip', 'install', 'PyAudio'])
    subprocess.run(['sudo', 'apt', 'update'])
    subprocess.run(['sudo', 'apt', 'install', 'ffmpeg', '-y'])
    subprocess.run(['pip', 'install', 'face_recognition'])
    subprocess.run(['pip', 'install', 'TTS'])
    subprocess.run(['pip', 'install', 'pydub'])
    subprocess.run(['pip', 'install', 'pygame'])
    
    #LLM
    subprocess.run(['pip', 'install', 'openai'])
    subprocess.run(['pip', 'install', 'pymupdf'])
    
    #AUDIO
    subprocess.run(['pip', 'install', 'openai-whisper'])
    subprocess.run(['pip', 'install', 'sounddevice'])




def install_ultralytics():
    subprocess.run(['pip', 'install', f'ultralytics=={ultralytics_target}'])
    return check_ultralytics_version(ultralytics_target)

def install_pytorch():
    pytorch_to_install = pytorch_target.split('+')[0]
    cudatorch_to_install = str(float(pytorch_target.split('+cu')[1]) / 10)
    subprocess.run(['pip', 'install', f'torch=={pytorch_to_install}', f'torchvision=={torchvision}', '-f', 'https://download.pytorch.org/whl/torch_stable.html'])
    return check_pytorch_version(pytorch_target)

def check_all_versions():
    checks = {
        "Ubuntu": check_ubuntu_version(ubuntu_target),
        "Python": check_python_version(python_version),
        "Ultralytics": check_ultralytics_version(ultralytics_target),
        "TensorFlow": check_tensorflow_version(tensorflow_target),
        "Graphic Drivers": check_graphic_drivers(),
        "PyTorch": check_pytorch_version(pytorch_target)
    }
    return checks

# GUI
def create_gui():
    root = tk.Tk()
    root.title("CHARMIE Installation")
    root.geometry("700x500")
    
    style = ttk.Style()
    style.configure('TButton', font=('Helvetica', 12))
    style.configure('TLabel', font=('Helvetica', 12))
    style.configure('TCheckbutton', font=('Helvetica', 12))

    frame = ttk.Frame(root, padding="10")
    frame.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

    # Load and display the robot image
    image = Image.open("Installation.png")
    photo = ImageTk.PhotoImage(image)
    label_image = tk.Label(frame, image=photo)
    label_image.pack()

    def show_main_content():
        label_image.pack_forget()

        checks = check_all_versions()
        
        def install_selected():
            if var_python.get() and not checks["Python"]:
                install_python_version(python_version)
            if var_pip.get():
                install_pip()
            if var_vscode.get():
                install_vscode()
            if var_requi.get():
                install_requirements()
            if var_ultralytics.get():
                install_ultralytics()
            if var_pytorch.get() :
                install_pytorch()
            if var_tensor.get():
                subprocess.run(['pip', 'install', f'tensorflow=={tensorflow_target}'], text=True) 
            if var_ros2.get():
                install_ros2()
            
            messagebox.showinfo("Installation", "Selected installations completed!")
            update_check_status()
    
        def update_check_status():
            checks = check_all_versions()
            for program, status in checks.items():
                labels[program].config(text=f"{program}: {'Correct ✅️' if status else 'Wrong ⚠️'}",
                                       foreground='green' if status else 'red')

        var_python = tk.BooleanVar()
        var_pip = tk.BooleanVar()
        var_vscode = tk.BooleanVar()
        var_requi = tk.BooleanVar()
        var_ros2 = tk.BooleanVar()
        var_ultralytics = tk.BooleanVar()
        var_tensor = tk.BooleanVar()
        var_pytorch = tk.BooleanVar()

        ttk.Label(frame, text="System Setup", font=("Helvetica", 16)).pack(anchor=tk.W, pady=10)

        labels = {}
        for program, status in checks.items():
            labels[program] = ttk.Label(frame, text=f"{program}: {'Correct ✅️' if status else 'Wrong ⚠️'}",
                                        foreground='green' if status else 'red')
            labels[program].pack(anchor=tk.W)

        ttk.Label(frame, text="Select programs to install:").pack(anchor=tk.W, pady=10)

        ttk.Checkbutton(frame, text="Install Python", variable=var_python).pack(anchor=tk.W)
        ttk.Checkbutton(frame, text="Install Pip", variable=var_pip).pack(anchor=tk.W)
        ttk.Checkbutton(frame, text="Install VSCode", variable=var_vscode).pack(anchor=tk.W)
        ttk.Checkbutton(frame, text="Install Ultralytics", variable=var_ultralytics).pack(anchor=tk.W)
        ttk.Checkbutton(frame, text="Install Tensor", variable=var_tensor).pack(anchor=tk.W)
        ttk.Checkbutton(frame, text="Install Requirements", variable=var_requi).pack(anchor=tk.W)
        ttk.Checkbutton(frame, text="Install PyTorch", variable=var_pytorch).pack(anchor=tk.W)
        ttk.Checkbutton(frame, text="Install ROS2", variable=var_ros2).pack(anchor=tk.W)
        
        ttk.Button(frame, text="Check Versions", command=update_check_status).pack(pady=5)
        ttk.Button(frame, text="Install Selected", command=install_selected).pack(pady=5)

    # Schedule the main content to be shown after 2 seconds
    root.after(2000, show_main_content)
    
    root.mainloop()

if __name__ == "__main__":
    create_gui()

