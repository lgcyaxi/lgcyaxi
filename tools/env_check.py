'''
Author       : Temporary User leolihao@arizona.edu
Date         : 2025-03-30 10:46:59
FilePath     : /EasyCalib/tools/env_check.py
Description  : 
Copyright (c) 2025 by Temporary User (leolihao@arizona.edu), All Rights Reserved.
'''
import sys
import subprocess
from tools.color_print import print_error, print_info, colorize, Colors, print_warning, print_success

def check_pixi_env():
    """Check if the pixi environment is activated"""
    # Check 1: Can we run pixi commands
    try:
        pixi_version = subprocess.check_output(["pixi", "--version"], stderr=subprocess.PIPE).decode().strip()
        print_info(f"Pixi version: {colorize(pixi_version, Colors.CYAN)}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print_error("pixi command not found or not working")
        print_info("Please run 'pixi shell' to activate the pixi environment first")
        sys.exit(1)

    # Check 2: Are we using the pixi Python interpreter?
    python_path = sys.executable
    pixi_paths = ['.pixi', 'pixi/envs']  # Common markers in pixi Python paths
    if not any(marker in python_path for marker in pixi_paths):
        print_warning(f"Current Python interpreter may not be from pixi environment: {python_path}")
        print_info("The script may not have access to packages installed via pixi")
        print_info("Please ensure you activated the environment with 'pixi shell'")
        response = input(colorize("Continue anyway? (y/n): ", Colors.YELLOW))
        if response.lower() != 'y':
            sys.exit(1)
    else:
        print_success(f"Using pixi Python interpreter: {colorize(python_path, Colors.CYAN)}")
        
def check_pixi_package_installed(package_name):
    """Check if a package is installed in the pixi environment"""
    try:
        # Use pixi list and grep for the package name
        output = subprocess.check_output(["pixi", "list"]).decode()
        return package_name in output
    except subprocess.CalledProcessError:
        return False
    except FileNotFoundError:
        # pixi command not found
        print_error("pixi command not found. Are you running in a pixi environment?")
        return False
    
def check_python_version(version="3.10"):
    """Check if the python version is supported"""
    try:
        # Get the python version
        output = subprocess.check_output(["python", "--version"]).decode()
        return version in output
    except subprocess.CalledProcessError:
        print_error("Python version is not supported. Please use Python " + version + ".")
        return False
    except FileNotFoundError:
        print_error("Python command not found. Are you running in a Python environment?")
        return False
