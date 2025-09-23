'''
Author       : Lihao leolihao@arizona.edu
Date         : 2024-10-27 14:50:35
FilePath     : /EasyCalib/tools/dump_structure.py
Description  : 
Copyright (c) 2024 by Lihao (leolihao@arizona.edu), All Rights Reserved.
'''
import argparse
import os

# List of directories/files to ignore
ignore_list = ["node_modules", "__pycache__", "backup", ".git", "postgres"]


def generate_folder_structure(path, level=0, max_depth=2, prefix=""):
    """
    Recursively prints the directory structure with improved formatting.

    Parameters:
    - path (str): The directory path to traverse.
    - level (int): The current depth level.
    - max_depth (int): The maximum depth of the directory tree to display.
    - prefix (str): Prefix for continuing lines in the structure display.
    """
    if level > max_depth or os.path.basename(path) in ignore_list or os.path.basename(path).startswith("."):
        return

    # print the root directory on the first level
    if level == 0:
        print(f"{prefix}{os.path.basename(path)}")

    # Get all items, ignoring specified directories and hidden files
    items = [item for item in os.listdir(
        path) if item not in ignore_list and not item.startswith(".")]
    items.sort()

    # Traverse items in the directory
    for i, item in enumerate(items):
        # Determine if this is the last item at the current level
        is_last = i == len(items) - 1
        branch = "└── " if is_last else "├── "
        item_path = os.path.join(path, item)

        # Print the current item
        print(f"{prefix}{branch}{item}")

        # Prepare the new prefix for sub-items
        new_prefix = prefix + ("    " if is_last else "│   ")

        # Recur for directories
        if os.path.isdir(item_path):
            generate_folder_structure(
                item_path, level + 1, max_depth, new_prefix)


if __name__ == "__main__":
    # Set project root path
    project_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), ".."))

    # use argparse to get the max depth
    parser = argparse.ArgumentParser(
        description="Dump the folder structure of the project")
    parser.add_argument("--max-depth", type=int, default=3,
                        help="The maximum depth of the directory tree to display")

    # use argparse to specify the directory in the project to dump
    parser.add_argument("--dir", type=str, default="",
                        help="The directory in the project to dump")
    args = parser.parse_args()

    # generate the folder structure
    dir = os.path.join(project_root, args.dir)
    if not os.path.exists(dir):
        print(f"The directory {dir} does not exist")
        exit(1)
    generate_folder_structure(
        dir, max_depth=args.max_depth)