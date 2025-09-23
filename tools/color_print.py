'''
Author       : AXIBA leolihao@arizona.edu
Date         : 2025-03-20 10:00:00
FilePath     : /undefinedd:/Github/tools/color_print.py
Description  : Utility functions for colorful console output
Copyright (c) 2025 by AXIBA (leolihao@arizona.edu), All Rights Reserved.
'''

class Colors:
    # Foreground colors
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'

    # Background colors
    BG_BLACK = '\033[40m'
    BG_RED = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_YELLOW = '\033[43m'
    BG_BLUE = '\033[44m'
    BG_MAGENTA = '\033[45m'
    BG_CYAN = '\033[46m'
    BG_WHITE = '\033[47m'

    # Text styles
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    ITALIC = '\033[3m'

    # Reset
    RESET = '\033[0m'


class TextFormatter:
    """Text formatting utility class"""
    @staticmethod
    def colorize(text, color=Colors.WHITE, style=None):
        """Apply color and optional style to text

        Args:
            text (str): The text to colorize
            color (str): Color code from Colors class
            style (str, optional): Style code from Colors class

        Returns:
            str: Colorized text
        """
        if style:
            return f"{style}{color}{text}{Colors.RESET}"
        return f"{color}{text}{Colors.RESET}"

    @staticmethod
    def _truncate_text(text: str, max_length: int) -> str:
        """Truncate text with ellipsis"""
        if len(text) <= max_length:
            return text
        return text[:max_length-3] + "..."

    @classmethod
    def print_header(
        cls,
        text: str,
        divider: str = '=',
        total_length: int = 120,
        min_side_dashes: int = 3,
        color: Colors = Colors.BLUE,
        style: Colors = Colors.BOLD
    ):
        """
        Print a formatted title

        :param text: Title text
        :param divider: Divider character (default '=')
        :param total_length: Total character length (default 120)
        :param min_side_dashes: Minimum number of equal signs to keep on each side (default 3)
        :param color: Color code (e.g. '\033[94m')
        :param style: Style code (e.g. '\033[1m')
        """
        # Calculate available text space
        max_text_width = total_length - 2 * min_side_dashes - 2  # -2 is for the spaces

        # Process text truncation
        processed_text = cls._truncate_text(text, max_text_width)

        # Build the base string
        base_str = f" {' ' * min_side_dashes} {processed_text} {' ' * min_side_dashes} "

        # Generate the final title
        final_str = base_str.center(total_length, divider)

        print(cls.colorize(final_str, color, style))

    @classmethod
    def print_title(cls, text: str, total_length: int = 120, min_frame_width: int = 50):
        """
        Print a title with a decorative frame

        :param text: The text to display
        :param total_length: Total display length (default 120)
        :param min_frame_width: Minimum frame width (default 50)
        """
        # Calculate actual available frame width
        frame_width = max(
            len(text) + 4,  # 2 spaces on both sides
            min_frame_width
        )

        # Generate decorative elements
        top_line = f"╔{'═' * (frame_width - 2)}╗"
        middle_line = f" ║ {text.center(frame_width - 4)} ║"
        bottom_line = f"╚{'═' * (frame_width - 2)}╝"

        # Print all lines
        cls.print_header(top_line, divider=' ', total_length=total_length)
        cls.print_header(middle_line, divider=' ', total_length=total_length)
        cls.print_header(bottom_line, divider=' ', total_length=total_length)

    @classmethod
    def print_info(cls, text):
        """Print information message in cyan"""
        print(cls.colorize(text, Colors.CYAN))

    @classmethod
    def print_success(cls, text):
        """Print success message in green"""
        print(cls.colorize(text, Colors.GREEN))

    @classmethod
    def print_warning(cls, text):
        """Print warning message in yellow"""
        print(cls.colorize(text, Colors.YELLOW))

    @classmethod
    def print_error(cls, text):
        """Print error message in red"""
        print(cls.colorize(text, Colors.RED, Colors.BOLD))

    @classmethod
    def print_step(cls, step_number, text):
        """Print a numbered step in magenta"""
        print(cls.colorize(f"[{step_number}] {text}", Colors.MAGENTA))

    @classmethod
    def print_command(cls, cmd):
        """Print a command in yellow with bold"""
        print(cls.colorize(f"$ {cmd}", Colors.YELLOW, Colors.BOLD))

    @classmethod
    def custom_header(cls, text: str, level: int = 1):
        """Custom title format"""
        dividers = {
            1: ('=', 120),
            2: ('-', 100),
            3: ('~', 80)
        }

        mapping = {
            1: Colors.BLUE,
            2: Colors.MAGENTA,
            3: Colors.YELLOW
        }

        cls.print_header(text=text, divider=dividers[level][0],
                         total_length=dividers[level][1],
                         color=mapping[level])
