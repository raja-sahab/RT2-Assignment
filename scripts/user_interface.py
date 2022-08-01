#! /usr/bin/env python
"""
.. module:: user_interface
    :platform: Unix
    :synopsis: Python module for implement the user interface.

.. moduleauthor:: Raja Farooq Dilshad

This module implements the user interface of the program.
It is developed with the curses library, dividing the terminal into windows and calling specific functions 
to print strings and receive user inputs. The class windows_organiser stores all information about the user 
interface and implements some methods to make the code slighter.

"""

# Run with: roslaunch final_assignment assignment.launch

import curses   # Library for printing on the whole screen.
import text     # Script with some text.

### CODE ###
class windows_organiser:
    """This class manages the user interface, dividing the terminal into windows.

    """
    def __init__(self):
        """This method initialises the windows_organiser class.

        """
        self.stdscr = curses.initscr()         # Starts the curses standard screen.
        self.stdscr.addstr(0, 0, text.title)   # Prints the title.
        self.stdscr.refresh()                  # Refreshes the screen.

        # Creates the win_modes window for showing the available commands.
        self.win_modes = curses.newwin(14, 36, 9, 0)
        self.win_modes.addstr(0, 0, text.modalities)
        self.win_modes.refresh()

        # Creates the win_info window for printing information.
        self.win_info = curses.newwin(5, 45, 19, 39)
        self.win_info.addstr(0, 0, text.info)
        self.win_info.refresh()

        self.win_input = curses.newwin(1, 5, 23, 0)      # Window for acquiring the user input.
        self.win_request = curses.newwin(1, 35, 22, 0)   # Window for printing an input request.

    def clear_modes(self):
        """This method clears the modes window. 

        """
        self.win_modes.clear()
        self.win_modes.addstr(0, 0, text.modalities)
        self.win_modes.refresh()

    def clear_info(self):
        """This method clears the info window. 

        """
        self.win_info.clear()
        self.win_info.addstr(0, 0, text.info)
        self.win_info.refresh()

    def clear_input(self):
        """This method clears the input window. 

        """
        self.win_input.clear()
        self.win_input.refresh()

    def clear_request(self):
        """This method clears the request window. 

        """
        self.win_request.clear()
        self.win_request.refresh()

    def set_wasd(self):
        """This method shows into the win_modes the command for directly drive the robot.

        """
        self.win_modes.clear()
        self.win_modes.addstr(0, 0, text.wasd)
        self.win_modes.refresh()

    def command_not_valid(self):
        """ This method sends the command not valid message into win_request.

        """
        self.win_request.clear()
        self.win_request.addstr(0, 0, "Command NOT valid")
        self.win_request.refresh()
