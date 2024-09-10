#!/usr/bin/env python3
"""
A pedagogical implementation of a priority queue
"""

from numbers import Number


class PriorityQueue:
    """ Implements a priority queue """

    def __init__(self):
        """
        Initializes the internal attribute  queue to be an empty list.
        """
        self.queue_list = []

    def check(self):
        """
        Check that the internal representation is a list of (key,value) pairs,
        where value is numerical
        """
        is_valid = True
        for pair in self.queue_list:
            if len(pair) != 2:
                is_valid = False
                break
            if not isinstance(pair[1], Number):
                is_valid = False
                break
        return is_valid

    def insert(self, key, cost):
        """
        Add an element to the queue.
        """
        pass  # Substitute with your code

    def min_extract(self):
        """
        Extract the element with minimum cost from the queue.
        """
        pass  # Substitute with your code
        return key, cost

    def is_member(self, key):
        """
        Check whether an element with a given key is in the queue or not.
        """
        pass  # Substitute with your code
        return flag
