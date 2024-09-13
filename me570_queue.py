#!/usr/bin/env python3
"""
A pedagogical implementation of a priority queue
"""

from numbers import Number


class PriorityQueue:
    """Implements a priority queue"""

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
        self.queue_list.append((key, cost))

    def min_extract(self):
        """
        Extract the element with minimum cost from the queue.
        """
        if len(self.queue_list) == 0:
            return None, None
        cost = self.queue_list[0][1]
        key = self.queue_list[0][0]
        for pair in self.queue_list:
            if pair[1] < cost:
                cost = pair[1]
                key = pair[0]
        return key, cost

    def is_member(self, key):
        """
        Check whether an element with a given key is in the queue or not.
        """
        for pair in self.queue_list:
            if key == pair[0]:
                return True
        return False
