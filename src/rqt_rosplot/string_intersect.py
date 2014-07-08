#!/usr/bin/env python

# Function to return a string up to the last matching letter between the two strings given as arguments
def string_intersect(str1, str2):
        newlist = []
        for i,j in zip(str1, str2):
            if i == j:
                newlist.append(i)
            else:
                break
        return ''.join(newlist)