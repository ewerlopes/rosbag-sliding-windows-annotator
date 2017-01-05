#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Ewerton Lopes
# Politecnico di Milano, December, 2016.
import unittest
import csv
from data_summary import getCSV


class CSVRetrievalTestCase(unittest.TestCase):
    """Tests the data retrieval for `data_summary.py`"""

    def loadCSV(self, filename):
        """
        Opens csv and return the data as a dictionary
        :param filename: the csv file to be open
        :return: the dictionary whose key is the column name in the csv and
                 the value is a list with the data for the column.
        """
        reader = csv.DictReader(open(filename))
        csv_data = {}
        for row in reader:
            for col, value in row.iteritems():
                csv_data.setdefault(col, []).append(value)
        return csv_data

    def test_retrieval(self):
        """Is the list of windows successfully retrieved?"""
        filename = "csv_test.csv"
        loaded_csv = self.loadCSV(filename)
        result = getCSV(filename)
        csv_windows_separator = ""

        ## Checks whether the getCSV return is the same
        # as the original data. That is, checks weather
        # the windows retrieval is correct.
        for key in result.keys():
            restored = []
            for l in result[key]:
                restored += l + [csv_windows_separator]
            self.assertEquals(restored, loaded_csv[key])

    def test_reference_error(self):
        """Does it throws a ValueError when the referenceColumn is not in the csv?"""
        filename = "csv_test.csv"
        reference_column = "ThisFails"

        with self.assertRaises(ValueError):
            _ = getCSV(filename, reference_column=reference_column)

    def test_separator_error(self):
        """Does it throws a ValueError when the windows separator is not in the csv?"""
        filename = "csv_test.csv"

        with self.assertRaises(ValueError):
            _ = getCSV(filename, windows_separator="???")

if __name__ == '__main__':

    unittest.main()
