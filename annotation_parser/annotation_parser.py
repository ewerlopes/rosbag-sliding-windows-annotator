#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Ewerton Lopes
# Politecnico di Milano, December, 2016.
# This program aims at exporting bag data as csv files. This piece of
# software should be used as an extension of the "annotation.py" program (also
# included in this directory). It should receive the bag file and the annotation
# generated as a json file by the annotation.py (for the corresponding bag file, of course).
# Then, it exports the data as a csv.

import argparse, os, sys, time
from PyQt5.QtWidgets import *
from parser_gui import AnnotationGUIParser
from parser_console import AnnotationConsoleParser

def readArgs():
    """ Deals with the argments"""
    parser = argparse.ArgumentParser(description=
                                     """Annotation parser script. This program is going to save a given
                                     rosbag file data as csv file, given the annotation described in its
                                     corresponding json file. It generates a csv file for each feature
                                     perspective, taking into account ther corresponding annotations in
                                     the jason and the topics on it described.
                                     """)

    parser.add_argument('-b', '--bags-dir', metavar='',
                        dest='bags_dir',
                        action='store', default=os.path.dirname(os.path.abspath(__file__)),
                        help='Specify the bag input directory.')
    parser.add_argument('-a', '--annotation_dir', metavar='',
                        dest='annotation_dir', action='store',
                        default=os.path.dirname(os.path.abspath(__file__)),
                        help='the folder containing the annotation json files location.')
    parser.add_argument('-o', '--output_dir', metavar='',
                        dest='output_dir', action='store',
                        default=os.path.dirname(os.path.abspath(__file__)) + "/annotation_to_csv_"
                                +time.strftime("%Y-%m-%d--%H-%M-%S"),
                        help='the folder where to save the extracted csv files.')
    parser.add_argument('-t', '--tolerance', metavar='',
                        dest='tolerance', action='store',
                        default=0.02,
                        help='mismatch tolerance value for deviation in the windows slice')
    parser.add_argument('--gui', dest='gui', action='store_true', default=False,
                        help='whether to display the GUI.')
    return parser.parse_args()


if __name__ == '__main__':
    args = readArgs()
    if args.gui:
        app = QApplication(sys.argv)
        player = AnnotationGUIParser()
        player.resize(1340, QApplication.desktop().screenGeometry().height())
        player.show()
        sys.exit(app.exec_())
    else:
        parser = AnnotationConsoleParser(args.bags_dir, args.annotation_dir, args.output_dir, args.tolerance)
        parser.run()