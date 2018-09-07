#!/usr/bin/env python

import os
import csv
import yaml

import rospy
import rospkg

rp = rospkg.RosPack()


def main():

    kitting_list = dict()
    
    with open(os.path.join(rp.get_path("o2as_routines"),"config", "ExampleOfSetListFile.csv"), 'r') as csvfile:
        reader = csv.reader(csvfile)
        header = next(csvfile)
        for data in reader:
            kitting_list["set_"+data[0]] = dict() if kitting_list["set_"+data[0]] is None
            kitting_list["set_"+data[0]]["part_"+ data[2]] = data[3]
            
    with open(os.path.join(rp.get_path("o2as_routines"), "config", "kitting_item_list.yaml"), 'w') as yamlfile:
        yaml.dump(kitting_list, yamlfile)


if __name__ == '__main__':
    main()
