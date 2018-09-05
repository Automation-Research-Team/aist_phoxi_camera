#!/usr/bin/env python

import os
import csv
import codecs
from collections import OrderedDict
import yaml

import rospy
import rospkg

rp = rospkg.RosPack()

def represent_odict(dumper, instance):
    return dumper.represent_mapping('tag:yaml.org,2002:map', instance.items())

yaml.add_representer(OrderedDict, represent_odict)

def main():
    kitting_list = OrderedDict()
    
    with open("ExampleOfSetListFile.csv", 'r') as csvfile:
        reader = csv.reader(csvfile)
        header = next(csvfile)
        term_number = 0
        # term_items = []
        for data in reader:
            if int(data[0]) != term_number:
                term_number += 1
                kitting_list['set_' + str(term_number)] = list()
            item_data = OrderedDict()
            item_data['num'] = data[1]
            item_data['id'] = data[2]
            item_data['name'] = data[3]
            kitting_list['set_' + str(term_number)].append(item_data)
            
    with open("kitting_item_list.yaml", 'w') as yamlfile:
        yaml.dump(kitting_list, yamlfile)


if __name__ == '__main__':
    
    main()

    exit(0)
