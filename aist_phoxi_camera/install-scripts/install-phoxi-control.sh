#!/bin/bash

install_photoneo()
{
    cd /tmp
    wget https://photoneo.com/files/installer/$1/$2.zip
    unzip $2.zip
    bash ./$2
    rm $2.zip $2
}

install_photoneo PhoXi/1.2.14 PhotoneoPhoXiControlInstaller-1.2.14-Ubuntu18-STABLE.run
