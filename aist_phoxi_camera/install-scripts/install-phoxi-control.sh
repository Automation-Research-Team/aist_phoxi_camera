#!/bin/bash

install_pkg()
{
    cd /tmp
    wget http://mirrors.kernel.org/ubuntu/pool/main/$1/$2
    dpkg -i $2
    rm $2
}

install_photoneo()
{
    cd /tmp
    wget https://photoneo.com/files/installer/$1/$2$3
    tar xvf $2$3 -o $2.run
    bash ./$2.run
    rm $2$3 $2.run
}

if [ `lsb_release -sc` != "kinetic" ]; then
    install_pkg i/icu libicu55_55.1-7ubuntu0.5_amd64.deb
fi

install_photoneo PhoXi/1.2.26 PhotoneoPhoXiControlInstaller-1.2.26-Ubuntu 16-STABLE.tar.gz
