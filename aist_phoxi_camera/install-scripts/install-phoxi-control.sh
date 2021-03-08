#!/bin/bash

install_pkg()
{
    cd /tmp
    curl http://mirrors.kernel.org/ubuntu/pool/main/$1/$2
    dpkg -i $2
    rm $2
}

install_photoneo()
{
    cd /tmp
    curl https://photoneo.com/files/installer/$1/$2.run.zip
    unzip $2.run.zip
    bash ./$2.run
    rm $2.run.zip $2.run
}

if [ `lsb_release -sc` != "kinetic" ]; then
    install_pkg i/icu libicuu55_55.1-7ubuntu0.5_amd64.deb
fi

install_photoneo PhoXi/1.2.26 PhotoneoPhoXiControlInstaller-1.2.26-Ubuntu16-STABLE
