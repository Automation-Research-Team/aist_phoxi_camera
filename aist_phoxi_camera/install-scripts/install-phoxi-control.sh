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
    wget https://photoneo.com/files/installer/$1/$2.tar.gz
    tar xvf $2.tar.gz
    bash ./$2.run
#    rm $2.tar.gz $2.run
}

case `lsb_release -sc` in
  "xenial" ) install_photoneo PhoXi/1.4.1 PhotoneoPhoXiControlInstaller-1.4.1-Ubuntu16-STABLE ;;
  "bionic" ) install_photoneo PhoXi/1.4.1 PhotoneoPhoXiControlInstaller-1.4.1-Ubuntu18-STABLE ;;
  "focal"  ) install_photoneo PhoXi/1.4.1 PhotoneoPhoXiControlInstaller-1.4.1-Ubuntu20-STABLE ;;
esac
