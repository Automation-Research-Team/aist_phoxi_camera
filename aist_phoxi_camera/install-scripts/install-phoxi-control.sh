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

case `lsb_release -sc` in
  "xenial" ) install_photoneo PhoXi/1.4.1 PhotoneoPhoXiControlInstaller-1.4.1-Ubuntu 16-STABLE.tar.gz ;;
  "bionic" ) install_photoneo PhoXi/1.4.1 PhotoneoPhoXiControlInstaller-1.4.1-Ubuntu 18-STABLE.tar.gz ;;
  "focal"  ) install_photoneo PhoXi/1.4.1 PhotoneoPhoXiControlInstaller-1.4.1-Ubuntu 20-STABLE.tar.gz ;;
esac
