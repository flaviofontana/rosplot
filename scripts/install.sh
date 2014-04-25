#!/bin/bash

chmod u+x rosplot.py
chmod u+x rosplot_autocomplete
chmod u+x rosplot_autocomplete_helper

sudo apt-get install python-qwt5-qt4

cur_dir=`pwd`
sudo ln -s ${cur_dir}/rosplot.py /usr/local/bin/rosplot

sudo cp rosplot_autocomplete /etc/bash_completion.d/
bash

