#!/bin/bash
# This shell script downloads all available sequences of "sync_data" in MS2 dataset 
# if you want to acess the zipped data through website, use below commented lines.

# All sequences of "sync_data" in MS2 dataset
# https://www.dropbox.com/s/65hkrz768n6wnqt/_2021-08-06-10-59-33.tar.bz2?dl=0
# https://www.dropbox.com/s/gith8ciig2eccla/_2021-08-06-11-23-45.tar.bz2?dl=0
# https://www.dropbox.com/s/h03hx66ov057l7k/_2021-08-06-11-37-46.tar.bz2?dl=0
# https://www.dropbox.com/s/yhpyyydi08vwp1p/_2021-08-06-16-19-00.tar.bz2?dl=0
# https://www.dropbox.com/s/sht99tbkwrhxssg/_2021-08-06-16-45-28.tar.bz2?dl=0
# https://www.dropbox.com/s/r75o84a6qayj2p6/_2021-08-06-16-59-13.tar.bz2?dl=0
# https://www.dropbox.com/s/54hcoy2hzs2ausg/_2021-08-06-17-21-04.tar.bz2?dl=0
# https://www.dropbox.com/s/o06b2th99fpgxs7/_2021-08-06-17-44-55.tar.bz2?dl=0
# https://www.dropbox.com/s/n6pieerypk5f77i/_2021-08-13-15-46-56.tar.bz2?dl=0
# https://www.dropbox.com/s/umafrxglzktetgm/_2021-08-13-16-08-46.tar.bz2?dl=0
# https://www.dropbox.com/s/psn3gyks9mwuvyh/_2021-08-13-16-14-48.tar.bz2?dl=0
# https://www.dropbox.com/s/zk2pc0lisaeevtw/_2021-08-13-16-31-10.tar.bz2?dl=0
# https://www.dropbox.com/s/f6fnp41xaj5qw3l/_2021-08-13-16-50-57.tar.bz2?dl=0
# https://www.dropbox.com/s/pqi0bk29z3cisq8/_2021-08-13-17-06-04.tar.bz2?dl=0
# https://www.dropbox.com/s/33jfb59tux847tk/_2021-08-13-21-18-04.tar.bz2?dl=0
# https://www.dropbox.com/s/urc65q9luelfflc/_2021-08-13-21-36-10.tar.bz2?dl=0
# https://www.dropbox.com/s/fhqhja5e3x4g698/_2021-08-13-21-58-13.tar.bz2?dl=0
# https://www.dropbox.com/s/m38zrl2r8nvarot/_2021-08-13-22-03-03.tar.bz2?dl=0
# https://www.dropbox.com/s/g4j375qx7yqfv7a/_2021-08-13-22-16-02.tar.bz2?dl=0
# https://www.dropbox.com/s/so29g3cgj7e0yed/_2021-08-13-22-36-41.tar.bz2?dl=0

# Uncomment below lines for full dataset
mkdir ./sync_data
cd ./sync_data
wget --tries=2 -c https://dl.dropboxusercontent.com/s/65hkrz768n6wnqt/_2021-08-06-10-59-33.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/gith8ciig2eccla/_2021-08-06-11-23-45.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/h03hx66ov057l7k/_2021-08-06-11-37-46.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/yhpyyydi08vwp1p/_2021-08-06-16-19-00.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/sht99tbkwrhxssg/_2021-08-06-16-45-28.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/r75o84a6qayj2p6/_2021-08-06-16-59-13.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/54hcoy2hzs2ausg/_2021-08-06-17-21-04.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/o06b2th99fpgxs7/_2021-08-06-17-44-55.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/n6pieerypk5f77i/_2021-08-13-15-46-56.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/umafrxglzktetgm/_2021-08-13-16-08-46.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/psn3gyks9mwuvyh/_2021-08-13-16-14-48.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/zk2pc0lisaeevtw/_2021-08-13-16-31-10.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/f6fnp41xaj5qw3l/_2021-08-13-16-50-57.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/pqi0bk29z3cisq8/_2021-08-13-17-06-04.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/33jfb59tux847tk/_2021-08-13-21-18-04.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/urc65q9luelfflc/_2021-08-13-21-36-10.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/fhqhja5e3x4g698/_2021-08-13-21-58-13.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/m38zrl2r8nvarot/_2021-08-13-22-03-03.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/g4j375qx7yqfv7a/_2021-08-13-22-16-02.tar.bz2
# wget --tries=2 -c https://dl.dropboxusercontent.com/s/so29g3cgj7e0yed/_2021-08-13-22-36-41.tar.bz2

tar xvjf _2021-08-06-10-59-33.tar.bz2
# tar xvjf _2021-08-06-11-23-45.tar.bz2
# tar xvjf _2021-08-06-11-37-46.tar.bz2
# tar xvjf _2021-08-06-16-19-00.tar.bz2
# tar xvjf _2021-08-06-16-45-28.tar.bz2
# tar xvjf _2021-08-06-16-59-13.tar.bz2
# tar xvjf _2021-08-06-17-21-04.tar.bz2
# tar xvjf _2021-08-06-17-44-55.tar.bz2
# tar xvjf _2021-08-13-15-46-56.tar.bz2
# tar xvjf _2021-08-13-16-08-46.tar.bz2
# tar xvjf _2021-08-13-16-14-48.tar.bz2
# tar xvjf _2021-08-13-16-31-10.tar.bz2
# tar xvjf _2021-08-13-16-50-57.tar.bz2
# tar xvjf _2021-08-13-17-06-04.tar.bz2
# tar xvjf _2021-08-13-21-18-04.tar.bz2
# tar xvjf _2021-08-13-21-36-10.tar.bz2
# tar xvjf _2021-08-13-21-58-13.tar.bz2
# tar xvjf _2021-08-13-22-03-03.tar.bz2
# tar xvjf _2021-08-13-22-16-02.tar.bz2
# tar xvjf _2021-08-13-22-36-41.tar.bz2

