#!/usr/bin/gnuplot -persist

# Gnuplot script file for plotting angular velocity data

# This script can be run with the command: ./scriptname filename
# to produce an output file 

# A data file should have the format: "X.XXX X.XXX X.XXX X.XXX ... " per line, 
# where the columns are

# 1: Time since start of experiment (s)
# 2: PWM left motor -255..255
# 3: PWM right motor -255..255
# 4: Encoder Angular velocity left motor (rad/sec)
# 5: Encoder Angular velocity right motor (rad/sec)
# 6: Desired angular velocity left motor (rad/sec)
# 7: Desired angular velocity right motor (rad/sec)

# plot-independent gnuplot parameters
set time                               # date/time in lower-left corner
set autoscale                          # scale axes automatically
set xtic auto                          # set xtics automatically
set ytic auto                          # set ytics automatically
set size 1.0, 1.0                 # set the plot size


# plot-dependent gnuplot parameters
set title "Proportional Controller Behavior for Fixed Gain" # plot title
set xlabel "Time (s)"   # set the x-axis label
set ylabel "Motor Angular Velocity (radians/second)" # set the y-axis label
set terminal postscript enhanced mono lw 2 "Helvetica" 14

set out "plot.ps"       # plot filename
# time on X axis, commanded, measured, and error velocities on Y axis
plot "data.txt" using 1:7 title 'Commanded Velocity' with lines,\
     "data.txt" using 1:5 title 'Measured  Velocity' with lines
#    "data.txt" using 1:($5-$7) title 'Velocity Error (M - C)' with lines

