#!/bin/bash

VIDEO="/home/Steven/Spectrum Atlas Offcial Video.mp4"

while true; do
    cvlc --play-and-exit --fullscreen "$VIDEO"
    sleep 120
done

