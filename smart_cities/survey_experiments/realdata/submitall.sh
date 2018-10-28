#!/bin/bash

for i in `seq 1 $1`; do
   qsub palmetto.txt
   #sleep 1
done
