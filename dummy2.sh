#/bib/bash

# dummy data generator

port="$1"

a=0
f=0
j=0

while [ 1 ]
do
  
  a=$(echo "scale=0; (-$RANDOM * 0.1)/1" | bc)
  f=$(echo "scale=0; ($RANDOM * 0.1)/1" | bc)
  j=$(echo "scale=0; ($RANDOM * 1111.1)/1" | bc)
  printf "a:%d,f:%d,j:%d\n" $a $f $j > /dev/ttyS0
  sleep 0.5
done
