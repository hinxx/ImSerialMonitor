#/bib/bash

# dummy data generator

port="$1"

a=0
b=0
c=0
d=0

while [ 1 ]
do
  a=$((a+1))
  b=$((a+13))
  c=$((b+9))
  d=$((c+3))
  printf "a:%04d,b:%04d,c:%04d,d:%04d\n" $a $b $c $d > /dev/ttyS0
  sleep 0.5
done

