echo "alpha is set" $2
time ./bin/fp $2 input_pa2/$1.block input_pa2/$1.nets sample_out 
./checker/checker input_pa2/$1.block input_pa2/$1.nets sample_out $2
