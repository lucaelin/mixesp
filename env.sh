docker run --rm --device=/dev/ttyUSB0 -v $PWD:/project -w /project -it espressif/idf

#run
# idf.py build && idf.py flash && idf.py monitor 
#inside the container and ctrl+] to exit 