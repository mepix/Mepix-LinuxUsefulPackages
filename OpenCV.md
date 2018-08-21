# OpenCV

You can look at the headers or libs installed. pkg-config can tell you where they are:
````
pkg-config --cflags opencv
pkg-config --libs opencv
````
Alternatively you can write a simple program and print the following defs:
````
CV_MAJOR_VERSION
CV_MINOR_VERSION
````
